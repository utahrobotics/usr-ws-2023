import asyncio
from multiprocessing import Process, Value, Pipe
from asyncio import Event
from typing import Union
from random import randint
from time import sleep
from threading import Thread

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from telemetry.message_handler import parse_message, message_to_bytes
from telemetry.message_handler import SoftPing, HardPing
from telemetry.message_handler import IncompleteMessageException
from telemetry.message_handler import RemoteControl

from std_msgs.msg import Empty, Float32
from global_msgs.msg import MovementIntent


class Server(Node):
    """Starts up a TCP socket to communicate with a remote server."""

    # The maximum size of a message that we can parse at once
    BUFFER_SIZE = 128
    # How long to wait to relisten after a connection failure
    RELISTEN_DELAY = 2

    RATE_LIMIT_DELAY = 0.25

    def __init__(self):
        super().__init__("telemetry_server")
        main_loop_pipe, self.write_pipe = Pipe(duplex=False)
        self.can_write = Value('b', False)

        self.declare_parameter(
            'port',
            10000,
            ParameterDescriptor(
                description='The port to listen on'
            )
        )

        self.movement_intent_sub = self.create_subscription(
            MovementIntent,
            'movement_intent',
            self.on_movement_intent_msg,
            10
        )

        self.arm_vel_sub = self.create_subscription(
            Float32,
            'set_arm_velocity',
            self.on_arm_vel_msg,
            10
        )

        self.drum_vel_sub = self.create_subscription(
            Float32,
            'set_drum_velocity',
            self.on_drum_vel_msg,
            10
        )

        self.soft_ping_sub = self.create_subscription(
            Empty,
            'soft_ping',
            self.on_soft_ping_msg,
            10
        )

        self.hard_ping_sub = self.create_subscription(
            Empty,
            'hard_ping',
            self.on_hard_ping_msg,
            10
        )

        def main_loop(*args):
            asyncio.run(self.main_loop(*args))

        self.main_loop_process = Process(
            target=main_loop,
            args=(
                self.get_parameter("port")
                    .get_parameter_value()
                    .integer_value,
                self.get_logger(),
                main_loop_pipe,
                self.can_write,
                self.RELISTEN_DELAY,
                self.BUFFER_SIZE
            )
        )

        self.main_loop_process.start()

        self.current_drive = 0.0
        self.current_steer = 0.0
        self.current_arm_vel = 0.0
        self.current_drum_vel = 0.0

        Thread(target=self.send_loop).start()

    def send_loop(self):
        while True:
            sleep(self.RATE_LIMIT_DELAY)
            self.send_data(
                message_to_bytes(
                    RemoteControl(
                        self.current_drive,
                        self.current_steer,
                        self.current_arm_vel,
                        self.current_drum_vel
                    )
                )
            )

    def on_movement_intent_msg(self, msg):
        self.current_drive = msg.drive
        self.current_steer = msg.steering

    def on_arm_vel_msg(self, msg):
        self.current_arm_vel = msg.data

    def on_drum_vel_msg(self, msg):
        self.current_drum_vel = msg.data

    def on_soft_ping_msg(self, _msg):
        self.send_data(
            message_to_bytes(
                SoftPing(bytes([randint(0, 255), randint(0, 255)]))
            )
        )

    def on_hard_ping_msg(self, _msg):
        self.send_data(
            message_to_bytes(
                HardPing(bytes([randint(0, 255), randint(0, 255)]))
            )
        )

    @staticmethod
    async def main_loop(
        port: int,
        logger,
        write_pipe,
        can_write: Value,
        relisten_delay: int,
        buffer_size: int
    ):
        """Constantly listens for messages from the client over TCP."""
        logger.info(f"Server main_loop initiated on port: {port}")

        while True:  # outer loop
            while True:  # Connection loop
                try:
                    reader, writer = await Server.get_connection(port)
                    break
                except Exception as e:
                    logger.error(f"Error in Server: {e}")
                    await asyncio.sleep(relisten_delay)

            logger.info(f"TCP Connection established")

            async def send_data(data):
                writer.write(data)
                await writer.drain()
                logger.debug(f"Sent {data}")

            async def writer_task():
                # Turn a thread safe pipe into an async pipe
                # Because they aren't a thing by default
                data_available = Event()
                asyncio.get_running_loop().add_reader(
                    write_pipe.fileno(),
                    data_available.set
                )
                while True:
                    if not write_pipe.poll():
                        await data_available.wait()
                    data_available.clear()
                    data = write_pipe.recv()
                    try:
                        await send_data(data)
                    except BrokenPipeError or ConnectionResetError:
                        break

            writer_coro = asyncio.create_task(writer_task())
            with can_write.get_lock():
                can_write.value = True
            data = bytearray()

            while True:  # Processing loop
                try:
                    tmp = await reader.read(buffer_size)
                except BrokenPipeError:
                    break

                logger.debug(f"Received {tmp}")
                if len(tmp) == 0:  # Connection closed
                    break

                data += tmp

                try:
                    result = parse_message(data)
                except IncompleteMessageException:
                    continue
                except ValueError as e:
                    logger.error(str(e))
                    continue

                if isinstance(result, HardPing):
                    logger.info("Received Hard Ping")

                elif isinstance(result, SoftPing):
                    logger.info("Received Soft Ping")

                elif isinstance(result, RemoteControl):
                    logger.error("Received RemoteControl!?")

            with can_write.get_lock():
                can_write.value = False
            writer_coro.cancel()
            logger.warn("TCP Connection lost. Relistening...")

    def send_data(self, data: Union[bytes, bytearray]):
        """
        Send the given data to the remote client in a separate Task.

        As such, this method will not block the calling thread
        """
        with self.can_write.get_lock():
            if self.can_write.value:
                self.write_pipe.send(data)

    @staticmethod
    async def get_connection(port):
        """Wait for a single connection."""
        data = []

        def on_connection(remote_reader, remote_writer):
            data.append(remote_reader)
            data.append(remote_writer)
            server.close()

        server = await asyncio.start_server(
            on_connection,
            port=port
        )

        try:
            await server.serve_forever()
        except asyncio.CancelledError:
            pass

        if len(data) == 0:
            raise asyncio.CancelledError()

        reader, writer = data
        return reader, writer


def main():
    rclpy.init()
    server = Server()
    rclpy.spin(server)
    # It is not essential to kill the Process, but it is good practice
    server.main_loop_process.kill()


if __name__ == "__main__":
    main()
