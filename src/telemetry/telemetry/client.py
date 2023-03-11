import asyncio
from multiprocessing import Process, Value, Pipe
from typing import Union
from asyncio import Event

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from telemetry.message_handler import parse_message, SoftPing, HardPing
from telemetry.message_handler import message_to_bytes
from telemetry.message_handler import IncompleteMessageException
from telemetry.message_handler import RemoteMovementIntent

from std_msgs.msg import Empty
from global_msgs.msg import MovementIntent


class Client(Node):
    """Starts up a TCP socket to communicate with a remote server."""

    # The maximum size of a message that we can parse at once
    BUFFER_SIZE = 128
    # How long to wait to reconnect after a connection failure
    RECONNECTION_DELAY = 2

    def __init__(self):
        super().__init__("telemetry_client")
        main_loop_pipe, self.write_pipe = Pipe(duplex=False)
        self.can_write = Value('b', False)

        self.declare_parameter(
            'port',
            10000,
            ParameterDescriptor(
                description='The port to connect to'
            )
        )

        self.declare_parameter(
            'server_addr',
            "127.0.0.1",
            ParameterDescriptor(
                description='The address to connect to'
            )
        )

        def main_loop(*args):
            asyncio.run(Client.main_loop(*args))

        self.main_loop_process = Process(
            target=main_loop,
            args=(
                self.get_parameter("server_addr")
                    .get_parameter_value()
                    .string_value,
                self.get_parameter("port")
                    .get_parameter_value()
                    .integer_value,
                self.get_logger(),
                main_loop_pipe,
                self.can_write,
                self.create_publisher(
                    Empty,
                    'hard_ping',
                    10
                ),
                self.create_publisher(
                    MovementIntent,
                    'movement_intent',
                    10
                ),
                self.RECONNECTION_DELAY,
                self.BUFFER_SIZE
            )
        )

        self.main_loop_process.start()

    @staticmethod
    async def main_loop(
        server_addr: str,
        port: int,
        logger,
        write_pipe,
        can_write: Value,
        hard_ping_pub,
        movement_intent_pub,
        reconnection_delay: int,
        buffer_size: int
    ):
        """
        Constantly connects to the remote host on TCP and listen for messsages.

        Upon a connection failure, reconnection will always be attempted
        """
        logger.info(f"Client main_loop initiated to {server_addr}:{port}")

        while True:  # outer loop
            while True:  # Connection loop
                try:
                    reader, writer = await Client.connect(server_addr, port)
                    break
                except ConnectionRefusedError:
                    await asyncio.sleep(reconnection_delay)
                except Exception as e:
                    logger.error(f"Error in Client: {e}")
                    await asyncio.sleep(reconnection_delay)

            logger.info("TCP Connection established")

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
                    hard_ping_pub.publish(Empty())

                    try:
                        await send_data(message_to_bytes(result))
                    except BrokenPipeError or ConnectionResetError:
                        break

                elif isinstance(result, SoftPing):
                    logger.info("Received Soft Ping")

                    try:
                        await send_data(message_to_bytes(result))
                    except BrokenPipeError or ConnectionResetError:
                        break

                elif isinstance(result, RemoteMovementIntent):
                    msg = MovementIntent()
                    msg.drive = result.drive
                    msg.steering = result.steering
                    movement_intent_pub.publish(msg)

            with can_write.get_lock():
                can_write.value = False
            writer_coro.cancel()
            logger.warn("TCP Connection lost. Reconnecting...")

    def send_data(self, data: Union[bytes, bytearray]):
        """
        Send the given data to the remote server in a separate Task.

        As such, this method will not block the calling thread
        """
        with self.can_write.get_lock():
            if self.can_write.value:
                self.write_pipe.send(data)

    @staticmethod
    async def connect(server_addr: str, port: int):
        """Connect to the remote server."""
        return await asyncio.open_connection(server_addr, port)


def main():
    rclpy.init()
    client = Client()
    rclpy.spin(client)
    # It is not essential to kill the Process, but it is good practice
    client.main_loop_process.kill()


if __name__ == "__main__":
    main()
