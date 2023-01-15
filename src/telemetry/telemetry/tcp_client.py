import asyncio
from multiprocessing import Process
from typing import Union

import rclpy
from rclpy.node import Node

from telemetry.message_handler import parse_message, SoftPing, HardPing
from telemetry.message_handler import IncompleteMessageException
from telemetry.message_handler import RemoteMovementIntent

from std_msgs.msg import Empty
from global_msgs.msg import MovementIntent


class TCPClient(Node):
    """Starts up a TCP socket to communicate with a remote server."""

    # The maximum size of a message that we can parse at once
    BUFFER_SIZE = 128
    # How long to wait to reconnect after a connection failure
    RECONNECTION_DELAY = 2

    def __init__(self):
        super().__init__("tcp_client")
        self.writer: asyncio.StreamWriter = None

        self.movement_intent_pub = self.create_publisher(
            MovementIntent,
            'movement_intent',
            10
        )

        self.hard_ping_pub = self.create_publisher(
            Empty,
            'hard_ping',
            10
        )

        self.main_loop_process = Process(
            target=lambda: asyncio.run(self.main_loop()),
        )

        self.main_loop_process.start()

    async def main_loop(self):
        """
        Constantly connects to the remote host on TCP and listen for messsages.

        Upon a connection failure, reconnection will always be attempted
        """
        while True:  # outer loop
            while True:  # Connection loop
                try:
                    reader, self.writer = await self.connect()
                    break
                except Exception as e:
                    self.get_logger().error(f"Error in TCPClient: {e}")
                    await asyncio.sleep(self.RECONNECTION_DELAY)

            self.get_logger().info("TCP Connection established")

            data = bytearray()

            while True:  # Processing loop
                tmp = await reader.read(self.BUFFER_SIZE)

                if len(tmp) == 0:  # Connection closed
                    break

                data += tmp

                try:
                    result = parse_message(data)
                except IncompleteMessageException:
                    continue
                except ValueError as e:
                    self.get_logger().error(e)

                if isinstance(result, SoftPing):
                    self.send_data(data)

                elif isinstance(result, HardPing):
                    self.send_data(data)
                    self.hard_ping_pub.publish(Empty())

                elif isinstance(result, RemoteMovementIntent):
                    msg = MovementIntent()
                    msg.drive = result.drive
                    msg.steering = result.steering
                    self.movement_intent_pub.publish(msg)

            self.writer = None
            self.get_logger().warn("TCP Connection lost. Reconnecting...")

    def send_data(self, data: Union[bytes, bytearray]):
        self.writer.write(data)
        asyncio.create_task(self.writer.drain())

    async def connect(self):
        """
        Connect to the remote server.

        Uses rosparams to determine the address and port to connect to
        """
        return await asyncio.open_connection(
            self.get_parameter("server_addr")
                .get_parameter_value()
                .string_value,
            self.get_parameter("port")
                .get_parameter_value()
                .integer_value,
        )


def main():
    rclpy.init()
    client = TCPClient()
    rclpy.spin(client)
    # It is not essential to kill the Process, but it is good practice
    client.main_loop_process.kill()


if __name__ == "__main__":
    main()
