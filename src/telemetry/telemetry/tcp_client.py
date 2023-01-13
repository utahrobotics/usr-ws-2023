import asyncio

import rclpy
from rclpy.node import Node

from telemetry.message_handler import parse_message, SoftPing, HardPing, IncompleteMessageException
from telemetry.message_handler import InvalidMessage, RemoteMovementIntent
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

        self.movement_intent_pub = self.create_publisher(MovementIntent, 'movement_intent', 10)

        asyncio.run(self.main_loop())

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

                if isinstance(result, InvalidMessage):
                    self.get_logger().error(f"Received invalid message header: {result.header}")

                elif isinstance(result, SoftPing):
                    self.writer.write(data)

                elif isinstance(result, HardPing):
                    self.writer.write(data)
                    # TODO Trigger visual change (maybe we can shimmy the wheels)

                elif isinstance(result, RemoteMovementIntent):
                    msg = MovementIntent()
                    msg.drive = result.drive
                    msg.steering = result.steering
                    self.movement_intent_pub.publish(msg)

            self.writer = None
            self.get_logger().warn("TCP Connection lost. Reconnecting...")

    async def connect(self):
        """
        Connect to the remote server.

        Uses rosparams to determine the address and port to connect to
        """
        return await asyncio.open_connection(
            self.get_parameter("server_addr").get_parameter_value().string_value,
            self.get_parameter("port").get_parameter_value().int_value,
        )


def main():
    rclpy.init()
    rclpy.spin(TCPClient())


if __name__ == "__main__":
    main()
