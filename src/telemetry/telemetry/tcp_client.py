import asyncio
from threading import Event

import rclpy
from rclpy.node import Node

from telemetry.message_handler import parse_message, SoftPing, HardPing
from telemetry.message_handler import InvalidMessage, RemoteMovementIntent
from global_msgs.msg import MovementIntent


class TCPClient(Node):
    BUFFER_SIZE = 128
    FAIL_DELAY = 2

    def __init__(self):
        super().__init__("tcp_client")
        self.writer: asyncio.StreamWriter = None
        # For other processes that need to access the writer,
        # This will stop them if the writer is None
        self.writer_connected = Event()

        self.movement_intent_pub = self.create_publisher(MovementIntent, 'movement_intent', 10)

        asyncio.run(self.main_loop())

    async def main_loop(self):
        while True:  # outer loop
            while True:  # Connection loop
                try:
                    reader, self.writer = await self.connect()
                    break
                except Exception as e:
                    self.get_logger().error(f"Error in TCPClient: {e}")
                    await asyncio.sleep(self.FAIL_DELAY)

            self.writer_connected.set()
            self.get_logger().info("TCP Connection established")

            while True:  # Processing loop
                data = await reader.read(self.BUFFER_SIZE)

                if len(data) == 0:  # Connection closed
                    break

                result = parse_message(data)

                if isinstance(result, InvalidMessage):
                    self.get_logger().error(f"Received invalid message header: {result.header}")

                elif isinstance(result, (SoftPing, HardPing)):
                    self.writer.write(data)

                elif isinstance(result, RemoteMovementIntent):
                    msg = MovementIntent()
                    msg.drive = result.drive
                    msg.steering = result.steering
                    self.movement_intent_pub.publish(msg)

            self.writer_connected.clear()
            self.writer = None
            self.get_logger().info("TCP Connection lost. Reconnecting...")

    async def connect(self):
        return await asyncio.open_connection(
            self.get_parameter("server_addr").get_parameter_value().string_value,
            self.get_parameter("port").get_parameter_value().int_value,
        )


def main():
    rclpy.init()
    rclpy.spin(TCPClient())


if __name__ == "__main__":
    main()
