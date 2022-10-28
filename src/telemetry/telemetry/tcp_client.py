import asyncio
from threading import Event

import rclpy

from message_handler import parse_message, SoftPing, HardPing, InvalidMessage


class TCPClient(rclpy.node.Node):
    BUFFER_SIZE = 128
    FAIL_DELAY = 2

    def __init__(self):
        self.writer: asyncio.StreamWriter = None
        # For other processes that need to access the writer,
        # This will stop them if the writer is None
        self.writer_connected = Event()

        asyncio.create_task(self.main_loop())

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
                    continue

                if isinstance(result, (SoftPing, HardPing)):
                    self.writer.write(data)
                    continue

            self.writer_connected.clear()
            self.writer = None
            self.get_logger().info("TCP Connection lost. Reconnecting...")

    async def connect(self):
        return await asyncio.open_connection(
            self.get_parameter("server_addr").get_parameter_value().string_value,
            self.get_parameter("port").get_parameter_value().int_value,
        )


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(TCPClient())
