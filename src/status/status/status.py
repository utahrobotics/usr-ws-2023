import asyncio
from dataclasses import dataclass
from typing import Any

from rclpy.node import Node

from global_msgs.srv import DriveStatus


@dataclass
class TotalStatus:
    drive_status: Any


class TotalStatusChecker:
    INIT_TIMEOUT = 3.0
    STATUS_TIMEOUT = 1.0

    def __init__(self, node: Node):
        self.drive_status_req = DriveStatus.Request()
        self.drive_status_client = node.create_client(
            DriveStatus,
            'drive_status'
        )

        if not self.drive_status_client.wait_for_service(
            timeout_sec=self.INIT_TIMEOUT
        ):
            self.drive_status_client = None

    async def get_status(self) -> TotalStatus:
        total_status = TotalStatus()
        futures = []

        if self.drive_status_client is not None:
            async def drive_status():
                try:
                    total_status.drive_status = await asyncio.wait_for(
                        self.drive_status_client.call_async(
                            self.drive_status_req
                        ),
                        timeout=self.STATUS_TIMEOUT
                    )
                except Exception:
                    pass

            futures.append(
                drive_status()
            )

        await asyncio.wait(futures)
        return total_status
