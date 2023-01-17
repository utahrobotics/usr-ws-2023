import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from pyvesc import VESC
from threading import Event, Thread
from mining_arm.angle_sense import AngleSensor

from global_msgs.action import SetArmAngle
from std_msgs.msg import Float32


class MiningArm(Node):
    ANGLE_READ_RATE = 20    # Hz

    def __init__(self):
        super().__init__("mining_arm")

        self.arm_motor = VESC(serial_port=self.get_parameter("arm_motor_port").get_parameter_value().string_value)
        self.digger_motor = VESC(serial_port=self.get_parameter("digger_motor_port").get_parameter_value().string_value)

        self.arm_angle_pub = self.create_publisher(
            Float32,
            "arm_angle"
        )
        self.arm_angle = 0
        self.arm_angle_update_event = Event()
        self.updating_arm_angle = True

        def update_angle():
            angle_sensor = AngleSensor()
            rate = self.create_rate(self.ANGLE_READ_RATE)

            while self.updating_arm_angle:
                self.arm_angle = angle_sensor.get_angle()
                self.arm_angle_update_event.set()
                self.arm_angle_update_event.clear()
                self.arm_angle_pub.publish(Float32(data=self.arm_angle))
                rate.sleep()

        Thread(
            target=update_angle
        ).start()

        self.setting_arm_angle = False
        self.cancel_set_arm_angle = False
        self.set_cancellation = Event()

        self.set_arm_angle_server = ActionServer(
            self,
            SetArmAngle,
            "set_arm_angle",
            self.set_arm_angle_callback,
        )

        self.is_arm_vel_set = False
        self.set_arm_velocity_sub = self.create_subscription(
            Float32,
            "set_arm_velocity",
            self.set_arm_velocity
        )

        self.set_digger_velocity_sub = self.create_subscription(
            Float32,
            "set_digger_velocity",
            self.set_digger_velocity
        )

        self.arm_motor.start_hearbeat()
        self.digger_motor.start_heartbeat()
    
    def close(self):
        self.updating_arm_angle = False
        self.arm_motor.stop_heartbeat()
        self.digger_motor.stop_heartbeat()
    
    def set_arm_angle_callback(self, goal_handle: ServerGoalHandle):
        if self.setting_arm_angle:
            self.cancel_set_arm_angle = True
            self.set_cancellation.clear()
            self.set_cancellation.wait()
        
        self.setting_arm_angle = True
        self.cancel_set_arm_angle = False
        self.is_arm_vel_set = False

        less_than = self.arm_angle < goal_handle.request.target_angle

        # TODO check values
        self.arm_motor.set_duty_cycle(1 if less_than else -1)

        while True:
            self.arm_angle_update_event.wait()

            diff = goal_handle.request.target_angle - self.arm_angle
            feedback = SetArmAngle.Feedback()
            feedback.difference = diff
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested() or self.is_arm_vel_set or self.cancel_set_arm_angle:
                self.set_cancellation.set()
                self.setting_arm_angle = False

                if goal_handle.is_cancel_requested():
                    self.arm_motor.set_duty_cycle(0)

                goal_handle.canceled()
                return SetArmAngle.Result()
            
            if less_than:
                if diff >= 0:
                    break
            elif diff <= 0:
                break

        goal_handle.succeed()
        self.arm_motor.set_duty_cycle(0)
        self.setting_arm_angle = False

        return SetArmAngle.Result()

    def set_arm_velocity(self, msg):
        if not -1 <= msg.data <= 1:
            self.get_logger().error(f"Received out of bounds arm velocity: {msg.data}")
            return
        self.is_arm_vel_set = True
        self.arm_motor.set_duty_cycle(msg.data)

    def set_digger_velocity(self, msg):
        if not -1 <= msg.data <= 1:
            self.get_logger().error(f"Received out of bounds digger velocity: {msg.data}")
            return
        self.digger_motor.set_duty_cycle(msg.data)


def main():
    rclpy.init()
    node = MiningArm()
    rclpy.spin(node)
    node.close()


if __name__ == "__main__":
    main()
