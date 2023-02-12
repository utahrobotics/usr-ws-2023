from pypozyx import *
from pythonosc import *
from pythonosc import udp_client
import rclpy
from rclpy.node import Node

### ALL MEASUREMENT UNITS ARE IN MILLIMETERS

remote = False                                      # true if using given remote device for ranging
use_processing = False                              # true if sending position data through OSC
ip = '127.0.0.1'                                    # replace with IP for the OSC UDP
network_port = 8888                                 # replace with network port for OSC UDP
osc_udp_client = None if not use_processing else udp_client.SimpleUDPClient(ip, network_port)  # is using processing, setup UDP client
remote_id = None if not remote else 0x605D          # replace with netowrk ID of remote device
ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION   # ranging protocol
algorithm = POZYX_POS_ALG_UWB_ONLY                  # positioning algorithm to use
dimension = POZYX_2D                                # positioning dimension (should be 2D for our purposes)
height = 1000                                       # height of device (shouldn't be needed for our purposes, just in case)

# anchors used for positioning, replace device IDs and coordinates
# this assumes anchors are making a 3000mm x 3000mm grid, all at heights of 2000mm
anchors = [DeviceCoordinates(0x971a, 1, Coordinates(0, 0, 2000)),
           DeviceCoordinates(0x972d, 1, Coordinates(3000, 0, 2000)),
           DeviceCoordinates(0x9733, 1, Coordinates(0, 3000, 2000)),
           DeviceCoordinates(0x762a, 1, Coordinates(3000, 3000, 2000))]


class PozyxNode(Node):
    def __init__(self, port):
        self.pozyx = PozyxSerial(port)
        data = []
        self.pozyx.getRead(POZYX_WHO_AM_I, data, remote_id=remote_id)
        self.get_logger().info('who am i: 0x%0.2x' % data[0])
        self.get_logger().info('firmware version: 0x%0.2x' % data[1])
        self.get_logger().info('hardware version: 0x%0.2x' % data[2])
        self.get_logger().info('self test result: %s' % bin(data[3]))
        self.get_logger().info('error: 0x%0.2x' % data[4])
        self.setup()
        self.approx_loc()

    def setup(self):
        self.pozyx.clearDevices(remote_id)
        self.set_anchors_manual()

    
    def set_anchors_manual(self):
        status = self.pozyx.clearDevices(remote_id)
        for anchor in anchors:
            status &= self.pozyx.addDevice(anchor, remote_id)
        if len(anchors > 4):
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors))
        return

    def approx_range(self, anchor_id):
        while True:
            device_range = DeviceRange()
            status = self.pozyx.doRanging(anchor_id, device_range, remote_id)
            if status == POZYX_SUCCESS:
                self.get_logger().info(device_range)
            else:
                self.log_pos_error('ranging')

    def approx_loc(self):
        while True:
            position = Coordinates()
            status = self.pozyx.doPositioning(position, dimension, height, algorithm, remote_id=remote_id)
            if status == POZYX_SUCCESS:
                self.get_logger().info(position)
            else:
                self.log_pos_error('positioning')
    
    def log_pos_error(self, operation):
        error_code = SingleRegister()
        network_id = remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            self.get_logger().info(f'ERROR {operation}, local error code {str(error_code)}')
            if osc_udp_client is not None:
                osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, remote_id)
        if status == POZYX_SUCCESS:
            self.get_logger().info(f'ERROR {operation} on ID {"0x%0.4x" % network_id}, error code {str(error_code)}')
            if osc_udp_client is not None:
                osc_udp_client.send_message("/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            self.get_logger().info(f'ERROR {operation}, couldn\'t retrieve remote error code, local error code {str(error_code)}')
            if osc_udp_client is not None:
                osc_udp_client.send_message("/error", [operation, 0, -1])


def main():
    # port = 'COM1'                             # replace with port of pozyx device
    # port = get_serial_ports()[0].device         # should get serial port automatically?     
    port = get_first_pozyx_serial_port()
    print(port)
    rclpy.init()            
    rclpy.spin(PozyxNode(port))


if __name__ == "__main__":
    main()