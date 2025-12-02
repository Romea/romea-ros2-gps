# flake8: noqa Q000

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence

from .ntrip_client import NtripClient


def deg_from_degmin(val: str, nb_deg_digits):
    ''' Convert the latitude or longitude GGA field to degrees (float) '''
    return float(val[:nb_deg_digits]) + float(val[nb_deg_digits:]) / 60.


class NtripNode:

    def __init__(self):
        self.node = Node('ntrip')
        self.node.declare_parameter('type', 'ntrip')
        self.node.declare_parameter('host', 'caster.centipede.fr')
        self.node.declare_parameter('port', 2101)
        self.node.declare_parameter('user', 'centipede')
        self.node.declare_parameter('password', 'centipede')
        self.node.declare_parameter('mountpoint', '')
        self.node.declare_parameter('frame_id', 'gps_link')

        self.type = self.node.get_parameter('type').get_parameter_value().string_value
        host = self.node.get_parameter('host').get_parameter_value().string_value
        port = self.node.get_parameter('port').get_parameter_value().integer_value
        user = self.node.get_parameter('user').get_parameter_value().string_value
        password = self.node.get_parameter('password').get_parameter_value().string_value
        self.mountpoint = self.node.get_parameter('mountpoint').get_parameter_value().string_value
        self.frame_id = self.node.get_parameter('frame_id').get_parameter_value().string_value

        self.type = self.type.lower()
        if self.mountpoint == '':
            self.mountpoint = None

        self.nmea_pub = self.node.create_publisher(Sentence, 'nmea_sentence', 30)
        self.ntrip_client = NtripClient(host, port, user, password, self.type == "ntrip")
        self.ntrip_client.set_data_callback(self.send_rtcm_data)

    def run_nmea_reader(self):
        while rclpy.ok():
            data = self.recv_nmea_data()

            sentence = Sentence()
            sentence.header.stamp = self.node.get_clock().now().to_msg()
            sentence.header.frame_id = self.frame_id

            try:
                sentence.sentence = data.decode('ascii')
            except UnicodeError:
                self.node.get_logger().warn(
                    "Skipped reading a line from the serial device because it could not be "
                    "decoded as an ASCII string. The bytes were {0}".format(data))
            else:
                self.nmea_pub.publish(sentence)
                self.detect_position(sentence.sentence)

    def start_ntrip(self):
        self.ntrip_client.connect(self.mountpoint)

    def detect_position(self, sentence):
        if sentence[3:6] == 'GGA':
            parts = sentence.split(',')
            if not parts[2]:
                return

            latitude = deg_from_degmin(parts[2], 2)
            longitude = deg_from_degmin(parts[4], 3)
            self.ntrip_client.set_position(latitude, longitude)
