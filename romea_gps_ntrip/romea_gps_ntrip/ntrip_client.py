
# flake8: noqa Q000

import socket
from base64 import b64encode
import time
import threading
import dataclasses
import rclpy

from .ntrip_str import NtripSTR

TIMEOUT = 10  # seconds
RECONNECT_DELAY = 3  # seconds
HTTP_USER_AGENT = "NTRIP RTKLIB/2.4.3"
BUFFER_SIZE = 4096


class NtripClient:

    def __init__(self, host, port, user, password, use_ntrip=True):
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.use_ntrip = use_ntrip

        self.mountpoint = None
        self.data_callback = None
        self.position = None

        self.logger = rclpy.logging.get_logger('ntrip_client')

    def _perform_connect(self):
        ''' Connect to the TCP or NTRIP server
        mountpoint (str): the ID of the GNSS sation to connect to
        use_ntrip (boolean): use simple TCP socket or NTRIP protocol
    '''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(TIMEOUT)

        try:
            self.sock.connect((self.host, self.port))
        except OSError as e:
            self.logger.error(f"Failed to connect to server '{self.host}:{self.port}': {e}")
            self.sock.close()
            return False

        if self.use_ntrip:
            http_request = self._build_http_request('/' + self.mountpoint)
            self.sock.send(http_request.encode('utf-8'))

            # check response header
            response = self.sock.recv(BUFFER_SIZE)
            if b'ICY 200 OK' not in response:
                if b'200 OK' in response:
                    self.logger.error(f"Failed to connect to NTRIP server:"
                                      " mountpoint '{self.mountpoint}' is invalid")
                else:
                    self.logger.error(
                        f"The request to the NTRIP server failed:\n{response.decode('utf-8')}")
                self.sock.close()
                return False

            self.logger.info(f"Connected to NTRIP server {self.host}:{self.port}/{self.mountpoint}")

        else:
            self.logger.info(f"Connected to TCP server {self.host}:{self.port}")

        return True

    def _build_http_request(self, url='/'):
        auth = b64encode(f"{self.user}:{self.password}".encode('utf-8')).decode('utf-8')

        # build an HTTP request header to start receiving NTRIP messages
        http_request = f"GET {url} HTTP/1.0\r\n"
        http_request += f"User-Agent: {HTTP_USER_AGENT}\r\n"
        http_request += f"Authorization: Basic {auth}\r\n"
        http_request += "\r\n"

        return http_request

    def _rtcm_task(self):
        while rclpy.ok() and self.use_ntrip and self.mountpoint is None:
            if self.position is not None:
                self.select_closest_mountpoint()
            time.sleep(1)

        if not self._perform_connect():
            return

        # read RTCM3 data from the NTRIP server and send it to the GPS
        while rclpy.ok():
            try:
                buf = self.sock.recv(BUFFER_SIZE)
                # self.logger.info(f'NTRIP: read {len(buf)} bytes')

                if not len(buf):
                    self.logger.warn(f'NTRIP: no data received')
                    continue

                if self.data_callback:
                    self.data_callback(buf)

            except socket.error as e:
                self.logger.warn(f'failed to read RTCM data: {e}')
                while rclpy.ok() and not self._perform_connect():
                    time.sleep(RECONNECT_DELAY)

        self.sock.close()

    def connect(self, mountpoint):
        self.mountpoint = mountpoint
        self.rtcm_thread = threading.Thread(target=self._rtcm_task)
        self.rtcm_thread.start()

    def set_data_callback(self, callback):
        self.data_callback = callback

    def select_closest_mountpoint(self):
        stations = self.load_stations()
        if not len(stations):
            self.logger.error('No GNSS base found')
            return

        station_ranges = map(lambda s: (s.dist(self.position), s), stations)
        station_ranges = list(sorted(station_ranges, key=lambda x: x[0]))

        pos_str = f"[lat: {self.position[0]:5.3f}, lon: {self.position[1]:4.3f}]"
        self.logger.info(f'closest GNSS bases from {pos_str}:')
        for wgs84_dist, station in station_ranges[:min(5, len(station_ranges))]:
            self.logger.info(f"    {wgs84_dist:6.4f} {station.mountpoint} ({station.identifier})")

        selected_station: NtripSTR = station_ranges[0][1]
        self.logger.info("selected base:")
        for key, value in dataclasses.asdict(selected_station).items():
            self.logger.info(f"    {key}: {value}")

        self.mountpoint = selected_station.mountpoint

    def set_position(self, latitude, longitude):
        self.position = (latitude, longitude)

    def load_stations(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)

        try:
            sock.connect((self.host, self.port))
        except OSError as e:
            self.logger.error(f"Failed to connect to server '{self.host}:{self.port}': {e}")
            sock.close()
            return None

        header = self._build_http_request('/')
        sock.sendall(header.encode())

        buf = sock.recv(BUFFER_SIZE)
        data = buf

        if not buf.startswith(b'SOURCETABLE 200 OK\r\n'):
            self.logger.error("Invalid answer for request 'GET /'")
            self.logger.error(f"Received: {buf}")
            sock.close()

        while not buf.endswith(b'ENDSOURCETABLE\r\n'):
            buf = sock.recv(BUFFER_SIZE)
            data += buf

        lines = data.split(b'\r\n')
        stations = []
        for line in lines:
            if line.startswith(b'STR;'):
                station = NtripSTR.from_string(line.decode())
                stations.append(station)
                # print(station.name)

        return stations
