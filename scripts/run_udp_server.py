# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Unlicense OR CC0-1.0
import argparse
import socket
import struct
import csv
from threading import Event, Thread

DEF_PORT = 3333
NUM_SAMPLES_PER_CHIRP = 128


class UdpServer:

    def __init__(self, port, family_addr, persist=False, timeout=60):  # type: ignore
        self.port = port
        self.family_addr = family_addr
        self.socket = socket.socket(family_addr, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(timeout)
        self.shutdown = Event()
        self.persist = persist
        self.server_thread = None

    def __enter__(self):  # type: ignore
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise

        print('Starting server on port={} family_addr={}'.format(self.port, self.family_addr))
        self.server_thread = Thread(target=self.run_server)
        self.server_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):  # type: ignore
        if self.persist:
            sock = socket.socket(self.family_addr, socket.SOCK_DGRAM)
            sock.sendto(b'Stop', ('localhost', self.port))
            sock.close()
            self.shutdown.set()
        self.server_thread.join()
        self.socket.close()

    def run_server(self) -> None:
        csv_file = open('output.csv', 'w', newline='')  
        csv_writer = csv.writer(csv_file)  

        while not self.shutdown.is_set():
            try:
                buffer_size = 8 + 2 * NUM_SAMPLES_PER_CHIRP
                data, addr = self.socket.recvfrom(buffer_size)
                if not data:
                    return

                format_string = '=q' + str(NUM_SAMPLES_PER_CHIRP) + 'H'  # 'q' for 64-bit long long
                unpacked_data = struct.unpack(format_string, data)
                timestamp = unpacked_data[0]
                samples = unpacked_data[1:]

                csv_writer.writerow([timestamp] + list(samples))

                reply = 'OK'
                self.socket.sendto(reply.encode(), addr)
            except socket.timeout:
                print(f'socket recvfrom timeout ({self.socket.timeout}s)')
            except socket.error as e:
                print('Running server failed:{}'.format(e))
                raise
            if not self.persist:
                break

        csv_file.close()  # Close the CSV file when done


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=DEF_PORT, type=int, help='UDP server port')
    parser.add_argument('--ipv6', action='store_true', help='Create IPv6 server.')
    parser.add_argument('--timeout', default=10, type=int, help='socket recvfrom timeout.')
    args = parser.parse_args()

    if args.ipv6:
        family = socket.AF_INET6
    else:
        family = socket.AF_INET

    with UdpServer(args.port, family, persist=True, timeout=args.timeout):
        input('Server Running. Press Enter or CTRL-C to exit...\n')


if __name__ == '__main__':
    main()
