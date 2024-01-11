import argparse
import socket
import struct
import csv
from threading import Event, Thread

DEF_PORT = 3333
NUM_SAMPLES_PER_CHIRP = 128  # Adjust according to your ESP32 code
BATCH_SIZE = 15

class UdpServer:
    def __init__(self, port, family_addr, persist=False, timeout=60):
        self.port = port
        self.family_addr = family_addr
        self.socket = socket.socket(family_addr, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(timeout)
        self.shutdown = Event()
        self.persist = persist
        self.server_thread = None
        self.csv_file = None
        self.csv_writer = None

    def __enter__(self):
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print(f'Bind failed: {e}')
            raise

        print(f'Starting server on port={self.port} family_addr={self.family_addr}')
        self.csv_file = open('output.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.server_thread = Thread(target=self.run_server)
        self.server_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown.set()
        if self.persist:
            sock = socket.socket(self.family_addr, socket.SOCK_DGRAM)
            sock.sendto(b'Stop', ('localhost', self.port))
            sock.close()
        self.server_thread.join()
        self.socket.close()
        self.csv_file.close()

    def run_server(self):
        while not self.shutdown.is_set():
            try:
                # Adjust buffer size for BATCH_SIZE radar_data_t structures
                buffer_size = (8 + 2 * NUM_SAMPLES_PER_CHIRP) * BATCH_SIZE
                data, addr = self.socket.recvfrom(buffer_size)
                if not data:
                    continue

                # Unpack the data for each radar_data_t in the batch
                for i in range(BATCH_SIZE):
                    offset = i * (8 + 2 * NUM_SAMPLES_PER_CHIRP)
                    format_string = '=q' + 'H' * NUM_SAMPLES_PER_CHIRP  # 'q' for int64 (timestamp), 'H' for uint16 (radar values)
                    unpacked_data = struct.unpack_from(format_string, data, offset)

                    # Write timestamp and radar values to CSV
                    self.csv_writer.writerow(unpacked_data)

                reply = 'OK'
                self.socket.sendto(reply.encode(), addr)
            except socket.timeout:
                print(f'socket recvfrom timeout ({self.socket.timeout}s)')
            except socket.error as e:
                print(f'Running server failed: {e}')
                break

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=DEF_PORT, type=int, help='UDP server port')
    parser.add_argument('--ipv6', action='store_true', help='Create IPv6 server.')
    parser.add_argument('--timeout', default=10, type=int, help='socket recvfrom timeout.')
    args = parser.parse_args()

    family = socket.AF_INET6 if args.ipv6 else socket.AF_INET

    with UdpServer(args.port, family, persist=True, timeout=args.timeout):
        input('Server Running. Press Enter or CTRL-C to exit...\n')

if __name__ == '__main__':
    main()
