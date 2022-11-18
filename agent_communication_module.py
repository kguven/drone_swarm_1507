import socket
import threading
import time
import udp_server
from datetime import datetime
import json
from pynput import keyboard
import numpy as np

formation = "ARROW"


def on_press(key):
    global formation
    if key == keyboard.Key.esc:
        return False  # stop listener
    if key == keyboard.Key.ctrl:
        formation = "LINE"
    if key == keyboard.Key.alt:
        formation = "ARROW"


class UDPServerMultiClient(udp_server.UDPServer):
    def __init__(self, host, port):
        super().__init__(host, port)
        self.socket_lock = threading.Lock()
        self.swarm_telemetries = dict()

    def handle_request(self, data, client_address):
        # handle request
        global formation
        telemetry_inc_str = data.decode('utf-8')
        telemetry_inc = json.loads(telemetry_inc_str)
        key = "UAV"+str(int(telemetry_inc["id"]))
        self.swarm_telemetries[key] = telemetry_inc
        resp = json.dumps(self.swarm_telemetries)
        if len(self.swarm_telemetries) > 2:
            self.swarm_telemetries["Command"] = formation
        try:
            self.sock.sendto(resp.encode('utf-8'), client_address)
        except:
            print("Udp send exception")

    def wait_for_client(self):
        try:
            while True:  # keep alive

                try:  # receive request from client
                    data, client_address = self.sock.recvfrom(2048)

                    c_thread = threading.Thread(target=self.handle_request,
                                                args=(data, client_address))
                    c_thread.daemon = True
                    c_thread.start()

                except OSError as err:
                    pass
                    # self.printwt(err)

        except KeyboardInterrupt:
            pass
            # self.shutdown_server()


def main():
    udp_server_multi_client = UDPServerMultiClient('127.0.0.1', 12345)
    udp_server_multi_client.configure_server()
    udp_server_multi_client.wait_for_client()


def thread_function():
    listener = keyboard.Listener(on_press=on_press)
    listener.start()  # start to listen on a separate thread
    listener.join()  # remove if main thread is polling self.keys


if __name__ == '__main__':
    x = threading.Thread(target=thread_function, args=())
    x.start()
    main()