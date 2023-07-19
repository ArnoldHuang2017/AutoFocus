import time
import traceback
import os
import selectors
import threading
from serial import Serial, SerialException
from serial.tools import list_ports
from Common import singleton, Timeout, DecodeBytes
from socketserver import ThreadingUnixStreamServer, BaseRequestHandler


class SerialCommLogHandler(BaseRequestHandler):
    def setup(self) -> None:
        SerialComm().AddClient(self.request)

    def handle(self) -> None:
        selector = selectors.PollSelector()
        selector.register(self.request, selectors.EVENT_READ)
        selector.select()

    def finish(self) -> None:
        SerialComm().RemoveClient()


@singleton
class SerialCommServer(ThreadingUnixStreamServer):
    SocketAddress = "/run/console.socket"

    def __init__(self):
        ThreadingUnixStreamServer.__init__(self, self.SocketAddress, SerialCommLogHandler)

    def server_bind(self):
        if os.path.exists(self.SocketAddress):
            os.unlink(self.SocketAddress)

        super().server_bind()


@singleton
class SerialComm(Serial):
    def __init__(self):
        Serial.__init__(self, baudrate=115200)
        self.__stop = False
        self.online = False
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.client = None
        self.lock = threading.Lock()

    def __get_nucleus_serial_port(self):
        comports = list_ports.comports()
        return [info for info in comports if info.device.startswith(self.port)]

    def SendToClient(self, data):
        if self.client is None:
            return
        try:
            self.client.send(data.encode())
        except:
            pass

    def AddClient(self, client):
        self.client = client

    def RemoveClient(self):
        self.client = None

    def Write(self, data):
        if self.is_open:
            self.flushOutput()
            return self.write(data.encode())

    def Read(self, time_out=5):
        selector = selectors.PollSelector()
        selector.register(self.fd, selectors.EVENT_READ)

        while not self.__stop:
            try:
                ready = selector.select(1)
                if ready:
                    try:
                        data = os.read(self.fd, self.in_waiting)
                        if data:
                            data = DecodeBytes(data)
                            self.SendToClient(data)
                    except:
                        print("Something wrong during read Serial Comm, please retry")
            except SerialException:
                print(traceback.format_exc())
                self.close()
                break
            except Exception:
                print(traceback.format_exc())
                self.close()
                break

        selector.unregister(self.fd)
        self.close()

    def MonitorPortOnline(self):
        while not self.__stop:
            try:
                port_list = self.__get_nucleus_serial_port()
                if not port_list:
                    print("Error: The USB Serial Device may be removed. Communication to Nucleus is missing!")
                    time.sleep(1)
                    if self.online:
                        self.close()
                        self.online = False
                    continue
                else:
                    if not self.online:
                        print("Reconnect the USB Serial.")
                        self.open()
                        if self.is_open:
                            self.online = True
                            self.Read()

            except:
                print(traceback.format_exc())
                pass

            time.sleep(1)
