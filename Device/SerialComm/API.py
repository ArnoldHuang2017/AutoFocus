import socket
import threading
from .SerialComm import SerialComm, SerialCommServer
from Common.Utils import DecodeBytes
import time
import selectors


def Start():
    """
    Start Nucleus N Serial Communication
    :return:
    """
    t = threading.Thread(target=SerialComm().MonitorPortOnline)
    t.daemon = True
    t.start()

    server_t = threading.Thread(target=SerialCommServer().serve_forever)
    server_t.daemon = True
    server_t.start()
    return True


def Stop():
    """
    Stop Nucleus N Serial Communication
    :return:
    """
    SerialComm().exit = True
    time.sleep(2)
    SerialComm().close()


def Read():
    return SerialComm().Read()


def Write(data):
    return SerialComm().Write(data)


class ThreadingRead(threading.Thread):
    def __init__(self, connect_callback=None, read_callback=None):
        '''
        Read console output in threading
        :param connect_callback: when socket connected, call the function
        :param read_callback: when socket read data from socket, call the function
        :param is_print: if True print the output
        '''
        threading.Thread.__init__(self)
        self.connect_callback = connect_callback
        self.read_callback = read_callback
        self.daemon = True
        self.event = threading.Event()
        self.port = SerialComm().port
        self.__content = ""
        self.start()

    @property
    def content(self):
        return self.__content

    @property
    def lines(self):
        return self.__content.splitlines()

    def stop(self):
        self.event.set()

    def run(self) -> None:
        sock = socket.socket(socket.AF_UNIX)
        sock.connect(SerialCommServer.SocketAddress)

        selector = selectors.PollSelector()
        selector.register(sock, selectors.EVENT_READ)

        if self.connect_callback is not None:
            self.connect_callback()

        while not self.event.is_set():
            ready = selector.select(0.5)
            if ready:
                data = sock.recv(2048)
                if not data:
                    break

                data = DecodeBytes(data)
                if self.read_callback is not None and self.read_callback(data):
                    break

        selector.unregister(sock)
        sock.close()


__all__ = ["Start", "Stop", "Write", "Read", "ThreadingRead"]
