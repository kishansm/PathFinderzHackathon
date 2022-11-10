import socket
from time import sleep, time

class Server():
    def __init__(self, UDP_IP, UDP_PORT):
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        print("Socket initialized with IP {}, PORT {} ".format(self.UDP_IP,self.UDP_PORT))

    def send_command(self, MESSAGE):
        #writing
        bit_string = MESSAGE.encode('utf-8')
        self.sock.sendto(bit_string, (self.UDP_IP, self.UDP_PORT))
        # sleep(0.5)

    def recv_message(self, buffer_size):
        #reading
        data, addr = self.sock.recvfrom(buffer_size) # buffer size is 1024 bytes
        # print("received message: %s" % data)
        return data.decode('UTF-8')