import socket
import sys
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('10.211.55.2', 10000)
message = b'capture'

def capture():
        # Send data
        print('sending "%s"' % message)
        sent = sock.sendto(message, server_address)

        # Receive response
        print('waiting to receive')
        data, server = sock.recvfrom(57621)
        print('received "%s"' % data)

