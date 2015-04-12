import socket
import struct

UDP_IP = "192.168.0.9"
# UDP_IP = "10.1.1.1"
UDP_PORT = 21050

while True:
    msg = raw_input("Enter [CMD] [INDEX] [VALUE] : ").split()
    # msg = struct.pack('c', msg[0]), struct.pack('c', int(msg[1])), struct.pack('h', msg[2])
    msg = chr(int(msg[0])), chr(int(msg[1])), str(chr(int(msg[2])/256)+chr(int(msg[2])%256))
    fmsg = ''.join(msg)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.sendto(fmsg, (UDP_IP, UDP_PORT))

    print fmsg
    print "Msg sent.  Waiting on reply..."

    reply, reply_addr = sock.recvfrom(10240)
    if(reply):
        print "Reply received:", reply_addr, reply