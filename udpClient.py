import socket

UDP_IP = "10.1.1.1"
UDP_PORT = 21050

while True:
    msg = raw_input("Enter [CMD][INDEX][VALUE] : ")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.sendto(msg, (UDP_IP, UDP_PORT))
    print "Msg sent.  Waiting on reply..."

    reply, reply_adr = sock.recvfrom(10240)
    if(reply):
        print "Reply received:", reply_addr, reply