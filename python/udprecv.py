import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.bind(('', 4000))

count = 0

while True:
  buf = sock.recv(1500)
  print(count)
  count += 1
  (version, samples, bytes) = struct.unpack('iii', buf[0:12])
