#!/usr/bin/env python

import pyaudio
import wave
import time
import sys
import socket
import struct
import math
import numpy as np
import threading

# this is to prevent concurrency problems
lock = threading.Lock()

total = 0
recvPort = 4000
sendPort = 4001
fifo = {}
previous = {}
running = True
clients = []
threads = []

def audioHandler(input, frame_count, time_info, status):

  output = np.zeros(frame_count).astype(np.int16)
  
  with lock:
    l = []
    for key, value in fifo.items():
      l.append(len(value)/2)
    print(l)

    for key, value in fifo.items():
      if len(value)>=(frame_count*2):
        output += np.frombuffer(bytes(value[0:(frame_count*2)]), dtype=np.int16)
        fifo[key] = value[(frame_count*2):]
      else:
        print('underrun', key)
 
  return (output.tobytes(), pyaudio.paContinue)


def clientHandler(conn, addr):

  frame_count = 1024

  while running:

    buf = conn.recv(16)
    while len(buf)<16:
      buf += conn.recv(16-len(buf))
    (version, id, counter, samples) = struct.unpack('IIII', buf[0:16])

    buf = conn.recv(samples*2)
    while len(buf)<samples*2:
      buf += conn.recv(samples*2-len(buf))
    data = bytearray(buf)

    with lock:
      # print(version, id, counter, samples, len(buf))
   
      if not id in fifo:
        fifo[id] = bytearray(0)
   
      if len(fifo[id])==0:
        fifo[id] = data
      elif len(fifo[id])>2*(frame_count*2):
        print('overrun', id)
        fifo[id] = fifo[id][(frame_count*2):]
      else:
        fifo[id] = fifo[id] + data
   
      if not id in previous:
        previous[id] = 0
   
      if counter != previous[id]+1:
        print('missed packet', id)
      previous[id] = counter


if __name__ == '__main__':

  p = pyaudio.PyAudio()

  print('------------------------------------------------------------------')
  info = p.get_host_api_info_by_index(0)
  print(info)
  print('------------------------------------------------------------------')
  for i in range(info.get('deviceCount')):
      if p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels') > 0:
          print("Input  Device id " + str(i) + " - " + p.get_device_info_by_host_api_device_index(0, i).get('name'))
      if p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:
          print("Output Device id " + str(i) + " - " + p.get_device_info_by_host_api_device_index(0, i).get('name'))
  print('------------------------------------------------------------------')

  stream = p.open(format=pyaudio.paInt16,
    channels=1,
    rate=22050,
    output=True,
    output_device_index=0,
    stream_callback=audioHandler)

  stream.start_stream()

  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind(('', recvPort))
  sock.listen(1)

  try:
    while True:
      conn, addr = sock.accept()
      if not addr in clients:
         print('Got connection from', addr)
         thread = threading.Thread(target=clientHandler,args=(conn,addr))
         clients.append(addr)
         threads.append(thread)
         thread.start()

  except (SystemExit, KeyboardInterrupt, RuntimeError):
    print('Stopping')

    for thread in threads:
      running = False
      thread.join()

    stream.stop_stream()
    stream.close()
    p.terminate()

    sock.close()
