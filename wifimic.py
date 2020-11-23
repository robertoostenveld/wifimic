#!/usr/bin/env python

"""PyAudio Example: Play a wave file (callback version)"""

import pyaudio
import wave
import time
import sys
import socket
import struct
import math
import numpy as np

recvPort = 4000
sendPort = 4001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.bind(('', recvPort))

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

fifo = {}
previous = {}

def callback(in_data, frame_count, time_info, status):
  global fifo

  out_data = np.zeros(frame_count).astype(np.int16)
  for key, value in fifo.items():
    if len(value)>=(frame_count*2):
      out_data += np.frombuffer(bytes(value[0:(frame_count*2)]), dtype=np.int16)
      fifo[key] = value[(frame_count*2):]
    else:
      print('underrun', key)

  return (out_data.tobytes(), pyaudio.paContinue)

stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=22050,
                output=True,
                output_device_index=0,
                stream_callback=callback)

stream.start_stream()

while True:
  buf, addr = sock.recvfrom(16)
  (version, id, counter, samples) = struct.unpack('IIII', buf[0:16])
  data = bytearray(sock.recv(samples*2))

  print(version, id, counter, samples, int(np.frombuffer(data[0:2], dtype=np.int16)))

  response = struct.pack('III', 1, 21, counter)
  sock.sendto(response, (addr[0], sendPort))

  if not id in fifo:
    fifo[id] = bytearray(0)

  if len(fifo[id])==0:
    fifo[id] = data
  elif len(fifo[id])>(5000*2):
    print('overrun')
    fifo[id] = fifo[id][-(2500*2):] + data
  else:
    fifo[id] = fifo[id] + data

  if not id in previous:
    previous[id] = 0

  if counter != previous[id]+1:
    print('missed packet', id)
  previous[id] = counter

stream.stop_stream()
stream.close()

p.terminate()

