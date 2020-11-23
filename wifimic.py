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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.bind(('', 4000))

volume = 0.1     # range [0.0, 1.0]
fs = 22050       # sampling rate, Hz, must be integer
duration = 1.0   # in seconds, may be float
f = 440.0        # sine frequency, Hz, may be float

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

def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        result = result * 256 + int(b)
    return result

def int_to_bytes(value, length):
    result = []
    for i in range(0, length):
        result.append(value >> (i * 8) & 0xff)
    result.reverse()
    return result

stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=22050,
                output_device_index=0,
                output=True)

previous = 0

while True:
  buf = sock.recv(720*2+16)
  (version, counter, samples, mean, rms) = struct.unpack('IIIHH', buf[0:16])
  data = bytearray(buf[16:])

  print(version, counter, samples, mean, rms, bytes_to_int(data[0:2]))
  if counter != previous+1:
    print('missed packet')
  previous = counter

  stream.write(bytes(data))

stream.stop_stream()
stream.close()

p.terminate()

