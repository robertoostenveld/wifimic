#!/usr/bin/env python

"""PyAudio Example: Play a wave file (callback version)"""

import pyaudio
import wave
import time
import sys

if len(sys.argv) < 2:
    print("Plays a wave file.\n\nUsage: %s filename.wav" % sys.argv[0])
    sys.exit(-1)

wf = wave.open(sys.argv[1], 'rb')

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

def callback(in_data, frame_count, time_info, status):
    data = wf.readframes(frame_count)
    return (data, pyaudio.paContinue)

stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True,
                output_device_index=0,
                stream_callback=callback)

stream.start_stream()

while stream.is_active():
    time.sleep(0.1)

stream.stop_stream()
stream.close()
wf.close()

p.terminate()
