#!/usr/bin/env python

import pyaudio
import numpy as np

p = pyaudio.PyAudio()

# for paFloat32 sample values must be in range [-1.0, 1.0]
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=fs,
                output=True)

volume = 1.0     # range [0.0, 1.0]
fs = 22050       # sampling rate, Hz, must be integer
duration = 1.0   # in seconds, may be float
f = 440.0        # sine frequency, Hz, may be float

samples = volume * 32767. * (np.sin(2*np.pi*np.arange(fs*duration)*f/fs)+1)/2
samples = samples.astype(np.uint16)
samples = samples.tobytes()

# play. May repeat with different volume values (if done interactively) 
stream.write(samples)

stream.stop_stream()
stream.close()

p.terminate()
