#!/usr/bin/env python3

import pyaudio
import socket
import struct
import numpy as np
import threading
import logging
import os
import time

logging.basicConfig(level=os.environ.get("LOGLEVEL", "INFO"))
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('wifimic')

# an amplitude of 1 passes the 16-bit signal from the wifi micriophones as-is
# which can be too "hot" for the microphone input
amplitude  = 1.0

device = 9
rate = 44100
format = pyaudio.paFloat32
channels = 8

USE_TCP = False
tcpPort = 4000
udpPort = 4001
syncPort = 4002

samples = 0

fifo = {}
fifolock = threading.Lock()
previous = {}
missed = {}
running = True
clients = []
threads = []

# this is used to send UDP packets with the server timestamp
clocksync = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
clocksync.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
clocksync.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
clocksync.settimeout(0.2)

class RingBuffer():
    """Class for a FIFO buffer
    """

    def __init__(self, id, length):
        self.id = id
        self.length = length
        self.buffer = np.zeros(length, dtype=np.int16)
        self.pushcount = 0  # the total number of samples that has been pushed
        self.popcount = 0   # the total number of samples that has been popped
        self.pushindex = 0  # the current sample in the buffer to be pushed
        self.popindex = 0   # the current sample in the buffer to be popped
        self.lastseen = 0
        self.lock = threading.Lock()  # prevent concurency problems
        logger.info('initialized buffer of %d samples for %d' % (length, id))

    def push(self, dat):
        l = len(dat)
        logger.debug('pushed %d samples from %d' % (l, self.id))
        self.lastseen = time.time()
        with self.lock:
            if self.pushindex + l < self.length:
                self.buffer[self.pushindex:self.pushindex + l] = dat
            else:
                l1 = self.length - self.pushindex
                l2 = l - l1
                self.buffer[-l1:] = dat[0:l1]  # insert this at the end
                self.buffer[:l2] = dat[l1:]    # insert this at the start
            self.pushcount += l
            self.pushindex = self.pushcount % self.length
            # check how many samples we have pushed and popped
            l = self.pushcount - self.popcount
            if l > self.length:
                logger.debug('overrun in %d' % (self.id))
                self.popcount = self.pushcount - self.length
                self.popindex = self.popcount % self.length
        return

    def pop(self, l):
        logger.debug('popped %d samples from %d' % (l, self.id))
        with self.lock:
            if self.popindex + l < self.length:
                dat = self.buffer[self.popindex:self.popindex + l]
            else:
                l1 = self.length - self.popindex
                l2 = l - l1
                dat = np.concatenate((self.buffer[-l1:], self.buffer[:l2]))
            self.popcount += l
            self.popindex = self.popcount % self.length
            # ensure that we do not pop more samples than we have pushed so far
            l = self.popcount - self.pushcount
            if l > 0:
                logger.debug('underrun in %d' % (self.id))
                dat[-l:] = 0
                self.popcount -= l
                self.popindex = self.popcount % self.length
        return dat


def audioHandler(input, frame_count, time_info, status):
    # this is called whenever the audio card needs new data
    output = np.zeros((frame_count, channels), dtype=np.float32)

    with fifolock:
        # if a single output channel is configured, the microphones will be mixed up
        # if multiple output channels are configured, each microphone will go to its own channel
        chanindx = 0
        for id, buffer in fifo.items():
            if chanindx<channels:
                output[:,chanindx] += buffer.pop(frame_count)*amplitude
            else:
                raise RuntimeError('there are more microphones than output channels')
            if channels>1:
                # only increment if there are multiple output channels, otherwise stick to the first channel
                chanindx += 1
        if channels==1 and len(fifo)>0:
            # automatically scale the volume so that it does not clip
            # the sum of int16 values should not exeed the maximum it can represent
            output = output / len(fifo)
        if format==pyaudio.paFloat32:
            # scale between -1.0 and 1.0
            output = output/32768.
        elif format==pyaudio.paInt16:
            # keep as it is
            output = output
        elif format==pyaudio.paInt8:
            # scale between -128 and 127
            output = output/256.

    if format==pyaudio.paFloat32:
        output = output.astype(np.float32)
    elif format==pyaudio.paInt16:
        output = output.astype(np.int16)
    elif format==pyaudio.paInt8:
        output = output.astype(np.int8)
 
    return (output.tobytes(), pyaudio.paContinue)


def messageHandler(conn):
    # this is to deal with a single incoming TCP or UDP message
    global samples

    # the packet will not be 2048 bytes long, but depends on the number of audio samples
    buf = sock.recv(2048)
    if len(buf)<12:
        return

    (byte0, byte1, counter, timestamp, id) = struct.unpack('BBHII', buf[0:12])
    if byte0!=128 or byte1!=11:
        return
    
    dat = np.frombuffer(bytearray(buf[12:]), dtype=np.int16)
    samples = len(dat)

    with fifolock:
         if not id in fifo:
             fifo[id] = RingBuffer(id, rate) # make a buffer that can hold one second
         if not id in previous:
             previous[id] = 0
             missed[id] = 0
         else:
             for missing in range(counter - previous[id] -1):
                 logger.debug('missed packet from %d' % (id))
                 missed[id] += 1
                 noise = np.random.random(samples) # FIXME
                 noise *= np.linalg.norm(dat)/np.linalg.norm(noise)
                 fifo[id].push(noise.astype(np.int16))
         fifo[id].push(dat)

    previous[id] = counter
    

def tcpClientHandler(conn, addr):
    # this is to deal with a connected TCP client 
    while running:
        messageHandler(conn)


def regularMaintenance():
    print(missed)
    
    # since the volume scales automatically with the number of microphones to prevent clipping
    # we should remove old microphones that have gone offline
    remove = []
    with fifolock:
        for id, buffer in fifo.items():
            toc = time.time() - buffer.lastseen
            if toc > 15:
                logger.info('removing buffer for %d' % (id))
                remove.append(id)
        for id in remove:
            del fifo[id]
            del previous[id]
            del missed[id]

    # send a broadcast UDP message with the server timestamp
    timestamp = round(time.monotonic()*1000)
    logger.info("server timestamp = %d" % timestamp)
    timestamp = struct.pack('!I', timestamp)
    clocksync.sendto(timestamp, ('<broadcast>', syncPort))
    
    # start this function again after some time
    threading.Timer(2.0, regularMaintenance).start()


if __name__ == '__main__':

    p = pyaudio.PyAudio()

    logger.info('------------------------------------------------------------------')
    info = p.get_host_api_info_by_index(0)
    logger.info(info)
    logger.info('------------------------------------------------------------------')
    for i in range(info.get('deviceCount')):
        if p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels') > 0:
            logger.info("Input  Device id " + str(i) + " - " +
                        p.get_device_info_by_host_api_device_index(0, i).get('name'))
        if p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:
            logger.info("Output Device id " + str(i) + " - " +
                        p.get_device_info_by_host_api_device_index(0, i).get('name'))
    logger.info('------------------------------------------------------------------')

    stream = p.open(format=format,
                    channels=channels,
                    rate=rate,
                    output=True,
                    output_device_index=device,
                    stream_callback=audioHandler)

    # stream.start_stream()

    if USE_TCP:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', tcpPort))
        sock.listen(1)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(('', udpPort))


    # this will run every N seconds
    regularMaintenance()

    try:
        while True:
            if USE_TCP:
                conn, addr = sock.accept()
                if not addr in clients:
                    logger.info('Got connection from %s on %d' % addr)
                    thread = threading.Thread(target=tcpClientHandler, args=(conn, addr))
                    clients.append(addr)
                    threads.append(thread)
                thread.start()
            else:
                messageHandler(sock)
                

    except (SystemExit, KeyboardInterrupt, RuntimeError):
        logger.info('Stopping')

        for thread in threads:
            running = False
            thread.join()

        stream.stop_stream()
        stream.close()
        p.terminate()

        sock.close()
