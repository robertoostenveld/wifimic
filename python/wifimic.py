#!/usr/bin/env python3

import pyaudio
import audioop
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

fifo = {}
previous = {}
missed = {}
state = {}

running = True
clients = []
threads = []
fifolock = threading.Lock()

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
        self.timestamp = np.zeros(length)
 
        self.pushcount = 0  # the total number of samples that has been pushed
        self.popcount = 0   # the total number of samples that has been popped
        self.pushindex = 0  # the current sample in the buffer to be pushed
        self.popindex = 0   # the current sample in the buffer to be popped
        self.lastseen = 0
        self.lock = threading.Lock()  # prevent concurency problems
        logger.info('initialized buffer of %d samples for %d' % (length, id))

    def push(self, dat, timestamp):
        l = len(dat)
        logger.debug('pushed %d samples from %d' % (l, self.id))
        self.lastseen = time.time()
        # make a millisecond timestamp for every sample
        ts = timestamp + np.arange(0, l)*1000/44100
        with self.lock:
            if self.pushindex + l < self.length:
                self.buffer[self.pushindex:self.pushindex + l] = dat
                self.timestamp[self.pushindex:self.pushindex + l] = ts
            else:
                l1 = self.length - self.pushindex
                l2 = l - l1
                self.buffer[-l1:] = dat[0:l1]  # insert this at the end
                self.buffer[:l2] = dat[l1:]    # insert this at the start
                self.timestamp[-l1:] = ts[0:l1]  # insert this at the end
                self.timestamp[:l2] = ts[l1:]    # insert this at the start
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

    def samples(self):
        # how many samples are available?
        return self.pushcount-self.popcount

    def latest(self):
        # what is the timestamp of the latest sample?
        return self.timestamp[self.pushindex-1]

    def oldest(self):
        # what is the timestamp of the oldest sample?
        return self.timestamp[self.popindex]

    def align(self, timestamp):
        if timestamp<self.oldest() or timestamp>self.latest():
            logger.warning('requested alignment is out of range')
        else:
            index = np.argmin(np.abs(self.timestamp - timestamp))
            if index<self.popindex:
                self.pop(self.popindex-index)
            elif index>self.popindex:
                self.pop(index-self.popindex)
 

def audioHandler(input, frame_count, time_info, status):
    # this is called whenever the audio card needs new data
    global fifo, previous, missed, state

    output = np.zeros((frame_count, channels), dtype=np.float32)

    with fifolock:
        # if a single output channel is configured, the microphones will be mixed up
        # if multiple output channels are configured, each microphone will go to its own channel
        chanindx = 0
        for id, buffer in fifo.items():
            
            if chanindx<channels:
                output[:,chanindx] += amplitude * buffer.pop(frame_count)
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
    global fifo, previous, missed, state

    # the actual packet length will not be 2048 bytes, but depends on the format and number of audio samples
    buf = sock.recv(2048)
    if len(buf)<12:
        return

    # see https://en.wikipedia.org/wiki/RTP_payload_formats
    (version, type, counter, timestamp, id) = struct.unpack('BBHII', buf[0:12])

    if version!=128:
        raise RuntimeError('unsupported packet version')
    
    fragment = bytearray(buf[12:])
    
    with fifolock:
        if not id in fifo:
             fifo[id] = RingBuffer(id, rate) # make a buffer that can hold one second of audio
             previous[id] = None
             state[id] = None
             missed[id] = 0
        
        if type==0:
            # type=0  PCMU  audio 1 8000  any 20  ITU-T G.711 PCM μ-Law audio 64 kbit/s               RFC 3551
            fragment = audioop.ulaw2lin(fragment, 2)
            fragment, state[id] = audioop.ratecv(fragment, 2, 1, 8000, 44100, state[id])
            dat = np.frombuffer(fragment, np.int16)
        elif type==1:
            # type=8  PCMA  audio 1 8000  any 20  ITU-T G.711 PCM A-Law audio 64 kbit/s               RFC 3551
            fragment = audioop.alaw2lin(fragment, 2)
            fragment, state[id] = audioop.ratecv(fragment, 2, 1, 8000, 44100, state[id])
            dat = np.frombuffer(fragment, np.int16)
        elif type==11:
            # type=11 L16   audio 1 44100 any 20  Linear PCM 16-bit audio 705.6 kbit/s, uncompressed  RFC 3551, Page 27
            dat = np.frombuffer(fragment, np.int16)
        else:
            raise RuntimeError('unsupported RTP packet type')
        
        if not previous[id]==None:
            for missing in range(previous[id] + 1 - counter, 0):
                logger.debug('missed packet from %d' % (id))
                # See https://en.wikipedia.org/wiki/Comfort_noise
                missing_dat = np.random.random(len(dat)) # FIXME these are only positive
                missing_dat *= np.linalg.norm(dat)/np.linalg.norm(missing_dat)
                missing_timestamp = timestamp + missing*len(dat)*1000/44100
                missed[id] += 1
                fifo[id].push(missing_dat.astype(np.int16), missing_timestamp)
        
        previous[id] = counter
        fifo[id].push(dat, timestamp)
    

def tcpClientHandler(conn, addr):
    # this is to deal with a connected TCP client 
    while running:
        messageHandler(conn)


def regularMaintenance():
    # this is to detect microphones that have disappeared and to send the synchronization clock
    global fifo, previous, missed, state
    
    with fifolock:
        print('------------------------------------------------------------------')
        for id, buffer in fifo.items():
            print(id, missed[id], buffer.samples())

        # remove microphones that have gone offline
        disappeared = []
        for id, buffer in fifo.items():
            toc = time.time() - buffer.lastseen
            if toc > 5:
                disappeared.append(id)
        for id in disappeared:
            logger.info('removing buffer for %d' % (id))
            del fifo[id]
            del previous[id]
            del missed[id]
            del state[id]

        # align the buffers to each other
        if len(fifo)>1:
            timestamp = []
            for id, buffer in fifo.items():
                timestamp.append(buffer.oldest())
            timestamp = np.max(timestamp)
            for id, buffer in fifo.items():
                buffer.align(timestamp)


    # send a broadcast UDP message with the server timestamp
    timestamp = round(time.monotonic()*1000)
    logger.debug("server timestamp = %d" % timestamp)
    timestamp = struct.pack('!I', timestamp)
    clocksync.sendto(timestamp, ('<broadcast>', syncPort))
    
    # start the maintenance function again after some time
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
