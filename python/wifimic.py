#!/usr/bin/env python3

import pyaudio
import socket
import struct
import numpy as np
import threading
import logging
import sys
import os

logging.basicConfig(level=os.environ.get("LOGLEVEL", "INFO"))
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('wifimic')

total = 0
recvPort = 4000
sendPort = 4001
fifo = {}
previous = {}
running = True
clients = []
threads = []


class RingBuffer():
    """Class for a FIFO buffer
    """

    def __init__(self, id, length):
        self.id = id
        self.length = length
        self.buffer = np.zeros(length, dtype=np.int16)
        self.pushcount = 0  # the total number of samples that has been pushed
        self.popcount = 0  # the total number of samples that has been popped
        self.pushindex = 0  # the current sample in the buffer to be pushed
        self.popindex = 0  # the current sample in the buffer to be popped
        self.lock = threading.Lock()  # prevent concurency problems

    def push(self, dat):
        l = len(dat)
        logger.debug('pushed %d samples from %d' % (l, self.id))
        with self.lock:
            if self.pushindex + l < self.length:
                self.buffer[self.pushindex:self.pushindex + l] = dat
            else:
                l1 = self.length - self.pushindex
                l2 = l - l1
                self.buffer[-l1:] = dat[0:l1]  # insert this at the end
                self.buffer[:l2] = dat[l1:]   # insert this at the start
            self.pushcount += l
            self.pushindex = self.pushcount % self.length
            # check how many samples we have pushed and popped
            l = self.pushcount - self.popcount
            if l > self.length:
                logger.debug('overrun in %d' % (self.id))
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

    output = np.zeros(frame_count).astype(np.int16)

    for id, buffer in fifo.items():
        output += buffer.pop(frame_count)

    return (output.tobytes(), pyaudio.paContinue)


def clientHandler(conn, addr):

    frame_count = 1024

    while running:

        buf = conn.recv(16)
        while len(buf) < 16:
            buf += conn.recv(16 - len(buf))
        (version, id, counter, samples) = struct.unpack('IIII', buf[0:16])

        buf = conn.recv(samples * 2)
        while len(buf) < samples * 2:
            buf += conn.recv(samples * 2 - len(buf))
        dat = np.frombuffer(bytearray(buf), dtype=np.int16)

        if not id in fifo:
            fifo[id] = RingBuffer(id, 22050)

        fifo[id].push(dat)

        if not id in previous:
            previous[id] = 0

        if counter != previous[id] + 1:
            logger.debug('missed packet from %d' % (id))
        previous[id] = counter


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
                logger.info('Got connection from %s on %d' % addr)
                thread = threading.Thread(target=clientHandler, args=(conn, addr))
                clients.append(addr)
                threads.append(thread)
                thread.start()

    except (SystemExit, KeyboardInterrupt, RuntimeError):
        logger.info('Stopping')

        for thread in threads:
            running = False
            thread.join()

        stream.stop_stream()
        stream.close()
        p.terminate()

        sock.close()
