#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time
import signal
import time

from multiprocessing import Queue, freeze_support
try:
    from queue import Empty, Full # for exception catching
except ImportError:
    from Queue import Empty, Full # for exception catching
import parserthread # our module realizing mavlink datastream parsing
import csvpool

# command line stuff
parser = argparse.ArgumentParser(description='Description will be here.')
parser.add_argument('-d', '--device', type=str, required=True,
        help='Connection device. For example `/dev/ttyUSB0` or `udp::14551`')
parser.add_argument('-b', '--baudrate', type=int, required=False,
        help='Com port baudrate. Not used with UDP connection')
args = parser.parse_args()


## Start main loop.
if __name__ == '__main__':
    freeze_support()

    csvpool = csvpool.CsvPool()
    queue = Queue(128)

    parse_th = parserthread.ParserThread(args.device, args.baudrate, queue)
    parse_th.start()
    time.sleep(0.3)

    print("*** Main process started. Pid:", os.getpid())
    try:
        while True:
            while not queue.empty():
                mavmsg = queue.get_nowait()
                csvpool.post(mavmsg)
            time.sleep(0.10)
    except KeyboardInterrupt:
        pass

    parse_th.stop()
    parse_th.join()


