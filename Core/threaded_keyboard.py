

from time import sleep
from threading import Thread
import queue
import time

import sys
import termios
import tty

class ThreadedKeyboard:
    NOMATCH = ''

    def __init__(self):
        self.stopped = False
        self.q = queue.LifoQueue()
        self.__original_settings = termios.tcgetattr(sys.stdin)

    def start(self):
        t = Thread(target=self.get_key_continuous, args=())
        t.daemon = True
        t.start()

    def get_key_continuous(self):
        print('get_key_continuous starting')
        while not self.stopped:
            self.__get_key()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__original_settings)

    def end_get_key_continuous(self):
        self.stopped = True

    def __get_key(self):
        tty.setcbreak(sys.stdin)
        key_code = sys.stdin.read(1) # ord(sys.stdin.read(1))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__original_settings)
        self.q.put(key_code)
        #print(f'helper put key={key_code}.')

    def read(self):
        if not self.q.empty():
            newCommand = self.q.get()
            while not self.q.empty():
                trashBin = self.q.get()
            return newCommand
        else:
            return self.NOMATCH

    def stop(self):
        self.stopped = True
