
from time import sleep
from threading import Thread
import queue
import time

import pygame
from pygame import locals

class ThreadedPygameJoystick:
    NOMATCH = 'No Match'
    
    def __init__(self, joystick_id):
        self.joystick_id = joystick_id
        # Initialise event dictionary.
        # Add gamepad commands using the append method before executing the start method.
        self.gamepadInputs = {}
        self.lastEventCode = self.NOMATCH
        # Initialise the thread status flag
        self.stopped = False
        self.q = queue.LifoQueue()

    def start(self):
        # Start the thread to poll gamepad event updates
        t = Thread(target=self.gamepad_update, args=())
        t.daemon = True
        t.start()
        
    def gamepad_update(self):
        while True:
            # Should the thread exit?
            if self.stopped:
                return
            # Code execution stops at the following line until a gamepad event occurs.
            events = pygame.event.get()
            for e in events:
                print('Processing event.joy=%s' % e)
                if e.joy != self.joystick_id:
                    return
                command = 'UNKNOWN'
                if e.type == pygame.locals.JOYAXISMOTION: # 7
                    if e.axis == 0:
                        command = 'ABS_Y'
                    elif e.axis == 1:
                        command == 'ABS_Z'
                    elif e.axis == 2:
                        command == 'ABS_RZ'
                    elif e.axis == 3:
                        command == 'ABS_X'
                if command == 'UNKNOWN':
                    return
                self.gamepadInputs[command] = (e.value + 1) * 256
                self.lastEventCode = command
                self.q.put(command)

    def read(self):
        # Return the latest command from gamepad event
        if not self.q.empty():
            newCommand = self.q.get()
            while not self.q.empty():
                trashBin = self.q.get()
    
            return newCommand, self.gamepadInputs[newCommand]
        else:
            return self.NOMATCH, 0

    def stop(self):
        # Stop the game pad thread
        self.stopped = True
        
    def append_command(self, newCommand, newValue):
        # Add new controller command to the list
        if newCommand not in self.gamepadInputs:
            self.gamepadInputs[newCommand] = newValue
        else:
            print('New command already exists')
        
    def delete_command(self, commandKey):
        # Remove controller command from list
        if commandKey in self.gamepadInputs:
            del self.gamepadInputs[commandKey]
        else:
            print('No command to delete')

    def command_value(self, commandKey):
        # Get command value
        if commandKey in self.gamepadInputs:
            return self.gamepadInputs[commandKey]
        else:
            return None