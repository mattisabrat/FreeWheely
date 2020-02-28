#!/usr/bin/env python3

from transitions.extensions import HierarchicalMachine as Machine
import time
import logging
import gpiozero as gpio
import numpy as np
from Stepper import Processed_PID_BigEasyDriver as BED
 

class FreeWheely(Machine):
    def __init__(self, poisson=True, microstepping = 2, log = None, queued = True):

        #init the logging
        self.logger = logging.getLogger('transitions').setLevel(logging.INFO)
        if log is not None: self.logger.addHandler(logging.FileHandler(log))

        #init the display
        #   self.OLED=Threaded_FW_OLED()

        #Poisson?
        self.poisson = poisson

        #Assign GPIO pins for triggers
        self.flir    = gpio.LED(21) 
        self.mscope  = gpio.LED(20) 
        self.avisoft = gpio.LED(19)
        self.flir.off()
        self.mscope.off()
        self.avisoft.off()
        self.rec = False

        #Assign GPIO for the easy driver and prep the microstepping pins
        self.ms1    = gpio.LED(22) 
        self.ms2    = gpio.LED(24)
        self.enable = gpio.LED(23)
        self.dir    = gpio.LED(25)
        self.step   = gpio.LED(18)
        self.slp    = gpio.LED(27)
        self.pfd    = gpio.LED(4)
        self.rst    = gpio.LED(17)
        self.set_microstep(microstepping)
        self.enable.off()
        self.rst.on()
        self.slp.on()

        #Assign the rest of the GPIO for the wheel rig
        self.LEDs = gpio.PWMLED(9)
        self.tint = gpio.LED(10) 


        ##State machine stuff for the wheel rig
        states = [
                {'name': 'Hopping', 'children': ['Rec_on', 'Rec_off']},
                {'name': 'Singing', 'children': ['Rec_on', 'Rec_off']},
                {'name': 'Waiting', 'children': ['Rec_on', 'Rec_off']},
                {'name': 'Finished', 'on_enter': 'finish'}
            ]

        transitions = [
            #Move between behavioral states
                {'trigger': 'Hop',    'source': ['Singing_Rec_on',  'Waiting_Rec_on'],   'dest': 'Hopping_Rec_on',  'after' : 'hop'},
                {'trigger': 'Hop',    'source': ['Singing_Rec_off', 'Waiting_Rec_off'],  'dest': 'Hopping_Rec_off', 'after' : 'hop'},
                {'trigger': 'Sing',   'source': ['Hopping_Rec_on',  'Waiting_Rec_on'],   'dest': 'Singing_Rec_on',  'after' : 'sing'},
                {'trigger': 'Sing',   'source': ['Hopping_Rec_off', 'Waiting_Rec_off'],  'dest': 'Singing_Rec_off', 'after' : 'sing'},
                {'trigger': 'Wait',   'source': ['Hopping_Rec_on',  'Singing_Rec_on'],   'dest': 'Waiting_Rec_on',  'after' : 'wait'},
                {'trigger': 'Wait',   'source': ['Hopping_Rec_off', 'Singing_Rec_off'],  'dest': 'Waiting_Rec_off', 'after' : 'wait'},
                {'trigger': 'Finish', 'source': ['Hopping_Rec_off', 'Singing_Rec_off', 'Waiting_Rec_off'], 'dest': 'Finished'},
            #Move between recording states
                {'trigger': 'Rec_Start', 'source': 'Hopping_Rec_off', 'dest': 'Hopping_Rec_on', 'after': 'record'},
                {'trigger': 'Rec_Start', 'source': 'Singing_Rec_off', 'dest': 'Singing_Rec_on', 'after': 'record'},
                {'trigger': 'Rec_Start', 'source': 'Waiting_Rec_off', 'dest': 'Waiting_Rec_on', 'after': 'record'},
                {'trigger': 'Rec_Stop',  'source': 'Hopping_Rec_on',  'dest': 'Hopping_Rec_off', 'after': 'stop'},
                {'trigger': 'Rec_Stop',  'source': 'Singing_Rec_on',  'dest': 'Singing_Rec_off', 'after': 'stop'},
                {'trigger': 'Rec_Stop',  'source': 'Waiting_Rec_on',  'dest': 'Waiting_Rec_off', 'after': 'stop'},
            #Control the lights
                {'trigger': 'Set_Lights', 'source': '*', 'dest': None, 'before': 'set_lights'}
            ]

        #Init the machine
        Machine.__init__(self, states = states,
                transitions = transitions,
                initial = 'Waiting_Rec_off',
                ignore_invalid_triggers=False,
                queued=queued)

      
    ##Define the state functions for the wheel
    def hop(self, duration, rpm, poisson=None, forwards=True):
        #Update display
        #self.display('Hopping', rpm, self.LEDs.value, forwards, self.rec)  

        #Handle the poisson
        if poisson is None: poisson = self.poisson
        if poisson is True: duration = float(np.random.poisson(duration,1))

        print('%s__%s__%s' % (duration,rpm,forwards,))
        
        #Turn the wheel
        self.turn(duration, rpm, forwards)
            
    def sing(self, duration, poisson=None):
        #Update display
        #self.display('Singing', 0, self.LEDs.value, ' ', self.rec)  

        #Handle the poisson
        if poisson is None: poisson = self.poisson
        if poisson is True: duration = float(np.random.poisson(duration,1))

        #Provide access to females
        self.tint.on()
        time.sleep(duration)
        self.tint.off()

    def wait(self, duration, poisson=None):
        #Update display
        #self.display('Waiting', 0, self.LEDs.value, ' ', self.rec)  

        #Handle the poisson
        if poisson is None: poisson = self.poisson
        if poisson is True: float(np.random.poisson(duration,1))

        #Wait
        time.sleep(duration)

    def finish(self):
        print('finished')#self.display('Finished', 0, self.LEDs.value, ' ', self.rec)

    ##Define the transition functions
    def set_lights(self, duty):
        self.LEDs.value=duty

    def record(self):
        self.flir.on()
        self.mscope.on()
        self.avisoft.on()

        self.rec=True

    def stop(self):
        self.flir.off()
        self.mscope.off()
        self.avisoft.off()

        self.rec=False


    ##function to turn the motor    
    def turn(self, duration, rpm, forwards):
        per = 1/(rpm/60*200*self.microstepping) #interstep period
        n = round(duration/per) #number of steps
        print(per)
        #set direction
        if forwards: self.dir.on()
        elif not forwards: self.dir.off()

        #turn motor
        for r in range(n):
            #self.step.toggle()
            self.step.off()
            time.sleep(per/2)
            self.step.on()
            time.sleep(per/2)
        
    #tell the machine how much to microstep th motor
    def set_microstep(self, step):
        if step == 8:
            self.microstepping = 8
            self.ms1.on()
            self.ms2.on()
            
        elif step == 4:
            self.microstepping = 4
            self.ms1.off()
            self.ms2.on()
            
        elif step == 2:
            self.microstepping = 2
            self.ms1.on()
            self.ms2.off()
            
        elif step == 1:
            self.microstepping = 1
            self.ms1.off()
            self.ms2.off()
            
        else: self.logger.error('Invalid microstep: %s' % step)    
        
    def display(self, state, rpm, duty, rec, direction):
        #Pass the updated to the other thread
        self.OLED.queue.put([state, rpm, duty, rec, direction])

                
        

if __name__ == '__main__':
    bird = FreeWheely()
    time.sleep(1)
    bird.Set_Lights(1)
    bird.Hop(60,200)
    bird.Finish()

        
        
            
