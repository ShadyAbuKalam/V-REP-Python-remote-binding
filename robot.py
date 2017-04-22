# todo : Inheirt the thread object and initialize left and right motors
# handle values
import vrep
import threading
import random
import time
from queue import Queue


class Robot(threading.Thread):
    # The analogus of setup() inside microcontoller
    def __init__(self, name, sim):
        threading.Thread.__init__(self)
        self.message_queue = Queue()
        self.sim = sim
        self.run_event = self.sim.run_event
        self.clientID = self.sim.clientID
        self.name = name

        # Setup the postfix of the robot name
        self.postfix = ""
        try:

            i = self.name.index("DragonOne")+len("DragonOne")
            self.postfix = self.name[i:]
        except:
            pass

        # Setup robot handle
        returnCode, self.handle = vrep.simxGetObjectHandle(
            self.clientID, name, vrep.simx_opmode_blocking)

        # Setup left motor and right motor handles respectively
        returnCode, self.left_motor = vrep.simxGetObjectHandle(
            self.clientID, "LeftWheelRevolute_joint{0}".format(self.postfix),
            vrep.simx_opmode_blocking)

        returnCode, self.right_motor = vrep.simxGetObjectHandle(
            self.clientID, "RightWheelRevolute_joint{0}".format(self.postfix),
            vrep.simx_opmode_blocking)

    def run(self):
        # The analogus of loop() inside microcontoller,write your code inside
        # the while loop
        while self.run_event.is_set():

            if(self.message_queue.qsize() > 0):
                msg = self.message_queue.get()
                status = ("Robot {0} Got message : {1}".format(self.name, msg))
                vrep.simxAddStatusbarMessage(
                    self.clientID, status, vrep.simx_opmode_blocking)
            if (random.random() > 0.8):
                self.broadcast("Robot {0} Says hello".format(self.name))
            vrep.simxSetJointTargetVelocity(
                self.clientID, self.left_motor, random.random() * 10,
                vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(
                self.clientID, self.right_motor, random.random() * 10,
                vrep.simx_opmode_blocking)
            time.sleep(0.2)

    # This will broadcast a message to all other robots
    def broadcast(self, message):
        self.sim.broadcast(self, message)

    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)
