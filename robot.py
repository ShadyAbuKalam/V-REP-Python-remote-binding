import vrep
import threading
import random
import time
import math
from queue import Queue


class Robot(threading.Thread):
    RUNNING = 0
    STOPPED = 1
    def setup(self):
        """
        The analogus of setup() inside microcontoller.
        """
        self.state = Robot.RUNNING
        

    def loop(self):
        """
        The analogus of loop() inside microcontoller,write your code inside
        """
        if self.state == Robot.STOPPED:
            self.motor1(0)
            self.motor2(0)
            return
        
        if self.message_queue.qsize() > 0:
            msg = self.message_queue.get()
        self.motor1(10)
        self.motor2(-10)

        detected,distance = self.read_mid_ultra_sonic()
        if detected:
            self.state = Robot.STOPPED
            status = ("Robot {0} detected element on distance  : {1}".format(self.name, distance))
            vrep.simxAddStatusbarMessage(
            self.clientID, status, vrep.simx_opmode_blocking)
            
            
    def __init__(self, name, sim):
        threading.Thread.__init__(self)
        self.message_queue = Queue()  # Queue for received messages
        self.sim = sim
        self.run_event = self.sim.run_event
        self.clientID = self.sim.clientID
        self.name = name

        # Setup the postfix of the robot name
        self.postfix = ""
        try:

            i = self.name.index("DragonOne") + len("DragonOne")
            self.postfix = self.name[i:]
        except:
            pass

        # Setup robot handle
        self.handle = vrep.simxGetObjectHandle(
            self.clientID, name, vrep.simx_opmode_blocking)[1]

        # Setup left motor, right motor and sensors handles respectively
        self.left_motor = vrep.simxGetObjectHandle(
            self.clientID, "LeftWheelRevolute_joint{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]

        self.right_motor = vrep.simxGetObjectHandle(
            self.clientID, "RightWheelRevolute_joint{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]

        self.left_ultrasonic = vrep.simxGetObjectHandle(
            self.clientID, "LeftProximity_sensor{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]

        self.mid_ultrasonic = vrep.simxGetObjectHandle(
            self.clientID, "MidProximity_sensor{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]

        self.right_ultrasonic = vrep.simxGetObjectHandle(
            self.clientID, "RightProximity_sensor{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]

        self.setup()

    def run(self):
        while self.run_event.is_set():
            self.loop()
            # Sleep so other threads can be scheduled
            time.sleep(0.1)

    def broadcast(self, message):
        """
        This will broadcast a message to all other robots
        """
        self.sim.broadcast(self, message)

    def motor1(self, value, direction=True):
        """
        Sets the motor1 speed proportional to value and the forward/backward direction
        """
        self.__set_motor(self.left_motor, value, direction)

    def motor2(self, value, direction=True):
        """
        Sets the motor1 speed proportional to value and the forward/backward direction
        """
        self.__set_motor(self.right_motor, value, direction)

    def __set_motor(self, motor_handle, value, direction):
        """
        Sets the motor speed proportional to value and the forward/backward direction
        """
        if not isinstance(direction, bool):
            raise ValueError("Parameter direction must be boolean")
        if not isinstance(value, int) and value > 255:
            raise ValueError(
                "Parameter value must be integer between 0 and 255")
        if not direction:
            value = -value
        vrep.simxSetJointTargetVelocity(
            self.clientID, motor_handle, value * .1,
            vrep.simx_opmode_blocking)

    
    def read_left_ultra_sonic(self):
        return self.__read_ultrasonic(self.left_ultrasonic)

    def read_mid_ultra_sonic(self):
        return self.__read_ultrasonic(self.mid_ultrasonic)


    def read_right_ultra_sonic(self):
        return self.__read_ultrasonic(self.right_ultrasonic)

    def __read_ultrasonic(self,sensor_handle):
        state , detected_point =vrep.simxReadProximitySensor(self.clientID,sensor_handle,vrep.simx_opmode_blocking)[1:3]
        if state == 1:
            distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
            return (True,distance)
        return (False,None)

    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)
