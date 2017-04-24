import vrep
import threading
import random
import time
import math
from queue import Queue


class Robot(threading.Thread):
    RUNNING = 0
    STOPPED = 1
    TICKS_PER_REVOLUTION = 16

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
        print("Let motor ticks {0} , right motor ticks {1}".format(
            self.get_left_motor_ticks(), self.get_right_motor_ticks()))
        if self.message_queue.qsize() > 0:
            msg = self.message_queue.get()
        self.motor1(10)
        self.motor2(10, False)

        detected, distance = self.read_mid_ultra_sonic()
        if detected:
            return
            self.state = Robot.STOPPED
            status = ("Robot {0} detected element on distance  : {1}".format(
                self.name, distance))
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

        self.__left_motor_ticks = 0
        self.__left_motor_old_position = 0  # This is used to calculate ticks

        self.__right_motor_ticks = 0
        self.__right_motor_old_position = 0  # This is used to calculate ticks
        self.setup()

    def run(self):
        while self.run_event.is_set():
            self.__calculate_ticks()
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

    def get_left_motor_coordinates(self):
        """
        return a tuple contating the coordinates (x,y,z) for the left motor joint in meters
        """
        return vrep.simxGetObjectPosition(self.clientID,self.left_motor,-1,vrep.simx_opmode_blocking)[1]

    def get_right_motor_coordinates(self):
        """
        return a tuple contating the coordinates (x,y,z) for the right motor joint in meters
        """
        return vrep.simxGetObjectPosition(self.clientID,self.right_motor,-1,vrep.simx_opmode_blocking)[1]
                
    def get_left_motor_ticks(self):
        return self.__left_motor_ticks

    def get_right_motor_ticks(self):
        return self.__right_motor_ticks

    def __calculate_ticks(self):
        position = vrep.simxGetJointPosition(
            self.clientID, self.left_motor, vrep.simx_opmode_blocking)[1]
        if position < 0:
            position += 2 * math.pi
        difference = abs(position - self.__left_motor_old_position)
        forward = False
        if difference > math.pi:
            if position < self.__left_motor_old_position:
                forward = True
            else:
                forward = False
        else:
            if position > self.__left_motor_old_position:
                forward = True
            else:
                forward = False
        min_angle = min(2 * math.pi - difference, difference)
        # The cast is just for printing beauty
        delta_ticks = int(
            (min_angle) // (2 * math.pi / Robot.TICKS_PER_REVOLUTION))
        if delta_ticks > 0:
            self.__left_motor_ticks += delta_ticks
            if forward:
                self.__left_motor_old_position = (self.__left_motor_old_position + delta_ticks * (
                    2 * math.pi / Robot.TICKS_PER_REVOLUTION)) % (2 * math.pi)
            else:
                self.__left_motor_old_position = (self.__left_motor_old_position - delta_ticks * (
                    2 * math.pi / Robot.TICKS_PER_REVOLUTION)) % (2 * math.pi)

        position = vrep.simxGetJointPosition(
            self.clientID, self.right_motor, vrep.simx_opmode_blocking)[1]
        if position < 0:
            position += 2 * math.pi
        difference = abs(position - self.__right_motor_old_position)
        forward = False
        if difference > math.pi:
            if position < self.__right_motor_old_position:
                forward = True
            else:
                forward = False
        else:
            if position > self.__right_motor_old_position:
                forward = True
            else:
                forward = False

        min_angle = min(2 * math.pi - difference, difference)
        # The cast is just for printing beauty
        delta_ticks = int(
            (min_angle) // (2 * math.pi / Robot.TICKS_PER_REVOLUTION))
        if delta_ticks > 0:
            self.__right_motor_ticks += delta_ticks
            if forward:
                self.__right_motor_old_position = (self.__right_motor_old_position + delta_ticks * (
                    2 * math.pi / Robot.TICKS_PER_REVOLUTION)) % (2 * math.pi)
            else:
                self.__right_motor_old_position = (self.__right_motor_old_position - delta_ticks * (
                    2 * math.pi / Robot.TICKS_PER_REVOLUTION)) % (2 * math.pi)

    def __set_motor(self, motor_handle, value, direction):
        """
        Sets the motor speed proportional to value and the forward/backward direction
        """
        if not isinstance(direction, bool):
            raise ValueError("Parameter direction must be boolean")
        if not isinstance(value, int) and (value > 255 or value < 0):
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

    def __read_ultrasonic(self, sensor_handle):
        state, detected_point = vrep.simxReadProximitySensor(
            self.clientID, sensor_handle, vrep.simx_opmode_blocking)[1:3]
        if state == 1:
            distance = math.sqrt(
                detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
            return (True, distance)
        return (False, None)

    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)
