import vrep
import threading
import time
import math
from Queue import Queue


class Robot(threading.Thread):


    # States
    SCAN = 0
    FORAGE = 1
    CLUSTER = 2
    FOLLOWER = 3
    AVOID = 4
    EXIT = 5
    STOP = 6
    

    # Physical properties
    ROBOT_LENGTH = 0.174
    ROBOT_WIDTH = 0.11
    WHEEL_R = 33e-3
    WHEEL_L = 92e-3
    TICKS_PER_REVOLUTION = 16
    METERS_PER_TICK = (2 * math.pi * WHEEL_R) / TICKS_PER_REVOLUTION

    # Params
    length_arena = 0
    width_arena = 0

    DISTANCE_DECREMENT = 0.2

    @staticmethod
    def distance(x2, y2, x1, y1):
        return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

    def print_ultrasonic_values(self):
        print "---------------------------------------------------"
        print "left = {0}".format(self.read_left_ultra_sonic())
        print "mid = {0}".format(self.read_mid_ultra_sonic())
        print "right = {0}".format(self.read_right_ultra_sonic())
        print "---------------------------------------------------"

    def update_odometry(self, r_dir=1, l_dir=1):
        
        right_pos = self.get_right_motor_coordinates()[0:2]
        left_pos = self.get_left_motor_coordinates()[0:2]
        old_right_pos = self.prev_right_pos
        old_left_pos = self.prev_left_pos
        d_right = r_dir * Robot.distance(right_pos[0], right_pos[1], old_right_pos[0], old_right_pos[1])
        d_left = l_dir * Robot.distance(left_pos[0], left_pos[1], old_left_pos[0], old_left_pos[1])
        self.prev_right_pos = right_pos
        self.prev_left_pos = left_pos
        d_center = (d_right + d_left) / 2
        phi = (d_right - d_left) / Robot.WHEEL_L
        x_dt = d_center * math.cos(self.theta)
        y_dt = d_center * math.sin(self.theta)
        theta_dt = phi
        theta_new = self.theta + theta_dt
        self.pos_x += x_dt
        self.pos_y += y_dt
        self.theta = math.atan2(math.sin(theta_new), math.cos(theta_new))

    def go_to_angle(self, theta_d, tolerance=0.01, cc=True):
        print "Target theta is %s" % theta_d
        if abs(theta_d) > 90:
            cc = False

        u = self.SPEED /5
        self.motor1(u,cc)
        self.motor2(u,not cc)
        Kp = 5
        Ki = 0.3
        Kd = 0.1

        e_1 = 0
        e_acc = 0
        previous_time = time.time()-1
        while True:
            e = theta_d - self.theta
            e = math.atan2(math.sin(e), math.cos(e))

            # Calculate dt
            c_time = time.time()
            dt = c_time - previous_time
            if(dt == 0):
                continue
            u = Kp*e + Ki * (e_acc+e*dt) + Kd * (e-e_1)/dt
            u = int(round(u))
            
            cc = u < 0
            u = abs(u)
            
            print "Error in degrees %s and U = %s , CC = %s" % (math.degrees(e),u,cc)

            e_1 = e
            e_acc += e*dt
            previous_time=c_time
            self.motor1(u,cc)
            self.motor2(u,not cc)
            if abs(e) < tolerance:
                self.motor1(0)
                self.motor2(0)
                return
            if cc:
                self.update_odometry(r_dir=-1)
            else:
                self.update_odometry(l_dir=-1)

    def go_to_point(self, x_g, y_g, tolerance=0.2):
        u_x = x_g - self.pos_x
        u_y = y_g - self.pos_y
        if not (abs(u_x) < 0.01 or abs(u_y) < 0.01):
            theta_g = math.atan2(u_y, u_x)
            self.go_to_angle(theta_g)
        self.motor1(self.SPEED)
        self.motor2(self.SPEED)
        while True:
            # print Robot.distance(x_g, y_g, self.pos_x, self.pos_y)
            if Robot.distance(x_g, y_g, self.pos_x, self.pos_y) < tolerance:
                self.motor1(0)
                self.motor2(0)
                return
            self.update_odometry()

    def go_to_point2(self, x_g, y_g, tolerance=0.01, us_check=True):
        u_x = x_g - self.pos_x
        u_y = y_g - self.pos_y
        if not (abs(u_x) < 0.01 or abs(u_y) < 0.01):
            theta_g = math.atan2(u_y, u_x)
            self.go_to_angle(theta_g)
        u_mag = Robot.distance(u_x, u_y, 0, 0)
        self.motor1(self.SPEED)
        self.motor2(self.SPEED)
        start_x = self.pos_x
        start_y = self.pos_y
        if us_check:
            ref_left_us = self.read_left_ultra_sonic()
            if ref_left_us[0]:
                ref_left_us = ref_left_us[1]
            else:
                us_check = False
        while True:
            covered = Robot.distance(self.pos_x, self.pos_y, start_x, start_y)
            # print "u_mag = ", u_mag, "covered = ", covered
            if abs(u_mag - covered) < tolerance:
                self.motor1(0)
                self.motor2(0)
                return
            if us_check:
                left_us = self.read_left_ultra_sonic()
                right_us = self.read_right_ultra_sonic()
                if left_us[0] and not right_us[0] and left_us[1] > ref_left_us + 0.01:
                    self.motor1(self.SPEED - 1)
                    self.motor2(self.SPEED)
                elif left_us[0] and not right_us[0] and left_us[1] < ref_left_us - 0.01:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED - 1)
                else:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED)
            self.update_odometry()

    def setup(self):
        """
        The analogy of setup() inside micro-controller.
        """
        if not self.read_mid_ultra_sonic()[0]:
            self.robot_ID = 0
        else:
            self.robot_ID = 1  # replaced later by distributed hash table code
        if self.robot_ID == 0:
            self.state = Robot.SCAN
        else:
            self.state = Robot.FOLLOWER

    def loop(self):
        """
        The analogy of loop() inside micro-controller,write your code inside
        """
        if self.state == Robot.STOP:
            self.motor1(0)
            self.motor2(0)
            return
        elif self.state == Robot.EXIT:
            self.go_to_point2(0, -0.3)
            self.motor1(0)
            self.motor2(0)
            self.state = Robot.STOP
            return
        elif self.state == Robot.SCAN:
            self.prev_right_pos = self.get_right_motor_coordinates()[0:2]
            self.prev_left_pos = self.get_left_motor_coordinates()[0:2]
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            start_x = self.pos_x
            start_y = self.pos_y
            ref_left_us = self.read_left_ultra_sonic()[1]
            while True:
                left_us = self.read_left_ultra_sonic()
                mid_us = self.read_mid_ultra_sonic()
                right_us = self.read_right_ultra_sonic()
                if left_us[0] and mid_us[0] and right_us[0] and mid_us[1] < 0.15:
                    self.motor1(0)
                    self.motor2(0)
                    end_x = self.pos_x
                    end_y = self.pos_y
                    Robot.length_arena = Robot.distance(end_x, end_y, start_x, start_y)
                    break
                if left_us[0] and not right_us[0] and left_us[1] > ref_left_us + 0.01:
                    self.motor1(self.SPEED-1)
                    self.motor2(self.SPEED)
                elif left_us[0] and not right_us[0] and left_us[1] < ref_left_us - 0.01:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED-1)
                else:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED)
                self.update_odometry()
            theta_d = self.theta - math.pi / 2
            theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
            self.go_to_angle(theta_d, tolerance=0.015)
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            start_x = self.pos_x
            start_y = self.pos_y
            ref_left_us = self.read_left_ultra_sonic()[1]
            while True:
                left_us = self.read_left_ultra_sonic()
                mid_us = self.read_mid_ultra_sonic()
                right_us = self.read_right_ultra_sonic()
                if left_us[0] and mid_us[0] and right_us[0] and mid_us[1] < 0.15:
                    self.motor1(0)
                    self.motor2(0)
                    end_x = self.pos_x
                    end_y = self.pos_y
                    Robot.width_arena = Robot.distance(end_x, end_y, start_x, start_y)
                    break
                if left_us[0] and not right_us[0] and left_us[1] > ref_left_us + 0.01:
                    self.motor1(self.SPEED - 1)
                    self.motor2(self.SPEED)
                elif left_us[0] and not right_us[0] and left_us[1] < ref_left_us - 0.01:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED - 1)
                else:
                    self.motor1(self.SPEED)
                    self.motor2(self.SPEED)
                self.update_odometry()
            theta_d = self.theta - math.pi / 2
            theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
            self.go_to_angle(theta_d, tolerance=0.015)
            self.state = Robot.FORAGE
        elif self.state == Robot.FORAGE:
            # Down motion
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            self.go_to_point2(self.pos_x, self.pos_y - Robot.length_arena)

            Robot.width_arena -= Robot.DISTANCE_DECREMENT
            if Robot.width_arena <= 0:
                self.motor1(0)
                self.motor2(0)
                self.theta = - math.pi / 2
                self.state = Robot.EXIT
                return
            else:
                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.go_to_angle(theta_d, tolerance=0.015)

            # Left motion
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            self.go_to_point2(self.pos_x - Robot.width_arena, self.pos_y, us_check=False)

            Robot.length_arena -= Robot.DISTANCE_DECREMENT
            if Robot.length_arena <= 0:
                self.motor1(0)
                self.motor2(0)
                self.theta = math.pi
                self.state = Robot.EXIT
                return
            else:
                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.go_to_angle(theta_d, tolerance=0.015)

            # Up motion
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            self.go_to_point2(self.pos_x, self.pos_y + Robot.length_arena)

            Robot.width_arena -= Robot.DISTANCE_DECREMENT
            if Robot.width_arena <= 0:
                self.motor1(0)
                self.motor2(0)
                self.theta = math.pi / 2
                self.state = Robot.EXIT
                return
            else:
                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.go_to_angle(theta_d, tolerance=0.015)

            # Right motion
            self.motor1(self.SPEED)
            self.motor2(self.SPEED)
            self.go_to_point2(self.pos_x + Robot.width_arena, self.pos_y)

            Robot.length_arena -= Robot.DISTANCE_DECREMENT
            if Robot.length_arena <= 0:
                self.motor1(0)
                self.motor2(0)
                self.theta = 0
                self.state = Robot.EXIT
                return
            else:
                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.go_to_angle(theta_d, tolerance=0.015)

    def __init__(self, name, sim):
        threading.Thread.__init__(self)
        self.message_queue = Queue()  # Queue for received messages
        self.sim = sim
        self.run_event = self.sim.run_event
        self.clientID = self.sim.clientID
        self.name = name
        self.state = Robot.STOP
        self.SPEED = 20
        self.robot_ID = 0
        self.pos_x = 0
        self.pos_y = 0
        self.theta = math.pi / 2
        self.prev_right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_pos = None
        self.prev_left_pos = None

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

        self.color_sensor = vrep.simxGetObjectHandle(
            self.clientID, "Vision_sensor{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]
        self.__left_motor_ticks = 0
        self.__left_motor_old_position = 0  # This is used to calculate ticks

        self.__right_motor_ticks = 0
        self.__right_motor_old_position = 0  # This is used to calculate ticks
        
        # Gripper handlers
        self.GripperProxSensor = vrep.simxGetObjectHandle(
            self.clientID, "RG2_attachProxSensor{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]
        self.GripperAttachPoint = vrep.simxGetObjectHandle(
            self.clientID, "RG2_attachPoint{0}".format(self.postfix),
            vrep.simx_opmode_blocking)[1]
        self.__GrippedShape = None
        

        # Request streaming values
        vrep.simxGetObjectPosition(self.clientID, self.right_motor, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.left_motor, -1, vrep.simx_opmode_streaming)

        vrep.simxReadProximitySensor(self.clientID,self.left_ultrasonic,vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID,self.mid_ultrasonic,vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID,self.right_ultrasonic,vrep.simx_opmode_streaming)

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
        
        return vrep.simxGetObjectPosition(self.clientID, self.left_motor, -1, vrep.simx_opmode_buffer)[1]

    def get_right_motor_coordinates(self):
        """
        return a tuple contating the coordinates (x,y,z) for the right motor joint in meters
        """
       
        return vrep.simxGetObjectPosition(self.clientID, self.right_motor, -1, vrep.simx_opmode_buffer)[1]
        

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
            vrep.simx_opmode_oneshot)

    def read_left_ultra_sonic(self):
        return self.__read_ultrasonic(self.left_ultrasonic)

    def read_mid_ultra_sonic(self):
        return self.__read_ultrasonic(self.mid_ultrasonic)

    def read_right_ultra_sonic(self):
        return self.__read_ultrasonic(self.right_ultrasonic)

    def __read_ultrasonic(self, sensor_handle):
        state, detected_point = vrep.simxReadProximitySensor(
            self.clientID, sensor_handle, vrep.simx_opmode_buffer)[1:3]
        if state == 1:
            distance = math.sqrt(
                detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
            return (True, distance)
        return (False, None)

    def read_color_sensor(self):
        """
        This function returns a list of 4 values: light intensity, red, green, blue averges across the pixels of the detector in case of success,
        these values range from 0-255. Or it returns None in case of faliures
        """
        if hasattr(self, "__first_color_sensor_read") == False:
            self.__first_color_sensor_read = True
            result = vrep.simxReadVisionSensor(
                self.clientID, self.color_sensor, vrep.simx_opmode_streaming)
        else:
            result = vrep.simxReadVisionSensor(
                self.clientID, self.color_sensor, vrep.simx_opmode_buffer)

        if result[0] != vrep.simx_return_ok:
            return None

        result = [e * 255 for e in result[2][0][10:14]]
        return result

    def grip(self):
        """
        This function simulates a magentic gripping technique as the following: 
        1. Use ray shaped ultrasonic sensor to detect if there is a shape that can be gripped
        2. Attach this shape to the a force sensor - used as connector - to the robot

        limitations: 
        1. It will grip anything can be detected by the ultrasonic sensor used in it
        2. It will not grip items farther than 2.5 cm (this is adjustable in the model)

        return value: True if gripped an item, otherwise false.
        """
        if self.__GrippedShape is not None:
            return False

        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.GripperProxSensor, vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok or detectionState is False:
            return False
        self.__GrippedShape = detectedObjectHandle
        returnCode = vrep.simxSetObjectParent(self.clientID, self.__GrippedShape,
                                 self.GripperAttachPoint, True, vrep.simx_opmode_blocking)
        return True if returnCode == vrep.simx_return_ok else False

    def degrip(self):

        vrep.simxSetObjectParent(
            self.clientID, self.__GrippedShape, -1, True, vrep.simx_opmode_blocking)
        self.__GrippedShape = None

    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)