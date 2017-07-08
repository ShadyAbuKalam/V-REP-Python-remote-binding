import operator

import utils
import vrep
import threading
import time
import math
from Queue import Queue, Empty


class Robot(threading.Thread):

    #Directions
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    #Message types
    ROTATION_MESSAGE = 0
    END_OF_ARENA_MESSAGE = 1
    ORDERING_MESSAGE = 2
    DETACHING_MESSAGE = 3
    #Color
    RED = 0
    GREEN = 1
    BLUE = 2
    BLACK = 3
    WHITE = 4

    MAX_VELOCITY = 100
    #Gripper state
    EMPTY = 0
    LIGHT = 1
    HEAVY = 2
    # States
    SCAN = 0
    FORAGE = 1
    CLUSTER = 2
    FOLLOWER = 3
    AVOID = 4
    EXIT = 5
    STOP = 6
    
    GRIPPING = 7
    DEPOSITING = 8
    OBSTACLE_HANDLING = 9

    STARTING_UP = 10
    DETACHING = 11
    # Physical properties
    ROBOT_LENGTH = 0.174
    ROBOT_WIDTH = 0.11
    WHEEL_R = 33e-3
    WHEEL_L = 99e-3
    TICKS_PER_REVOLUTION = 16
    METERS_PER_TICK = (2 * math.pi * WHEEL_R) / TICKS_PER_REVOLUTION

    # Params
    length_arena = 0
    width_arena = 0

    DISTANCE_DECREMENT = 0.2
    FORAGING_STOP_DISTANCE = 0.3

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
        # print("X = {0}, Y = {1}, Theta = {2}".format(self.pos_x,self.pos_y,self.theta))

        right_pos = self.get_right_motor_coordinates()
        left_pos = self.get_left_motor_coordinates()
        

        if True:

            self.pos_x = (left_pos[0]+right_pos[0])/2 
            self.pos_y = (left_pos[1]+right_pos[1])/2 


            diff = [r - l for r,l in zip(right_pos,left_pos)]
            theta_new = math.atan2(diff[1],diff[0]) + math.pi/2

        else:

            old_right_pos = self.prev_right_pos
            old_left_pos = self.prev_left_pos
            d_right = r_dir * Robot.distance(right_pos[0], right_pos[1], old_right_pos[0], old_right_pos[1])
            d_left = l_dir * Robot.distance(left_pos[0], left_pos[1], old_left_pos[0], old_left_pos[1])
            self.prev_right_pos = right_pos
            self.prev_left_pos = left_pos
            d_center = (d_right + d_left) / 2
            x_dt = d_center * math.cos(self.theta)
            y_dt = d_center * math.sin(self.theta)

            self.pos_x += x_dt
            self.pos_y += y_dt
            phi = (d_right - d_left) / Robot.WHEEL_L
            theta_dt = phi
            theta_new = self.theta + theta_dt

        self.theta = math.atan2(math.sin(theta_new), math.cos(theta_new))


    def go_to_angle(self, theta_d, tolerance=0.015, cc=True):

        if( abs(theta_d - self.theta) < tolerance):
            return
        print "Target theta is %s" % theta_d



        Kp = 10
        Ki = 0.8
        Kd = 0.5

        e_1 = 0
        e_acc = 0
        previous_time = time.time()-1
        while True:
            
            e = theta_d - self.theta
            e = math.atan2(math.sin(e), math.cos(e))

            # Calculate dt
            c_time = time.time()
            dt = c_time - previous_time
            if(dt <= 0.001):
                continue
            u = Kp*e + Ki * (e_acc+e*dt) + Kd * (e-e_1)/dt
            v_right, v_left = self.__convert_unicycle_to_differential(0,u)


            # print "Error in rad %s and U = %s , CC = %s" % (e,u,cc)

            e_1 = e
            e_acc += e*dt
            previous_time=c_time
            # if(abs(u)>255):
            #     u = 255 # Sometimes for unknown reasons u get values in order of thousands, so it's an odd ball solution
            self.motor1(v_left)
            self.motor2(v_right)
            if abs(e) < tolerance:
                self.brake()
                return
            if cc:
                self.update_odometry(r_dir=-1)
            else:
                self.update_odometry(l_dir=-1)

    def go_to_point(self, x_g, y_g, tolerance=0.075,base_velocity=None,avoidance = False):
        self.go_to_point_target = (x_g,y_g)
        if(base_velocity is None):
            base_velocity = self.SPEED
        angular_Kp = 10
        angular_Ki = 0.7
        angular_Kd = 0.1
        e_1 = 0
        e_acc = 0
        previous_time = time.time()-1
        blending_alpha = 0.25

        u_x = x_g - self.pos_x
        u_y = y_g - self.pos_y
        theta_d = math.atan2(u_y, u_x)


        if not (math.sqrt(u_x**2 + u_y**2)< 0.05 ):
            self.go_to_angle(theta_d)

        while True:
            if avoidance:
                left_distance = 0.4
                left_us = self.read_left_ultra_sonic()
                if left_us[0]:
                    left_distance = left_us[1]
                robot_frame_left_x = math.cos(math.pi / 4) * left_distance + 0.07
                robot_frame_left_y = math.sin(math.pi / 4) * left_distance + 0.05
                us_left_x = math.cos(self.theta) * robot_frame_left_x - math.sin(
                    self.theta) * robot_frame_left_y + self.pos_x
                us_left_y = math.sin(self.theta) * robot_frame_left_x + math.cos(
                self.theta) * robot_frame_left_y + self.pos_y

                mid_distance = 0.4
                mid_us = self.read_mid_ultra_sonic()
                if mid_us [0]:
                    mid_distance = mid_us[1]
                robot_frame_mid_x = math.cos(0) * mid_distance + 0.08
                robot_frame_mid_y = math.sin(0) * mid_distance + 0
                us_mid_x = math.cos(self.theta) * robot_frame_mid_x - math.sin(
                    self.theta) * robot_frame_mid_y + self.pos_x
                us_mid_y = math.sin(self.theta) * robot_frame_mid_x + math.cos(self.theta) * robot_frame_mid_y + self.pos_y

                right_distance =0.4
                right_us = self.read_right_ultra_sonic()
                if right_us[0]:
                    right_distance = right_us[1]
                robot_frame_right_x = math.cos(-math.pi / 4) * right_distance + 0.07
                robot_frame_right_y = math.sin(-math.pi / 4) * right_distance - 0.05
                us_right_x = math.cos(self.theta) * robot_frame_right_x - math.sin(
                    self.theta) * robot_frame_right_y + self.pos_x
                us_right_y = math.sin(self.theta) * robot_frame_right_x + math.cos(
                    self.theta) * robot_frame_right_y + self.pos_y

                self.brake()
                if mid_distance < 0.1 or left_distance <0.1 or right_distance <0.1:
                    blending_alpha = 0

                elif left_distance == 0.4 and mid_distance == 0.4 and right_distance == 0.4:
                    blending_alpha = 1
                    e_acc = 0
                    e_1 = 0
                else:
                    blending_alpha = 0.25

                print(left_distance,mid_distance,right_distance,blending_alpha )
                u_x = blending_alpha * (x_g - self.pos_x) + (1 - blending_alpha) * (us_left_x + 0.5* us_mid_x +  us_right_x - 2.5*self.pos_x)
                u_y = blending_alpha * (y_g - self.pos_y) + (1 - blending_alpha) * ( us_left_y + 0.5* us_mid_y +  us_right_y - 2.5*self.pos_y)
                theta_d = math.atan2(u_y, u_x)

            if (self.state == Robot.FORAGE):
                mid_us = self.read_mid_ultra_sonic()
                if (mid_us[0] and mid_us[1] <= 0.20):
                    self.state = Robot.OBSTACLE_HANDLING
                    return False

            e = theta_d - self.theta
            e = math.atan2(math.sin(e), math.cos(e))

            # Calculate dt
            c_time = time.time()
            dt = c_time - previous_time
            if(dt == 0):
                continue
            u = angular_Kp*e + angular_Ki * (e_acc+e*dt) + angular_Kd * (e-e_1)/dt

            vRight, vLeft = self.__convert_unicycle_to_differential(base_velocity,u)

            #Find the max and min of vLeft/vRight
            velMax = max(vLeft, vRight)
            velMin = min(vLeft, vRight)
            if (velMax > Robot.MAX_VELOCITY):
                vLeft -= (velMax - Robot.MAX_VELOCITY)
                vRight -= (velMax - Robot.MAX_VELOCITY)
            elif (velMin < -Robot.MAX_VELOCITY):
                vLeft -= (velMin + Robot.MAX_VELOCITY)
                vRight -= (velMin + Robot.MAX_VELOCITY)
            # if(self.postfix):
            #     print("Error in radians and input to controller and VLeft & VRight",e,u,vLeft,vRight)
            self.motor1(vLeft)
            self.motor2(vRight)
            # print Robot.distance(x_g, y_g, self.pos_x, self.pos_y)
            if Robot.distance(x_g, y_g, self.pos_x, self.pos_y) < tolerance:
                self.brake()
                return True
            self.update_odometry()


    def setup(self):
        """
        The analogy of setup() inside micro-controller.
        It must be called after the start of simulation from the app.py
        """
        self.isHead = False
        self.isTail = False
        self.SuccessorRobot = None
        self.PredecessorRobot = None
        #Starting up algorithm
        self.robots = {}
        self.starting_up_intiail_time = time.time()
        self.state = Robot.STARTING_UP
        # Foraging variables
        self.foragin_motion = None

        self.gripper_state = Robot.EMPTY

        # Follower algorithm state 
        self.next_follower_rotation = []

        #Reset old values for speed saved in simulator from the last simulation pass
        self.brake()

        #Initialize these two important variables :D 
        self.prev_right_pos = self.get_right_motor_coordinates()
        self.prev_left_pos = self.get_left_motor_coordinates()

        #Intial location for robots
        self.update_odometry()

        #Related to detach algorithm
        self.go_to_point_target = None
        self.foraging_target_point = None

        #Exit point
        self.exit_point = None

    def loop(self):
        """
        The analogy of loop() inside micro-controller,write your code inside
        """
        if self.state == Robot.STARTING_UP:
            if(time.time() - self.starting_up_intiail_time <5):
                self.send_ordering_message()
            else:
                distance = Robot.distance(self.pos_x,self.pos_y,0,0)
                self.robots[self.handle]= distance
                sorted_robots = sorted(self.robots.items(), key=operator.itemgetter(1))
                i  = sorted_robots.index((self.handle,distance))
                print(sorted_robots)
                self.exit_point = (0,-1.5-(len(sorted_robots) - i -1)*0.4)

                if(i > 0):
                    self.PredecessorRobot = sorted_robots[i - 1][0]
                if(i<len(sorted_robots)-1):
                    self.SuccessorRobot = sorted_robots[i + 1][0]

                if(self.PredecessorRobot is None):
                    self.isHead = True
                if(self.SuccessorRobot is None):
                    self.isTail = True

                #todo : Shift to the next state according to the the position
                if(self.isHead):
                    self.state = Robot.SCAN
                else:
                    self.state = Robot.FOLLOWER

        elif self.state == Robot.STOP:
            self.brake()
            return
        elif self.state == Robot.EXIT:
            self.go_to_point(*self.exit_point)
            self.state = Robot.STOP
            return
        elif self.state == Robot.SCAN:

            for i in range(2):
                reached_wall = False
                start_x = self.pos_x
                start_y = self.pos_y
                end_x = None
                end_y = None
                ref_left_us = False
                while (not ref_left_us):
                    ref_left_us = self.read_left_ultra_sonic()[1]
                while reached_wall == False:
                    left_us = self.read_left_ultra_sonic()
                    mid_us = self.read_mid_ultra_sonic()
                    right_us = self.read_right_ultra_sonic()
                    if left_us[0] and mid_us[0] and right_us[0] and mid_us[1] < 0.15:
                        v_right,v_left = self.__convert_unicycle_to_differential(0)

                        end_x = self.pos_x
                        end_y = self.pos_y
                        reached_wall = True

                    if left_us[0] and not right_us[0] and left_us[1] > ref_left_us + 0.01:
                        v_right,v_left = self.__convert_unicycle_to_differential(self.SPEED, 0.1)

                    elif left_us[0] and not right_us[0] and left_us[1] < ref_left_us - 0.01:
                        v_right,v_left = self.__convert_unicycle_to_differential(self.SPEED, -0.1)

                    else:
                        v_right,v_left = self.__convert_unicycle_to_differential(self.SPEED)

                    self.motor1(v_left)
                    self.motor2(v_right)
                    self.update_odometry()

                if i == 0:
                    Robot.length_arena = Robot.distance(end_x, end_y, start_x, start_y)
                else:
                    Robot.width_arena = Robot.distance(end_x, end_y, start_x, start_y)

                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.send_rotation_message()
                self.go_to_angle(theta_d, tolerance=0.015)


            self.state = Robot.FORAGE
            self.foragin_motion = Robot.DOWN
        elif self.state == Robot.FORAGE:
            #If a follower was updated to be a head it may have previous rotation points
            while(len(self.next_follower_rotation) > 0):
                coord = self.next_follower_rotation.pop(0)
                self.go_to_point(coord['x'],coord['y'])
            if(self.foraging_target_point is not None):
                if(self.go_to_point(self.foraging_target_point[0],self.foraging_target_point[1])):
                    self.foraging_target_point = None
                else:
                    return

            else:

                if (self.foragin_motion == Robot.DOWN ):
                    self.foraging_target_point = (self.pos_x, self.pos_y - Robot.length_arena)

                elif self.foragin_motion == Robot.UP:
                    self.foraging_target_point = (self.pos_x, self.pos_y + Robot.length_arena)

                elif self.foragin_motion == Robot.LEFT:
                    self.foraging_target_point = (self.pos_x - Robot.width_arena, self.pos_y)

                elif self.foragin_motion == Robot.RIGHT:
                    self.foraging_target_point = (self.pos_x + Robot.width_arena, self.pos_y)

                if(self.go_to_point(self.foraging_target_point[0],self.foraging_target_point[1])):
                    self.foraging_target_point = None
                else:
                    return
            stop = False
            if(self.foragin_motion == Robot.DOWN or self.foragin_motion == Robot.UP):

                Robot.width_arena -= Robot.DISTANCE_DECREMENT
                stop = Robot.width_arena <= Robot.FORAGING_STOP_DISTANCE


            elif self.foragin_motion == Robot.LEFT or self.foragin_motion == Robot.RIGHT:
                Robot.length_arena -= Robot.DISTANCE_DECREMENT
                stop= Robot.length_arena <= Robot.FORAGING_STOP_DISTANCE
            if stop:
                self.brake()
                self.theta = - math.pi / 2
                self.state = Robot.EXIT
                self.send_end_of_arena_message()
                return
            else:
                theta_d = self.theta - math.pi / 2
                theta_d = math.atan2(math.sin(theta_d), math.cos(theta_d))
                self.send_rotation_message()
                self.go_to_angle(theta_d, tolerance=0.015)

            #Update foraging motion to reflect the next direction
            if self.foragin_motion == self.DOWN:
                self.foragin_motion = self.LEFT
            elif self.foragin_motion == self.LEFT:
                self.foragin_motion = self.UP
            elif self.foragin_motion == self.UP:
                self.foragin_motion = self.RIGHT
            elif self.foragin_motion == self.RIGHT:
                self.foragin_motion = self.DOWN
    
        elif self.state == Robot.GRIPPING:

            start_x = self.pos_x
            start_y = self.pos_y
            velocity = self.__convert_unicycle_to_differential(self.SPEED/2)[0]
            self.motor1(velocity)
            self.motor2(velocity)
            while Robot.distance(start_x,start_y,self.pos_x,self.pos_y) <0.1:
                self.update_odometry()

            self.brake()


            if (self.grip()):
                self.state = Robot.DETACHING

            # else :
                # self.gripper_state = None
                # assert(False)  # It failed to grip
        elif self.state == Robot.DETACHING:
            self.send_detach_message()
            self.SuccessorRobot = None
            self.PredecessorRobot = None
            self.state = Robot.DEPOSITING
        elif self.state == Robot.DEPOSITING:
            if self.gripper_state == Robot.LIGHT:
                self.go_to_point(0,-0.3,avoidance=True)
                self.go_to_point(-0.5,-0.3)
                self.degrip()
                self.go_to_point(0,-0.3)

            elif self.gripper_state == Robot.HEAVY:
                self.go_to_point(1.6,-0.3,avoidance=True)
                self.go_to_point(2,-0.3)
                self.degrip()
                self.go_to_point(1.6,-0.3)

                                
            self.state = Robot.EXIT

        elif self.state == Robot.OBSTACLE_HANDLING:
            self.brake()
            color_sensor_reading = self.read_color_sensor()
            color = self.analyze_color(color_sensor_reading)
            if (color == Robot.GREEN ):
                self.state = Robot.GRIPPING
                self.gripper_state = Robot.HEAVY

            elif (color == Robot.RED):
                self.gripper_state = Robot.LIGHT
                self.state = Robot.GRIPPING

            else:
                self.state = Robot.FORAGE

        elif self.state == Robot.FOLLOWER:
            isObj_m,Dist_m=self.read_mid_ultra_sonic()
            isObj_l,Dist_l=self.read_left_ultra_sonic()
            isObj_r,Dist_r=self.read_right_ultra_sonic()
            if(len(self.next_follower_rotation) > 0 and Robot.distance(self.pos_x,self.pos_y,self.next_follower_rotation[0]['x'],self.next_follower_rotation[0]['y']) <0.25):
                print "Robot will rotate according to message"
                self.brake()
                coord= self.next_follower_rotation[0]
                print "Coord",coord,self.pos_x,self.pos_y
                self.go_to_point(coord['x'],coord['y'],tolerance=0.01)
                print "Follower is rotating"
                theta = utils.limitToOneOfPoles(self.theta)

                self.go_to_angle(theta-math.pi/2)
                self.next_follower_rotation.pop(0)
            elif ((Dist_m is not None) and  (Dist_r is None)):##straight Motion :

                if(Dist_m>=0.29):
                    L = 0.2
                    theta = utils.limitToOneOfPoles(self.theta)
                    y = L*math.sin(theta)+self.pos_y
                    
                    x = L*math.cos(theta)+self.pos_x
                    
                    self.go_to_point(x,y,base_velocity=self.SPEED*2)
                    
                elif(Dist_m<0.29 and Dist_m>0.2):
                    theta = utils.limitToOneOfPoles(self.theta)

                    L = 0.1
                    y = L*math.sin(theta)+self.pos_y
                    x = L*math.cos(theta)+self.pos_x
                    self.go_to_point(x,y)
                    
                elif(Dist_m<=0.2):
                    self.brake()
            

            elif Dist_m is None:
                vRight,vLeft = self.__convert_unicycle_to_differential(self.SPEED*2,0)
                self.motor1(vRight)
                self.motor2(vLeft)


        self.update_odometry()

    def brake(self):
        self.motor1(0)
        self.motor2(0)

    def __init__(self, name, sim):
        threading.Thread.__init__(self)
        self.message_queue = Queue()  # Queue for received messages
        self.sim = sim
        self.run_event = self.sim.run_event
        self.clientID = self.sim.clientID
        self.name = name
        self.SPEED = 1.0
        self.pos_x = None
        self.pos_y = None
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

        self.beacon = vrep.simxGetObjectHandle(
                    self.clientID, "Beacon", vrep.simx_opmode_blocking)[1]

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
        vrep.simxGetObjectPosition(self.clientID, self.right_motor, self.beacon, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.left_motor, self.beacon, vrep.simx_opmode_streaming)

        vrep.simxReadProximitySensor(self.clientID,self.left_ultrasonic,vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID,self.mid_ultrasonic,vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID,self.right_ultrasonic,vrep.simx_opmode_streaming)


    def run(self):
        while self.run_event.is_set():
            # self.__calculate_ticks()
            self.process_messages()
            self.loop()
            # Sleep so other threads can be scheduled
            time.sleep(0.1)

    def broadcast(self, message):
        """
        This will broadcast a message to all other robots
        """
        self.sim.broadcast(self, message)

    def motor1(self, value):
        """
        Sets the motor1 speed proportional to value and the forward/backward direction
        """
        self.__set_motor(self.left_motor, value)

    def motor2(self, value):
        """
        Sets the motor1 speed proportional to value and the forward/backward direction
        """
        self.__set_motor(self.right_motor, value)

    def get_left_motor_coordinates(self):
        """
        return a tuple contating the coordinates (x,y,z) for the left motor joint in meters
        """
        
        return_code,pos = vrep.simxGetObjectPosition(self.clientID, self.left_motor, self.beacon, vrep.simx_opmode_buffer)
        while (return_code == vrep.simx_return_novalue_flag):
            return_code,pos = vrep.simxGetObjectPosition(self.clientID, self.left_motor, self.beacon, vrep.simx_opmode_buffer)

        if (return_code == vrep.simx_return_ok):
            return pos[0:2]
        return None
    def get_right_motor_coordinates(self):
        """
        return a tuple contating the coordinates (x,y,z) for the right motor joint in meters
        """
       
        return_code,pos= vrep.simxGetObjectPosition(self.clientID, self.right_motor, self.beacon, vrep.simx_opmode_buffer)
        while (return_code == vrep.simx_return_novalue_flag):
            return_code,pos= vrep.simxGetObjectPosition(self.clientID, self.right_motor, self.beacon, vrep.simx_opmode_buffer)

        if (return_code == vrep.simx_return_ok):
            return pos[0:2]
        return None
        

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

    def __set_motor(self, motor_handle, value):
        """
        Sets the motor speed proportional to value and the forward/backward direction
        """

        if abs(value)> 255:
            raise ValueError(
                "Parameter value must be  between 0 and 255, but {0} was passed".format(value))

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
        return_code,state, detected_point = vrep.simxReadProximitySensor(
            self.clientID, sensor_handle, vrep.simx_opmode_buffer)[0:3]
        while return_code == vrep.simx_return_novalue_flag:
            return_code,state, detected_point = vrep.simxReadProximitySensor(
            self.clientID, sensor_handle, vrep.simx_opmode_buffer)[0:3]
        if state == 1:
            distance = math.sqrt(
                detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
            return (True, distance)
        return (False, None)

    def analyze_color(self,color):
        if(color is None):
            return None
        rgb = color[1:]

        errors = {}
        w_error = math.sqrt((255-rgb[0])**2 + (255-rgb[1])**2 + (255-rgb[2])**2)
        if(w_error < 80):
            return None
        r_error = math.sqrt((255-rgb[0])**2 + rgb[1]**2 + rgb[2]**2)
        errors[Robot.RED] = r_error

        g_error =  math.sqrt((255-rgb[1])**2 + rgb[0]**2 + rgb[2]**2)
        errors[Robot.GREEN] = g_error
        
        b_error =  math.sqrt((255-rgb[2])**2 + rgb[0]**2 + rgb[1]**2)
        errors[Robot.BLUE] = b_error

        b_error = math.sqrt(rgb[0]**2 + rgb[1]**2 + rgb[2]**2)
        errors[Robot.BLACK] = b_error

        return min(errors,key=errors.get)

    def read_color_sensor(self):
        """
        This function returns a list of 4 values: light intensity, red, green, blue averges across the pixels of the detector in case of success,
        these values range from 0-255. Or it returns None in case of faliures
        """
        result = vrep.simxReadVisionSensor(
                self.clientID, self.color_sensor, vrep.simx_opmode_blocking)
        while result[0] == vrep.simx_return_novalue_flag:
            result = vrep.simxReadVisionSensor(
                self.clientID, self.color_sensor, vrep.simx_opmode_blocking)
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
        returnCode = vrep.simxSetObjectParent(self.clientID, detectedObjectHandle,
                                 self.GripperAttachPoint, False, vrep.simx_opmode_blocking)

        if returnCode == vrep.simx_return_ok:
            self.__GrippedShape = detectedObjectHandle
            vrep.simxSetObjectPosition(self.clientID, detectedObjectHandle, self.GripperAttachPoint, (0, 0.01, 0.045),
                                       vrep.simx_opmode_blocking)

            return True
        else:
            return False

    def degrip(self):

        vrep.simxSetObjectParent(
            self.clientID, self.__GrippedShape, -1, True, vrep.simx_opmode_blocking)
        self.__GrippedShape = None

    def send_rotation_message(self):
        if(self.isHead):
            message = {'type':Robot.ROTATION_MESSAGE,'data':{'x':self.pos_x,'y':self.pos_y}}
            self.broadcast(message)

    def send_end_of_arena_message(self):
        """
        Tell other robots that we are done
        """
        if (self.isHead):
            message = {'type': Robot.END_OF_ARENA_MESSAGE, 'data': None}
            self.broadcast(message)

    def send_ordering_message(self):
        distance = Robot.distance(self.pos_x,self.pos_y,0,0)
        message = {'type': Robot.ORDERING_MESSAGE, 'data': {'handler': self.handle, 'distance': distance}}
        self.broadcast(message)

    def send_detach_message(self):
        message = {'type': Robot.DETACHING_MESSAGE, 'data': {'coordinates':(self.go_to_point_target),'direction':self.foragin_motion},'target':self.SuccessorRobot
                   }
        self.broadcast(message)

    def process_messages(self):
        while (not self.message_queue.empty()):
            try:
                message = self.message_queue.get_nowait()
            except Empty:
                return
            if (message['type'] == Robot.ROTATION_MESSAGE):
                self.next_follower_rotation.append(message['data'])
            elif (message['type'] == Robot.END_OF_ARENA_MESSAGE):
                self.state = Robot.STOP
            elif (message['type'] == Robot.ORDERING_MESSAGE):
                self.robots[message['data']['handler']] = message['data']['distance']
            elif (message['type'] == Robot.DETACHING_MESSAGE):
                if(message['target'] != self.handle):
                    return
                self.PredecessorRobot = None
                self.state = Robot.FORAGE
                self.foraging_target_point = message['data']['coordinates']
                self.foragin_motion = message['data']['direction']


    def __convert_unicycle_to_differential(self,v,w=0):
        vRight = (2*v+w*Robot.WHEEL_L) / (2*Robot.WHEEL_R)
        vLeft = (2*v-w*Robot.WHEEL_L) / (2*Robot.WHEEL_R)
        print ("Converted V = {0} & W = {1} to VRight= {2} & Vleft = {3}".format(v,w,vRight,vLeft))
        return vRight,vLeft

    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)