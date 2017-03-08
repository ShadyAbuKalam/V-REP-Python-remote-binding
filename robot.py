# todo : Inheirt the thread object and initialize left and right motors
# handle values
import vrep
import threading
import random
import time
class robot(threading.Thread):
    # The analogus of setup() inside microcontoller
    def __init__(self, name, clientID,run_event):
        threading.Thread.__init__(self)
        self.run_event = run_event
        self.clientID = clientID
        self.name = name

        # Setup the postfix of the robot name
        self.postfix = ""
        try:

            i = self.name.index("#")
            self.postfix = self.name[i:]
        except:
            pass

        # Setup robot handle
        returnCode, self.handle = vrep.simxGetObjectHandle(
            clientID, name, vrep.simx_opmode_blocking)

        # Setup left motor and right motor handles respectively
        returnCode, self.left_motor = vrep.simxGetObjectHandle(
            clientID, "K3_leftWheelMotor{0}".format(self.postfix), vrep.simx_opmode_blocking)
        returnCode, self.right_motor = vrep.simxGetObjectHandle(
            clientID, "K3_rightWheelMotor{0}".format(self.postfix), vrep.simx_opmode_blocking)


    def run(self):
        # The analogus of loop() inside microcontoller,write your code inside the while loop
        while self.run_event.is_set():
            vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor,random.random()*10, vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor,random.random()*10, vrep.simx_opmode_blocking)
            time.sleep(2)
    def __repr__(self):
        return "A Robot with handle : {0}".format(self.handle)
