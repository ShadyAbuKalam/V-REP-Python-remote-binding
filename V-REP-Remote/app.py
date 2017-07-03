# Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved.
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
#
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
#
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.3.2 on August
# 29th 2016

# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
from robot import Robot
try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import threading


class Simulator():
    def __init__(self):
        self.clientID = -1  # The connection id
        # An event used to stop the loops inside the robot threads
        self.run_event = threading.Event()
        self.robots = []  # A list of robots' threads

    def start(self):
        print('Program started')
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True,
                                       True, 5000, 5)  # Connect to V-REP
        if self.clientID != -1:
            print('Connected to remote API server')
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
           
            # Now try to retrieve robot identifers in a blocking fashion (i.e. a service call):
            # We know that each robot name has the following
            returnCode, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(
                self.clientID, vrep.sim_object_shape_type, 0, vrep.simx_opmode_blocking)
            for name in stringData:
                # Assume that we use DragonOne, so thier names will start with
                # DragonOne
                if "DragonOne" in name:
                    r = Robot(name, self)
                    self.robots.append(r)

            print("Found {0} robots".format(len(self.robots)))
            
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)

            # Setup robot threads
            for r in self.robots:
                r.setup()

            # Set run Event and start the robot threads
            self.run_event.set()
            for r in self.robots:
                r.start()
            


        else:
            print('Failed connecting to remote API server')

    def stop(self):
        # Clear the run event to allow robots loop to stop
        self.run_event.clear()

        # Wait for each thread to join
        for r in self.robots:
            r.join()

        # Stop the simulation
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

        # Before closing the connection to V-REP, make sure that the last
        # command sent out had time to arrive. You can guarantee this with (for
        # example):

        vrep.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)

    def broadcast(self, sender, message):
        status = "Broadcasting message from {0} to {1} robots".format(
            sender, len(self.robots))
        vrep.simxAddStatusbarMessage(
            self.clientID, status, vrep.simx_opmode_blocking)
        for r in self.robots:
            if (r == sender):
                continue
            r.message_queue.put(message)


def main():
    sim = Simulator()
    try:
        sim.start()
        while(True):
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Keyboard interrupted.. Currently closing threads")
        sim.stop()
    except Exception as e:
        sim.stop()
        print(e)
    print("Good Bye")


if (__name__ == "__main__"):
    main()
