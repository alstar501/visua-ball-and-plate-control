# 控制机器人
import sys
import vrep
import math
import numpy as np
import  time
import sys

class Robot:

    def __init__(self,clientID):
        self.clientID = clientID
        self.servos = []
        self.vRGBs=None
        self.center=None
        self.laser_init = True
        self.pos_first = True
        self.orientation_first = True
        self.on_startup()

    #############################################################
    # helper functions for simulator interfacing
    #############################################################

    def connect_simulator(self):
        #establish connection to the simulator on localhost
        vrep.simxFinish(-1) # just in case, close all opened connections
        IP_address = "127.0.0.1"
        port = 19999 # port on which runs the continuous remote API
        waitUntilConnected = True
        doNotReconnectOnceDisconnected = True
        timeOutInMs = 5000
        commThreadCycleInMs = 10
        new_clientID = vrep.simxStart(IP_address, port, waitUntilConnected, doNotReconnectOnceDisconnected, timeOutInMs, commThreadCycleInMs)
        if new_clientID!=-1:
            print("Connected to remote API server")
        else:
            print("Connection to remote API server failed")
            sys.exit()
        return new_clientID

    def start_simulation(self):
        #start the simulation
        errorCode = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        assert errorCode == vrep.simx_return_ok, "Simulation could not be started"
        return

    def stop_simulation(self):
        #stop the simulation
        errorCode = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        assert errorCode == vrep.simx_return_ok, "Simulation could not be stopped"
        return

    def disconnect_simulator(self):
        #disconnect from the simulator
        vrep.simxFinish(self.clientID)
        return

    def get_object_handle(self, string):
        #provides object handle for V-REP object
        errorCode, handle = vrep.simxGetObjectHandle(self.clientID, string, vrep.simx_opmode_oneshot_wait)
        assert errorCode == vrep.simx_return_ok, "Conection to " + string + "failed"
        return handle


    def get_servos_handles(self):
        #retrieve servo handles
        servos=[]
        for i in range(1,7):
            servo = self.get_object_handle("conjoint" + str(i))
            servos.append(servo)
        print("Get joint ok")
        return servos

    def get_vision_handle(self):
        #获取视觉句柄
        for i in range(1,2):
            vRGB = self.get_object_handle("vinsion")

        return vRGB

    def get_center_handle(self):
        #获取中心句柄
        center = self.get_object_handle("center")
        return center
    def on_startup(self):
        # startup routine
        self.servos = self.get_servos_handles()
        self.vRGBs = self.get_vision_handle()
        #self.turn_on()
        #start the simulation
        #self.start_simulation()
        #print("Robot ready")
        return


    #############################################################
    # locomotion helper functions
    #############################################################

    def get_servo_position(self, servoID):
        #getting position of servos
        print(str(servoID))
        assert servoID > 0 and servoID <= 2, "Commanding unexisting servo"
        errorCode, value = vrep.simxGetJointPosition(self.clientID, self.servos[servoID-1], vrep.simx_opmode_blocking)
        assert errorCode == vrep.simx_return_ok, "Failed to read servo position"
        return value

    def set_servo_position(self, servoID, angle):
        #setting position of servos
        assert servoID >0 and servoID <=6, "Commanding unexisting servo"
        errorCode = vrep.simxSetJointTargetPosition(self.clientID, self.servos[servoID-1], angle, vrep.simx_opmode_streaming)
        assert errorCode == vrep.simx_return_ok or errorCode == vrep.simx_return_novalue_flag, "Failed to set servo position"
    def turn_on(self):
        #standing up the robot to base position
        for i in range(1,10):
            self.set_servo_position(i, 0/180.0*3.141593)


    def get_robot_position(self):
        #get the position of the robot
        if self.pos_first:
            self.pos_first = False
            errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.center, -1, vrep.simx_opmode_streaming) #start streaming
        else:
            errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.center, -1, vrep.simx_opmode_buffer) #fetch new data from stream

        if errorCode == vrep.simx_return_novalue_flag:
            position = None
        else:
            assert errorCode == vrep.simx_return_ok, "Cannot get object position"
        return position



    #############################################################
    # 图像接口
    #############################################################

    def get_vision_image(self,handle):
        errorCode, resolution, imageRGB = vrep.simxGetVisionSensorImage(self.clientID, handle, 0, vrep.simx_opmode_streaming)
        time.sleep(0.1)
        errorCode,resolution,imageRGB=vrep.simxGetVisionSensorImage(self.clientID, handle, 0, vrep.simx_opmode_buffer)
        assert errorCode == vrep.simx_return_ok, "获取图像失败"
        return errorCode,resolution,imageRGB

    def set_vision_image(self,handle,img):
        vrep.simxSetVisionSensorImage(self.clientID, handle, img, 0, vrep.simx_opmode_oneshot)

    def get_connection_id(self,clientID):
        return vrep.simxGetConnectionId(clientID)

    def set_vision_rotation(self,clientID,angles):
        vrep.simxSetObjectOrientation(clientID,-1,angles)
        #vrep.simxSetObject
    def robot_stop(self,clientID):
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)

        return True