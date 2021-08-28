# 主函数
import _thread as thread

import numpy

import controlRobot as rob
import pygame
import sys
import time
import os
import win32api
import vrep
from PIL import Image as I
import sim
import random

def PlaceBall():
    # Find coordincate for sphere
    ret, targetObj = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
    ret2, arr = sim.simxGetObjectPosition(clientID, targetObj, -1, sim.simx_opmode_blocking)

    if ret2 == sim.simx_return_ok:
        print(arr)

    # place the ball into random place on the plate
    x_ori = 0.4198
    y_ori = 0.0944
    arr[0] = x_ori + random.uniform(-0.425, 0.425)
    arr[1] = y_ori + random.uniform(-0.425, 0.425)
    arr[2] = 1.2467
    sim.simxSetObjectPosition(clientID, targetObj, -1, (arr[0], arr[1], arr[2]), sim.simx_opmode_blocking)
    return


# 键盘输入检测函数
def keyScan():
    global R
    global H
    global stop
    # 初始化pygame
    pygame.init()
    size = width, height = 600, 400  # 设置窗口大小，size实际为元组
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("KeyBoard Input")  # 窗口标题
    bg = (0, 0, 0)  # 背景填充颜色
    font = pygame.font.Font(None, 20)  # 字体None使用系统默认字体，大小为20
    line_height = font.get_linesize()  # 获取字体文本的行高
    position = 0
    screen.fill(bg)  # 填充背景色


    while True:
        for event in pygame.event.get():  # 从队列中获取事件
            if event.type == pygame.QUIT:  # 判断是否点击退出动作
                sys.exit()
            elif event.type == pygame.KEYUP:
                key=pygame.key.name(event.key)
                if key == 'w':
                    #print('set front speed 10')
                    R = R+0.5  #0.5-5.0
                    screen.blit(font.render(str('set front pos 0.5'), True, (0, 255, 0)), (0, position))  # 在面板上绘制事件文本，绿色
                elif key == 's':
                    R = R-0.5  #0.5-5.0
                    #print('set back speed 10')
                    screen.blit(font.render(str('set back pos 0.5'), True, (0, 255, 0)), (0, position))  # 在面板上绘制事件文本，绿色
                elif key == 'a':
                    H = H+0.5  #0.5-5.0
                    #print('set left speed 10')
                    screen.blit(font.render(str('set left pos 1.0'), True, (0, 255, 0)), (0, position))  # 在面板上绘制事件文本，绿色
                elif key == 'd':
                    H = H-0.5  #0.5-5.0
                    #print('set right speed 10')
                    screen.blit(font.render(str('set right pos 1.0'), True, (0, 255, 0)), (0, position))  # 在面板上绘制事件文本，绿色
                elif key == 'q':
                    stop = robot.robot_stop(robot.clientID)
                    end = sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
                    sys.exit(0)

                elif key == 'p':
                    PlaceBall()
                    screen.blit(font.render(str('place the ball'), True, (0, 255, 0)), (0, position))  # 在面板上绘制事件文本，绿色






            position += line_height  # 切换到下一行显示

            if position > height:  # 下一页显示
                position = 0
                screen.fill(bg)

        pygame.display.flip()  # 更新整个面板显示在屏幕上
        time.sleep(0.1)
# 移动小车控制函数
def conArm():
    global R
    global H
    ang2rad=1/180*3.1415926
    # set max and min position
    if R>100:
        R=100
    elif R<-100:
        R=-100
    if H>180:
        H=180
    elif H<-180:
        H=-180
    while True:
        if R>=0: #前后
            #robot.set_servo_position(4, R*ang2rad)
            robot.set_servo_position(6, (R+180) * ang2rad)
        else:
            #robot.set_servo_position(4, R*ang2rad)
            robot.set_servo_position(6, (R+180) * ang2rad)
        if H>0: #左右旋转
            #robot.set_servo_position(1, H*ang2rad)
            robot.set_servo_position(4, -H * ang2rad)
        elif H<0:
            #robot.set_servo_position(1, H*ang2rad)
            robot.set_servo_position(4, -H * ang2rad)


        time.sleep(0.1)
    return
def vinsion():
    i=402


    while True:
        error, resolution, imageRGB = robot.get_vision_image(robot.vRGBs)
        # save left eye picture
        image_byte_array = numpy.array(imageRGB, dtype=numpy.uint8)
        image_buffer = I.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        img2 = numpy.asarray(image_buffer)
        save_img=image_buffer.transpose(I.ROTATE_180) #图片旋转180°
        



        i=i+1 #图片编号




        path = "./camera/picture" + str(i) + ".png"
        save_img.save(path)
        print("Picture Saved, ID:",i)
        #time.sleep(0.01)
    thread.exit_thread()
    return







if __name__ == "__main__":

    global clientID
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
    else:
        print('Failed connecting to remote API server')
    end = sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    time.sleep(0.5)  # reaction time
    start = sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)








    #res = win32api.ShellExecute(0,'open','C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/coppeliaSim.exe','-s50000 -q C:/Users/Administrator/FYP/TestEnv/Steady001.ttt','',1)
    #res = win32api.ShellExecute(0,'open','C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/coppeliaSim.exe','-h -s50000 -q C:/Users/Administrator/FYP/TestEnv/Steady001.ttt','',1)




    # -h open VREP wit GUI

    #os.system('"C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/coppeliaSim.exe" -s50000 -q C:/Users/Administrator/FYP/TestEnv/Steady001.ttt')

    #simxGetObjectPosition
    #simxSetObjectPosition


    #####
    #simxSetObjectPosition
    #####
    time.sleep(5)
    global i
    # instantiate robot

    ##########
   #coppeliaSim.exe -s50000 -q C:\Users\Administrator\FYP\TestEnv\Steady001.ttt

    ###########


    robot = rob.Robot(clientID)

    start = robot.get_connection_id(robot.clientID)


    # define global variables used for the robot control
    Lock = thread.allocate_lock()  # mutex for access to turn commands

    global R
    global H
    #initial  R H
    R=0
    H=0
    global stop
    stop = False





    # join walking thread - comply to the hierarchical architecture where robot cintrol is on the bottom level
    try:
        keyScan_thread = thread.start_new_thread(keyScan, ())
    except:
        print("Error: unable to start new thread")
    # # time.sleep(10)
    try:
        Arm_thread = thread.start_new_thread(conArm, ())

    except:
        print("Error: unable to start new thread")
    try:
        vinsion_thread = thread.start_new_thread(vinsion, ())
    except:
        print("Error: unable to start new thread")

    while(stop!=True):
        time.sleep(0.1)

    #stop = True


