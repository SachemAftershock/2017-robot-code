#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import cv2
import socket
import time
import numpy as np
import signal
import atexit
from networktables import NetworkTables
from enum import Enum

# Only keep the threads running barring a halt
isOn = False
lock = threading.Lock()
class Main:
    """
    Executive class for controlling flow
    """
    SOCK_IP = '10.2.63.57'  # Static IP for Driver's Station on FMS
    SOCK_PORT = 5810  # TCIP Port to use for communication with Driver's Station
    NT_IP = 'roboRIO-263-FRC.local'  # RIO server IP for NetworkTables

    def __init__(self):
        #register exit methods
        atexit.register(Sender.sender_exit)
        atexit.register(ImageRetriever.image_exit)
        """initializes all threads and class instances
        """
        self.image_retrieval = ImageRetriever()
        self.image_processor = FindTape()
        self.image_sender = Sender(self.SOCK_IP, self.SOCK_PORT, self.NT_IP)
        #Create continual threads to run
        self.processor_t = threading.Thread(name='image_processor', target=self.image_processor.update, args=(self.image_retrieval,self.image_sender))
        self.sender_t = threading.Thread(name='image_sender', target=self.image_sender.update_camera_feed, args=(self.image_retrieval, ))

    def start(self):
        """responsible for starting threads that will continue until bot is in off state
        """
        self.processor_t.start()
        self.sender_t.start()
        #self.toggle_t.start()


class FindTape:

    """
    An OpenCV pipeline generated by GRIP and modified by FIRST Team 263.
    """
    def __init__(self):
        self.min_hsv  = np.array([0, 0, 230])
        self.max_hsv  = np.array([80, 80, 255])
        self.min_area = 1000

    def update(self, retriever, nt_manager):
        global isOn
        while isOn:
            peg = None
            try:
                peg = self.process(retriever.get_latest_gear())
            except Exception as e:
                print(str(e))
            nt_manager.update_gear_coords(peg)

    def process(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.min_hsv, self.max_hsv)
        tape = cv2.bitwise_and(frame, frame, mask = mask)

        gray = cv2.cvtColor(tape, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(gray, 127, 255, 0)

        x, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            if True: #TODO: Add minimum area condition here
                M_area = 0
                M2_area = 0
                if (cv2.contourArea(contours[0]) > cv2.contourArea(contours[1])):
                    M  = contours[0]
                    M_area = cv2.contourArea(contours[0])
                    M2 = contours[1]
                    M2_area = cv2.contourArea(contours[1])
                else:
                    M  = contours[1]
                    M_area = cv2.contourArea(contours[1])
                    M2 = contours[0]
                    M2_area = cv2.contourArea(contours[0])
                for i in range(2, len(contours)):
                    if cv2.contourArea(contours[i]) > M_area:
                        M2 = M
                        M = contours[i]
                        M2_area = M_area
                        M_area = cv2.contourArea(M)
                    elif cv2.contourArea(contours[i]) > M2_area:
                        M2 = contours[i]
                        M2_area = cv2.contourArea(M2)
                M         = cv2.moments(M)
                M2        = cv2.moments(M2)
                centroid  = (int(M['m10']) // int(max(M['m00'], 1)), int(M['m01']) // int(max(M['m00'], 1)))
                centroid2 = (int(M2['m10']) // int(max(M2['m00'], 1)), int(M2['m01']) // int(max(M2['m00'], 1)))
                peg       = np.mean([centroid, centroid2], axis = 0)
                print("ASFHNASOUFHSA" + str(peg))
                cv2.circle(tape,(int(peg[0]), int(peg[1])),15,(0,255,255),thickness = -1)
                cv2.putText(tape,'Peg',(int(peg[0])-12,int(peg[1])+5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
                cv2.circle(tape,centroid,5,(0,255,0),thickness = -1)
                cv2.circle(tape,centroid2,5,(0,255,0),thickness = -1)
                return peg

class Sender:
    """
    Class responsible for all data sent from Raspberry Pi to both NT server and DS server
    """
    NetworkTables.initialize(server='roboRIO-263-FRC.local')
    gear = NetworkTables.getTable('cameraData/gear')
    shooter = NetworkTables.getTable('cameraData/shooter')
    camera_data = NetworkTables.getTable('cameraData/clientMode')
    FRAMERATE_PERIOD = 1 / 15.0 #its 0 unless 15 is 15.0
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    toggle = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def __init__(self,serverIP, serverPort, networktableIP):
        self.mode_gear = True

        while True:
            try:
                self.sock.connect((serverIP, serverPort))
                break
            except Exception as e:
                print('Failed to connect: ' + str(e))
        print('Connected')

    def update_camera_feed(self, retriever):
        global isOn
        while isOn:   
            self.mode_gear = self.camera_data.getBoolean('gearMode', defaultValue=True)
            if self.mode_gear:
                frame = retriever.get_latest_gear()
            else:   
                frame = retriever.get_latest_shooter()
            
            if frame is None: return
            im = cv2.imencode('.jpg', frame)[1].tostring()
            self.sock.send(im)
            time.sleep(self.FRAMERATE_PERIOD)
        
    @classmethod
    def update_gear_coords(self, peg):
        print('updating gear coords')
        if peg is None:
           print('sending -1s')
           self.gear.putNumber('x', 0)
           self.gear.putNumber('y', 0)
        else: 
            self.gear.putNumber('x', peg[0] / cv2.CAP_PROP_FRAME_WIDTH)
            self.gear.putNumber('y', peg[1] / cv2.CAP_PROP_FRAME_HEIGHT)

    @classmethod
    def update_shooter_coords(self, pointOne, pointTwo):
        print('updating shooter coords')
        (p1x,p1y,p1w,p1h) = cv2.boundingRect(pointOne)
        p1x += p1w/2.0
        p1y += p1h/2.0
        (p2x,p2y,p2w,p2h) = cv2.boundingRect(pointTwo)
        p2x += p2w/2.0
        p2y += p2h/2.0
        self.shooter.putNumber('pointOneX', p1x)
        self.shooter.putNumber('pointOneY', p1y)
        self.shooter.putNumber('pointTwoX', p2x)
        self.shooter.putNumber('pointTwoY', p2y)
        print('updated shooter coords')

    @classmethod
    def sender_exit(self):
       self.sock.close()
       print('sender closed')

    @classmethod
    def is_match_over(self):
        return NetworkTables.getTable('cameraData').getBoolean("end", False)


class ImageRetriever:
    gearCam = cv2.VideoCapture(-1)
    shooterCam = cv2.VideoCapture(-2)
    def __init__(self):
        for x in range(10):
            time.sleep(1)
        self.gearCam.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        self.gearCam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.gearCam.set(cv2.CAP_PROP_FPS, 15)
        self.gearCam.set(cv2.CAP_PROP_EXPOSURE, 20)

        self.shooterCam.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        self.shooterCam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.shooterCam.set(cv2.CAP_PROP_FPS, 15)
        self.shooterCam.set(cv2.CAP_PROP_EXPOSURE, 20)

    @classmethod
    def get_latest_gear(self):
        lock.acquire()
        frame = self.gearCam.read()[1]
        time.sleep(0.05)
        print('got latest gear: ' + str(len(frame)))
        lock.release()
        return frame
        
    @classmethod
    def get_latest_shooter(self):
        lock.acquire()
        frame = self.gearCam.read()[1]
        time.sleep(0.05)
        print('got latest shooter: ' + str(len(frame)))
        lock.release()
        return frame
    
    @classmethod
    def image_exit(self):
        self.shooterCam.release()
        self.gearCam.release()

if __name__ == '__main__':
    global isOn
    isOn = True
    executive = Main()
    executive.start()
