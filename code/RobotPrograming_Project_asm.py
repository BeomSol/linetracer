import serial
import threading
import tkinter
import cv2
# import PIL.Image, PIL.ImageTk
import PIL.Image
from PIL import ImageTk
import time
import os
import math
import numpy as np
import tkinter.ttk
from tkinter import messagebox
import queue
#import ORB_match_cam_class_modify as OB
from ORB_match_cam_class_modify import *
from linetracer_class import *

Go_flag =0
Stop_flag =0;

class App(tkinter.Tk):
    def __init__(self,video_source=0):
        tkinter.Tk.__init__(self)
        self.vid = MyVideoCapture(0)

        # Create a canvas that can fit the above video source size
        self.canvas = tkinter.Canvas(self, width = self.vid.width, height = self.vid.height)
        self.canvas.pack()

        self.btn_snapshot=tkinter.Button(self, text="Go", width=50, command=self.Go)
        self.btn_snapshot.pack(anchor=tkinter.CENTER, expand=True)

        self.btn_stop = tkinter.Button(self, text="Stop", width=50, command=self.Stop)
        self.btn_stop.pack(anchor=tkinter.CENTER, expand=True)


        # After it is called once, the update method will be automatically called every delay milliseconds
        self.delay = 15
        self.update()

        self.beginThread()
        self.mainloop()

    def Go(self):
        global Go_flag
        if Go_flag==1:
            Go_flag=0
        else:
            Go_flag=1

        print("Hi")
    def Stop(self):
        global Stop_flag
        if Stop_flag==0:
            Stop_flag=1
        else:
            Stop_flag=0

    def update(self):
        global what_match
        # Get a frame from the video source
        ret, frame = self.vid.get_frame()
        what_match = orb.matching(frame, 2, False)  # matching(Camera frame, down_scale, draw rect)

        orb.spend_time()
        if ret:
            self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(frame))
            self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW)

        self.after(self.delay, self.update)

    def beginThread(self):
        self.queue = queue.Queue()  # MK: queue를 생성함.
        threadTask(self.queue).start()  # MK: 생성된 queue를 threadTask에 전달함.다른 thread에서 threadTask 연산을 시작함.
        self.after(100, self.processQueue)  # MK: 100 milliseconds 후에 self.processQueue 함수를 실행함

    def processQueue(self):
        try:
            msg = self.queue.get(0)  # MK: threadTask가 보내는 메시지를 받음
            print(msg)
        except queue.Empty:  # MK: 만약 message가 없으면 expect를 통하여 해당 함수 연산을 다시 수행함
            self.after(100, self.processQueue)


class MyVideoCapture:
    def __init__(self, video_source=0):
        # Open the video source
        self.vid = cv2.VideoCapture(video_source)
        if not self.vid.isOpened():
            raise ValueError("Unable to open video source", video_source)

        # Get video source width and height
        self.width = self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def get_frame(self):
        if self.vid.isOpened():
            ret, frame = self.vid.read()
            if ret:
                # Return a boolean success flag and the current frame converted to BGR
                return (ret, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            else:
               return (ret, None)
        else:
            return (ret, None)

    # Release the video source when the object is destroyed
    def __del__(self):
        if self.vid.isOpened():
            self.vid.release()


class threadTask(threading.Thread):
    i=0

    def __init__(self, queue):
        threading.Thread.__init__(self); #MK:새로운 thread를 생성하는 단계로 판단됨. threadTask가 기존 threading.Thread class를 override하는 것으로 판단됨.
        self.queue = queue;              #MK: mainThread에서 보낸 queue를 threadTask class의 queue와 연결함
        self.line = Motor(0,20,50)       #initial method
        self.line.direction('F')         #direction set
        self.line.start()                #PWM start
        self.stage = 0

    def run(self): #MK: 기존 threading class의 run함수 override함
        global what_match
        while True:
            time.sleep(0.05) #MK: 5초 동안 thread가 아무 일을 하지 않음
            # self.queue.put("Task Finished") #MK: 해당 메시지를 queue에 추가함
            # self.queue.put(str(self.i))
            # line.go(direction, speed, img type, mode)
            if Go_flag==1:
                print(self.stage,what_match)
                try:
                    if self.stage == 0:
                        self.stage = self.line.go('R',50,what_match[0],self.stage)
                    elif self.stage == 1:
                        self.stage = self.line.stop(0)
                    self.stage = 0

                except KeyboardInterrupt:
                    self.line.stop(0)
                    GPIO.cleanup()
                    pass
                
            if Stop_flag==1:
                self.line.stop(0)

# Create a window and pass it to the Application object
watch_match = []
orb = ORB(300,7,'G',(5,5))
obj2 = cv2.imread('stop.jpg')
orb.add_object('stop',obj2)
App()
GPIO.cleanup()
