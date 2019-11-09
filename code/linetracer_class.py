import RPi.GPIO as GPIO
import time

class Motor:
    sensor = [2,3,4,16,20,21]           #sensor pin number
    sen = [0 for _ in range(6)]         #zero set
    motorR = [17,18]                    #right motor pin number
    motorL = [6,12]                     #left motor pin number
    
    error1 = error2 = error3 = 0
    errorR = errorL = 0
    dirR = dirL = 0
    PWM_R = PWM_L = 0
    speed = 0
    defaultR = 60                       #default duty for right motor
    defaultL = 72                       #default duty for left motor
    
    turning = False                     #flag for turn
    
    def __init__(self,e1,e2,e3):
        self.error1 = e1                #error range set by sensor
        self.error2 = e2
        self.error3 = e3
        self.on_line = False
        self.be_stop = True
        self.dirR = GPIO.LOW            #default direction
        self.dirL = GPIO.LOW
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motorR,GPIO.OUT)    #motor set
        GPIO.setup(self.motorL,GPIO.OUT)
        GPIO.setup(self.sensor,GPIO.IN)     #sensor set
        GPIO.output(self.motorR[0],self.dirR)   #motor direction
        GPIO.output(self.motorL[0],self.dirL)
        
        self.PWM_R = GPIO.PWM(self.motorR[1],1500)  #PWM set
        self.PWM_L = GPIO.PWM(self.motorL[1],1500)

    def start(self):                    #PWM start
        self.PWM_R.start(50)
        self.PWM_L.start(50)
        self.stop(0)

    def error_update(self):             #make center when sen[0,1,4,5]
        self.sensor_get()
        if self.sen[1] :
            self.errorR = self.error1
            self.errorL = self.error2
        if self.sen[4] :
            self.errorR = self.error2
            self.errorL = self.error1
        if self.sen[0] :
            self.errorR = self.error1
            self.errorL = self.error3
        if self.sen[5] :
            self.errorR = self.error3
            self.errorL = self.error1
        if self.sen[2] or self.sen[3] or self.turning:
            self.errorR = self.error1
            self.errorL = self.error1

    def sensor_get(self):               #get sensor data
        self.sen[0] = GPIO.input(self.sensor[0])
        self.sen[1] = GPIO.input(self.sensor[1])
        self.sen[2] = GPIO.input(self.sensor[2])
        self.sen[3] = GPIO.input(self.sensor[3])
        self.sen[4] = GPIO.input(self.sensor[4])
        self.sen[5] = GPIO.input(self.sensor[5])
        print(self.sen)
        
    def direction(self,_dir):        #Set direction
        if (_dir == 'R') or (_dir == 'B') :
            self.dirR = GPIO.HIGH
        if (_dir == 'L') or (_dir == 'B') :
            self.dirL = GPIO.HIGH
        if (_dir == 'L') or (_dir == 'F') :
            self.dirR = GPIO.LOW
        if (_dir == 'R') or (_dir == 'F') :
            self.dirL = GPIO.LOW

    def go(self,_type,speed,flag_type,mode):       #go straight until detect sen[0 or 5]
        if flag_type == 'slow':
            self.motor_update(int(speed/2))
        elif flag_type == 'stop' and self.be_stop:
            self.be_stop = False
            self.stop(2)
        elif flag_type == 'another':
            self.motor_update(speed)
        else :
            self.motor_update(speed)
            
        if(_type == 'R'):
            if self.sen[5] : self.on_line = True
            if not self.sen[5] and self.on_line:
                mode += 1
                self.on_line = False
                self.be_stop = True
            
        elif(_type == 'L'):
            if self.sen[0] : self.on_line = True
            if not self.sen[0] and self.on_line:
                mode += 1
                self.on_line = False
                self.be_stop = True

        return mode
            
    def turn(self,_dir,speed,mode):      #turn right or left after finished go()
        self.stop()
        self.turning = True
        
        if (_dir == 'R'):
            print("turn right")
            self.direction('R')
            while not self.sen[4] : self.motor_update(speed)
            while not self.sen[3] : self.motor_update(speed)
            self.direction('F')
            print("done")
            
        elif (_dir == 'L'):
            print("turn left")
            self.direction('L')
            while not self.sen[1] : self.motor_update(speed)
            while not self.sen[2] : self.motor_update(speed)
            self.direction('F')
            print("done")
        self.turning = False
        self.stop()

        mode += 1

        return mode

    def stop(self,sec=0):             #stop during sec
        print("stop")
        GPIO.output(self.motorR[0],GPIO.LOW)
        self.PWM_R.ChangeDutyCycle(0)
        GPIO.output(self.motorL[0],GPIO.LOW)
        self.PWM_L.ChangeDutyCycle(0)
        time.sleep(sec)
                    
    def motor_update(self,spd):   #speed == 0~50
        self.error_update()
        self.speed = spd
        dutyR = self.defaultR + self.errorR - self.speed     #change duty by speed
        dutyL = self.defaultL + self.errorL - self.speed
            
        if not self.dirR:
            self.PWM_R.ChangeDutyCycle(100-dutyR)
        elif self.dirR:
            self.PWM_R.ChangeDutyCycle(dutyR)
               
        if not self.dirL:
            self.PWM_L.ChangeDutyCycle(100-dutyL)
        elif self.dirL:
            self.PWM_L.ChangeDutyCycle(dutyL)
                
        GPIO.output(self.motorR[0],self.dirR)
        GPIO.output(self.motorL[0],self.dirL)

##--------------------------------------------------##
## Motor.__init__(error1 , error2, error3) == error for make center
##                                            (error1(sen[2,3]), error2(sen[1,4]), error3(sen[0,5])
## Motor.go(chr detect, int speed) == go straight for detect sensor (detect = 'R' or 'L')
## Motor.turn(chr _dir, int speed) == turn left of right (_dir = 'R' or 'L')
## Motor.stop(float sec) == motor stop during sec
## speed range == 0~50

#-------------------------------------------------------------- 실행부
# try:
#     line = Motor(0,20,50)       #initial method
#     line.direction('F')         #direction set
#     line.start()                #PWM start
#
#     line.go('R',50)
#     line.turn('R',50)
#     line.go('R',50)
#     line.stop()
#
# except KeyboardInterrupt:
#     pass
#
# GPIO.cleanup()
