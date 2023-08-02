import serial
import time
import atexit

class Robot:
    port = serial.Serial('/dev/ttyTHS1',115200, timeout=1)
    time.sleep(1)
    active = 0

    motion_state = 0
    motion_start = 0
    motion_end = 0

    def __init__(self):

        # Initialize drivers BLDC
        self.MotorsDriver = Motors(Robot.port)
        time.sleep(2)

        # Local variable for each motor
        self.left_motor = self.MotorsDriver.getMotor(1)
        self.right_motor = self.MotorsDriver.getMotor(2)

        # Stop motors if app stopped
        atexit.register(self.DeactiveRobot)

    def motion(self, motion, speed, duration):
        STRAIGHT = 1
        TURN_LEFT = 2
        TURN_RIGHT = 3
        SPIN_LEFT = 4
        SPIN_RIGHT = 5
        
        # INIT
        if (self.motion_state == 0):
            self.motion_start = time.time()
            self.motion_state = 1
        # START
        elif (self.motion_state == 1):

            if motion == STRAIGHT:
                self.straight(speed)
            if motion == TURN_LEFT:
                self.turn_left(speed)
            if motion == TURN_RIGHT:
                self.turn_right(speed)
            if motion == SPIN_LEFT:
                self.spin_left(speed)
            if motion == SPIN_RIGHT:
                self.spin_right(speed)

            self.motion_end = time.time()
            if( (self.motion_end-self.motion_start) > (duration/1000)):
                self.motion_state = 2
                self.motion_start = time.time()
        # STOP
        elif (self.motion_state == 3):
            # Need fix - brake with adjustable power
            # self.brake_motion(self, motion, (-1)*speed, 2)
            self.stop_motors

    def straight(self, speed):
        self.left_motor.setSpeed((-1)*speed)
        self.right_motor.setSpeed((-1)*speed)
        self.MotorsDriver.run(Motors.START)
        print("Go straight")

    def turn_left(self, speed):
        self.left_motor.setSpeed(0)
        self.right_motor.setSpeed((-1)*speed)
        self.MotorsDriver.run(Motors.START)
        print("Turn left")

    def turn_right(self, speed):
        self.left_motor.setSpeed((-1)*speed)
        self.right_motor.setSpeed(0)
        self.MotorsDriver.run(Motors.START)
        print("Turn right")

    def spin_left(self, speed):
        self.left_motor.setSpeed((-1)*speed)
        self.right_motor.setSpeed(speed)
        self.MotorsDriver.run(Motors.START)
        print("Spin left")

    def spin_right(self, speed):
        self.left_motor.setSpeed(speed)
        self.right_motor.setSpeed((-1)*speed)
        self.MotorsDriver.run(Motors.START)
        print("Spin right")

    def stop_motors(self):
        self.left_motor.setSpeed(0)
        self.right_motor.setSpeed(0)
        self.MotorsDriver.run(Motors.RELEASE)

    def ActiveRobot(self):
        cmd = "O1"
        for i in range(3):
            print("Tx: " + self.sendCmdRobot(cmd))
            response = self.readline()
            print("Rx: " + str(response))
            print("Test: " + self.compareTxRx(cmd, response))
            if response != "Timeout":
                break
    
    def DeactiveRobot(self):
        cmd = "O0"
        for i in range(3):
            print("Tx: " + self.sendCmdRobot(cmd))
            response = self.readline()
            print("Rx: " + str(response))
            print("Test: " + self.compareTxRx(cmd, response))
            if response != "Timeout":
                break

    def getMotorsDriverInfo(self):
        data = ""
        print("Tx: " + self.sendCmdRobot("U0"))
        self.paramU0 = self.readline()
        print("Rx: " + self.paramU0)

        print("Tx: " + self.sendCmdRobot("G0"))
        self.paramG0 = self.readline()
        print("Rx: " + self.paramG0)
        data = f"{self.paramU0}" + f"{self.paramG0}" + "}"
        return data

    def TestUartActiveDeactiveRobot(self, tim):
        print("TestUartActiveDeactiveRobot ...")
        for i in range(2):
            self.ActiveRobot()
            time.sleep(tim)
            self.DeactiveRobot()
            time.sleep(tim)

    def Test_CMD_Robot(self, cmd, value, tim):
        print("Test_CMD_Robot ...")
        response = ""
        for i in range(2):
            # 1
            self.ActiveRobot()
            # 2
            print("Tx: " + self.sendCmdRobot(cmd + str(value)))
            response = self.readline()
            print("Rx: " + str(response))
            print("Test: " + self.compareTxRx(str(value), str(int(float(response)))))
            time.sleep(tim)
            # 3
            print("Tx: " + self.sendCmdRobot(cmd + str(0)))
            response = self.readline()
            print("Rx: " + str(response))
            print("Test: " + self.compareTxRx(str(0), str(int(float(response)))))
            time.sleep(tim)
            # 4
            self.DeactiveRobot()

    def Test_Response_Robot(self, cmd, tim):
        print("Test_Response_Robot ...")
        response = ""
        for i in range(2):
            # 1
            self.ActiveRobot()
            # 2
            print("Tx: " + self.sendCmdRobot(cmd))
            response = self.readline()
            print("Rx: " + str(response))
            time.sleep(tim)
            # 3
            print("Tx: " + self.sendCmdRobot(cmd))
            response = self.readline()
            print("Rx: " + str(response))
            time.sleep(tim)
            # 4
            self.DeactiveRobot()

    def sendCmdRobot(self, data):
        self.port.write(bytearray(data + '\n','ascii'))
        return bytearray(data,'ascii').decode("utf-8")

    def readline(self):
        message = ""
        data = ""
        while True:
            data = self.port.read()
            if data == b'':
                message = "Timeout" # 1 s
                break
            if data == b'\n':
                break
            message += str(data.decode("utf-8") )
        return message

    def compareTxRx(self, tx, rx):
        ret = ""
        if(tx == rx):
            ret = rx + " --> OK!"
        else:
            ret = " --> Fail"
        return ret

class BLDC_Motor:
    def __init__(self, num):
        self.speed = 0
        self.motornum = num

    def setSpeed(self, speed):
        if (speed < -100):
            speed = -100
        if (speed > 100):
            speed = 100
        self.speed = speed

class Motors:
    START = 1
    BRAKE = 2
    RELEASE = 3

    def __init__(self, port):
        self.paramU0 = ''
        self.paramG0 = ''
        self.throttle = 0
        self.steering = 0
        self.port = port
        self.motors = [BLDC_Motor(m) for m in range(2)]

    def run(self, command):
        if (command == Motors.START):
            self.throttle = ((self.motors[0].speed)/2) + ((self.motors[1].speed)/2)
            self.steering = ((self.motors[0].speed)/2) - ((self.motors[1].speed)/2)
            # print("SPEED_A: " + f"{self.motors[0].speed}" + " SPEED_B: " + f"{self.motors[1].speed}")
            # print("Throttle: " + f"{self.throttle}" + " Steering: " + f"{self.steering}")
            self.port.write(bytearray('T' + str(self.throttle) + '\n','ascii'))
            self.port.write(bytearray('S' + str(self.steering) + '\n','ascii'))
        if (command == Motors.RELEASE):
            self.throttle = 0
            self.steering = 0
            self.port.write(bytearray('T' + str(self.throttle) + '\n','ascii'))
            self.port.write(bytearray('S' + str(self.steering) + '\n','ascii'))
            # print("Throttle: " + f"{self.throttle}" + " Steering: " + f"{self.steering}")

    def getMotor(self, num):
        if (num < 1) or (num > 2):
            raise NameError('MotorHAT Motor must be between 1 and 2 inclusive')
        return self.motors[num-1]