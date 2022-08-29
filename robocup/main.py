#!/usr/bin/env python3

from ev3dev2.motor import *
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.led import Leds
import sys
from ev3dev2.sound import Sound
from time import sleep, time
from ev3dev2.port import LegoPort
import pid as pidfile



csl = Sensor(address=INPUT_1, driver_name='ht-nxt-color-v2')
csl.mode = 'RGB'

csr = Sensor(address=INPUT_2, driver_name='ht-nxt-color-v2')
csr.mode = 'RGB'

# csf = Sensor(address=INPUT_3, driver_name='lego-ev3-color')
# csf.mode = 'COL-REFLECT'

csf = Sensor(address=INPUT_3, driver_name='ht-nxt-color-v2')
csf.mode = 'RGB'

mux1 = LegoPort('in4:i2c80:mux1')
mux1.set_device = 'lego-ev3-us'
usf = UltrasonicSensor(address='i2c80:mux1')

mux2 = LegoPort('in4:i2c81:mux2')
mux2.set_device = 'lego-ev3-us'
uss = UltrasonicSensor(address='i2c81:mux2')

mux3 = LegoPort('in4:i2c82:mux3')
mux3.set_device = 'lego-ev3-gyro'
gs = GyroSensor(address='i2c82:mux3')

ml = Motor(address=OUTPUT_A,driver_name='lego-ev3-l-motor')
ml.polarity = 'normal'

mr = Motor(address=OUTPUT_B,driver_name='lego-ev3-l-motor')
mr.polarity = 'normal'

mf = Motor(address=OUTPUT_C,driver_name='lego-ev3-m-motor')

mb = Motor(address=OUTPUT_D,driver_name='lego-ev3-m-motor')

pid = pidfile.PID()
gyropid = pidfile.PID()
kp, ki, kd = 0, 0, 0
gkp, gki, gkd = 0, 0, 0
speed = 20
l_black , r_black = 30,50 #30,75
l_white , r_white = 185,255 #220,255
bw_thres = 125
drive = MoveTank(OUTPUT_A,OUTPUT_B)
sound = Sound()
balls = []
modular = True

mode = 1 #1 for line, 2 for evac
detecting_rescue_kit = True
detecting_evacuation_zone = True
detecting_obstacle = True
last_black_pos = []
last_stalled_time = 0
skipping_green = False
low_p = False
on_green = False
double_black_pos = [0,0]
double_black_pos_2 = [0,0]
high_p = False
t_single_black = 0
side_black = 0
gyro_0 = 0
sort_pos = 0


def exit():
    sys.exit()
    
def log(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def isGreen(sensor):
    r,g,b = sensor.value(0),sensor.value(1),sensor.value(2)
    if r <15 and g<15 and b<15:
        return False
    elif g > 1.15*r and g > 1.8*b:
        # log('green ',r,g,b)
        return True
    else:
        # log('not green ',r,g,b)
        return False

def isBlue(sensor):
    r,g,b = sensor.value(0),sensor.value(1),sensor.value(2)
    if r<15 or g<15 or b<70:
        result = False
    elif b > 1.7*r and b > 1.2*g:
        result = True
    else:
        result = False

    return result

def ref():
    ls = csl.value(3)
    rs = csr.value(3)
    l1 = (ls - l_black)/(l_white-l_black)*100
    r1 = (rs - r_black)/(r_white-r_black)*100
    l = max(0,min(100,l1))
    r = max(0,min(100,r1))
    # log(l,r)
    return l,r

def gyro():
    return gs.value(0) - gyro_0

def playSound(name):
    sound_str = '/home/robot/sounds/' + name + '.wav'
    sound.play_file(sound_str)

def checkBall():
    r = csf.value(3)
    result = 0
    if r < 15:
        result = 1  #black
    elif r < 100:
        result = 0
    else:
        result = 2  #silver    
    log('Check ball',r,result)
    return result 

def shiftClaw(level):
    if level == 1:
        mf.on_for_seconds(80,1.5,False)
    elif level == 2:
        mf.on_for_seconds(-80,1.5,False)
    
def pickUp(color=0):
    global detecting_obstacle
    detecting_obstacle = False
    log('pickUp ',color)
    drive.stop()
    if color == 1:#black
        sort(-1)
    elif color == 2:#silver
        sort(1)
    drive.on_for_degrees(-30,-30,250,True,False)
    mf.on_for_seconds(-80,1.5,False,True)
    sleep(0.5)
    mf.on_for_seconds(80,1.5,False)
    drive.on_for_degrees(30,30,250,True,False)
    sweep()
    detecting_obstacle = True

def sort(direction):
    global sort_pos
    if sort_pos == direction:
        return
    if direction == -1:
        mb.on_for_seconds(-50,0.2,False)
        sort_pos = -1
        return
    elif direction == 1:
        mb.on_for_seconds(50,0.2,False)
        sort_pos = 1
        return
    else:
        return

def sweep():
    global sort_pos
    if sort_pos == 0:
        return
    elif sort_pos > 0:
        mb.on_for_seconds(-50,0.2,False,True)
        sort_pos = -1
    else:
        mb.on_for_seconds(50,0.2,False,True)
        sort_pos = 1

def stalledPickUp():
    log('stalledPickUp')
    drive.on_for_seconds(30,30,2)
    c = checkBall()
    log('c',c)
    # mf.on_for_seconds(15,1,True)

    drive.on_for_degrees(-30,-30,200)

    # mf.on_for_seconds(-5,2,False)
    if c == 1:
        shiftClaw(2)
        drive.on_for_seconds(-30,-30,1)
        pickUp(1)
        return
    
    elif c == 2:
        shiftClaw(2)
        drive.on_for_seconds(-30,-30,1)
        pickUp(2)
        return

    else:
        drive.on_for_degrees(-30,-30,300)

    return

def turn(direction):
    ang = 368
    if direction < 0:
        drive.on_for_degrees(-30,30,ang*abs(direction))
    elif direction > 0:
        drive.on_for_degrees(30,-30,ang*direction)
    else:
        return

def gyroTurn(deg,motor='both'):
    v = gs.value(0)
    desired_v = v + (deg)

    if deg < 0 :
        if motor == 'both':
            l, r = -50,50
        elif motor == 'r':
            l ,r = 0,50
        elif motor == 'l':
            l, r = -50,0
        while gs.value(0) - desired_v > 15:
            drive.on(l,r)
        while gs.value(0) > desired_v:
            drive.on(l/3,r/3)
        drive.stop()
        return
    elif deg > 0:
        if motor == 'both':
            l, r = 50,-50
        elif motor == 'r':
            l ,r = 0,-50
        elif motor == 'l':
            l, r = 50,0
        while desired_v - gs.value(0) > 15:
            drive.on(l,r)
        while gs.value(0) < desired_v:
            drive.on(l/3,r/3)
        drive.stop()
        return
    else:
        return

def gyroReset():
    global gyro_0
    gyro_0 = gs.value(0)

def lineTrackUnit(): 
    global last_black_pos, skipping_green, on_green, double_black_pos, speed, low_p, high_p, double_black_pos_2, t_single_black, side_black
    l, r = ref()
    # log('linetrack ',l,r)
    if l <= 90 or r <= 90:
        last_black_pos = [ml.position,mr.position]

    e =  l - r

    if on_green == True:
        e = 0

    # turn=''

    # if l <10 and r <10 and csf.value(3)>=120:
    #     log('V double black')
    #     pos = [ml.position,mr.position]
    #     dist = 0
    #     drive.on_for_degrees(-20,-20,150)
    #     while True:
    #         dist = abs((ml.position - pos[0])+(mr.position - pos[1]))/2
    #         if dist >= 400:
    #             break 
    #         ll, rr = ref()
    #         if ll <= 30:
    #             turn = 'r'
    #             log('go r')
    #             break
    #         elif rr <= 30:
    #             turn = 'l'
    #             log('go l')
    #             break
    #         drive.on(-20,-20)
    #     pos = [ml.position, mr.position]
    #     while True:
    #         if abs((ml.position - pos[0])+(mr.position - pos[1]))/2 >= dist:
    #             break
    #         drive.on(20,20)
    #         junctionDetection()
    #     if turn == 'l':
    #         drive.on_for_degrees(-20,20,100)
    #     elif turn == 'r':
    #         drive.on_for_degrees(20,-20,100)

    # if not t_single_black == 0:
    #     if time() - t_single_black <= 1:
    #         if side_black == 1:
    #             if r <= 20:
    #                 log('Left T')
    #                 drive.stop()
    #                 sleep(10)
    #                 drive.on_for_degrees(20,0,150)
                    
    #         elif side_black == 2:
    #             if l <= 20:
    #                 log('Right T')
    #                 drive.stop()
    #                 sleep(10)
    #                 drive.on_for_degrees(0,20,150)
                    
    #     else:
    #         t_single_black = 0

    # if l<= 20:
    #     t_single_black = time()
    #     side_black = 1
    
    # if r<= 20:
    #     t_single_black = time()
    #     side_black = 2

    # if l <= 30:
    #     side_black = 1
    #     log('left black')
    #     t = time()
    #     while 1:
    #         ll,rr = ref()
    #         if rr <= 30:
    #             log('double black')
    #             drive.stop()
    #             drive.on_for_degrees(20,20,30)
    #             break
    #         if time() - t >= 1:
    #             log('no double black')
    #             break
    #         drive.on(-10.5,30)

    # if r <= 30:
    #     side_black = 2
    #     log('r black')
    #     t = time()
    #     while 1:
    #         ll,rr = ref()
    #         if ll <= 30:
    #             log('double black')
    #             drive.stop()
    #             drive.on_for_degrees(20,20,30)
    #             break
    #         if time() - t >= 1:
    #             log('no double black')
    #             break
    #         drive.on(30,-10.5)

    if l <= 30 and r <= 30: #double black
        log('double black')
        if not low_p: 
            # drive.stop()
            # sound.play_tone(1500, 0.1)
            # sleep(3)
            low_p = True
            double_black_pos = [ml.position,mr.position]
        if not skipping_green:
            log('skipping green True')
            skipping_green = True

    if ((ml.position - double_black_pos[0])+(mr.position - double_black_pos[1]))/2 >= 30 and low_p:
        log('double black out')
        drive.stop()
        sleep(0.1)
        # sound.play_tone(800, 0.1)
        double_black_pos_2 = [ml.position,mr.position]
        high_p = True
        low_p = False

    if ((ml.position - double_black_pos[0])+(mr.position - double_black_pos[1]))/2 >= 100 and skipping_green:
        log('skipping green False')
        skipping_green = False

    if ((ml.position - double_black_pos_2[0])+(mr.position - double_black_pos_2[1]))/2 >= 50 :
        high_p = False

    if high_p:
        e *= 1

    if low_p:
        e *= 1
    else:
        e *= 1 #1

    # log(e)
    pid.update(e)
    o = pid.output
    speed_l = speed - o
    speed_r = speed + o

    # thres = 90 #80

    # if e >=thres: 
    #     speed_r = -speed 
    #     speed_l = -speed + ((1-((e-thres)/thres)) * (speed*2))
    # elif e >= 0:
    #     speed_r = -speed + ((1-e/thres) * (speed*2))
    #     speed_l = speed 
    # elif e >= -thres:
    #     speed_l = -speed + ((1-abs(e)/thres) * (speed*2)) 
    #     speed_r = speed 
    # else:
    #     speed_l = -speed 
    #     speed_r = -speed + ((1-(abs(e)-thres)/thres) * (speed*2))

    speed_r = max(-100,min(100,(speed_r)))
    speed_l = max(-100,min(100,(speed_l)))

    # log(speed_l,speed_r)
    drive.on(speed_l,speed_r)

def junctionDetection():
    global on_green, skipping_green
    bw_thres = 160 #low to prevent mistaking green as black
    l_green = isGreen(csl)
    r_green = isGreen(csr)

    if on_green == True: 
        if not l_green and not r_green: #no longer on green
            log('past green, skipping green false')
            on_green = False
            skipping_green = False
        return

    elif l_green or r_green:
        drive.on_for_degrees(30,30,10)
        drive.stop()
        sleep(0.1)
        l_green, r_green = isGreen(csl),isGreen(csr)

        if skipping_green == True:
            log('skipping green, on green')
            on_green = True
            return
    
    else:
        pass

    if l_green and r_green:
        log('both green')
        drive.on_for_degrees(-30,30,800)
        # while isGreen(csl) and isGreen(csr):
        #     log('driving on 10,10')
        #     drive.on(10,10)
        # if csl.value(3) <= actual_bw_thres and csr.value(3) <= actual_bw_thres:
        #     log('both on black, move 100 + turn l 800')
        #     drive.on_for_degrees(30,30,100)
        #     drive.on_for_degrees(-30,30,800)
        #     while True:
        #         drive.on(-30,30)
        #         log('turning left until right on black')
        #         if csr.value(3) <= bw_thres:
        #             log('right on black')
        #             break
        #     drive.on_for_degrees(30,-30,70)
        #     drive.on_for_degrees(30,30,150)
        # else:
        #     drive.on_for_degrees(30,30,5)
        
    elif l_green:
        log('l green')
        drive.stop()
        drive.on_for_degrees(30,30,180)
        while csl.value(3) <= bw_thres:
            drive.on(-30,30)
        while csl.value(3) >= bw_thres:
            drive.on(-30,30)
        drive.on_for_degrees(-30,30,200)
        while csr.value(3) >= bw_thres:
            drive.on(-30,30)
        drive.on_for_degrees(30,0,100)
        drive.stop()

    elif r_green:
        log('r green')
        drive.stop()
        drive.on_for_degrees(30,30,180)
        while csr.value(3) <= bw_thres:
            drive.on(30,-30)
        while csr.value(3) >= bw_thres:
            drive.on(30,-30)
        drive.on_for_degrees(30,-30,200)
        while csl.value(3) >= bw_thres:
            drive.on(30,-30)
        drive.on_for_degrees(0,30,100)
        drive.stop()

    else:
        return

def rescueKitDetection():
    global detecting_rescue_kit
    if isBlue(csf):
        drive.stop()
        pickUp()
        # detecting_rescue_kit = False
        return

def obstacleDetection():
    global detecting_evacuation_zone, last_black_pos
    fixed_value = 0
    v = usf.value(0)
    # log('obstacle detection ',v)
    if v >0 and v <= 120: #v <= 120
        log('obstacle detected')
        detecting_evacuation_zone = False
        # sound.beep()
        drive.stop()

        u = usf.value(0)
        drive.on_for_degrees(20,20,u*2)

        # drive.on_for_degrees(20,20,150)
        
        while True:
            u = uss.value(0)
            if u <= 200: #100
                break
            drive.on(20,-20)
        drive.stop()

        fixed_value = uss.value(0)

        while True:
            l,r = ref()
            if r <= 80:
                log('right black')
                drive.stop()
                break
            w = uss.value(0)
            e = w - fixed_value
            drive.on(15,max(0,min(e*5,100)))

        mr.on_for_degrees(-30,300)
        while True:
            l,r=ref()
            if r <= 80:
                drive.stop()
                break
            drive.on(20,20)
        
        while True:
            l,r=ref()
            if r >=99:
                drive.stop()
                break
            drive.on(20,-20)        

        ml.on_for_degrees(30,150)

        detecting_evacuation_zone = True
        last_black_pos = [ml.position,mr.position]
        
def evacuationZoneDetection():
    global mode
    # l,r = csl.value(0), csr.value(0)
    # if l >= 250 and r >= 250: #250
    #     log('evacuation zone detected: ',l,r)
    #     mode = 'e'
    #     sound.beep()
    len_white = (abs(ml.position - last_black_pos[0]) + abs(mr.position-last_black_pos[1]))/2
    # log('len_white ',len_white)
    if len_white > 1500:
        log('evacuation zone detected')
        mode = 2
        drive.stop()
        sound.beep()
    return

def scanBall(speed,zero):
    c = checkBall()
    if c == 1: #black
        log('scanBall black')
        pickUp(1)
    elif c == 2:
        pickUp(2)
    else:
        gyroTrackUnit(speed,zero)

def wallTurn(direction,corner=False):
    speed = 50
    if corner == True:
        gyroTurn(180,'l')

    else:
        if direction >= 0:
            gyroTurn(30,'r')
            gyroTurn(150,'l')
        else:
            gyroTurn(-30,'l')
            gyroTurn(-150,'r')
    
    drive.on_for_seconds(-50,-50,2)

def getOutOfCorner(direction): 
    #positive for right, negative for left, must start from straight position
    turn_amt = 50
    turn_amt2 = 30
    if direction < 0:
        turn_amt *= -1
        turn_amt2 *= -1
    else:
        pass
    gyroTurn(turn_amt)
    drive.on_for_seconds(30,30,3.5)
    gyroTurn(turn_amt2)

def gyroTrackUnit(speed,zero):
    g = (gs.value(0)-zero)
    # log(g)
    gyropid.update(g)
    o = gyropid.output
    ls = speed + o
    rs = speed - o
    ls = max(min(ls,100),0)
    rs = max(min(rs,100),0)
    drive.on(ls,rs)

def evacuationZone():
    global detecting_evacuation_zone, mode, gyro_0
    balls_collected = 0

    # shiftClaw(2)

    # turn left and align back against wall
    # turn(-1.6)
    # drive.on_for_seconds(-80,-80,3)
    # drive.on_for_seconds(-30,-30,2)
    # drive.stop()
    # sleep(0.3)          
    # gyro_0 = gs.value(0) + 90

    # #turn right
    # drive.on_for_degrees(50,50,100)
    # gyroTurn(90)
    # sleep(0.3)

    #scan ball and corner
    # side = 0
    # pos = [ml.position, mr.position]
    # corner_detected = False
    # corner_i = 0
    # for i in range(4): #scan sides for corner
    #     log('scanning side, i ',i)
    #     for i in range(i):
    #         sound.play_tone(1200,0.1) 
    #     while True: #scan 1 side
    #         f = usf.value(0)
    #         r = abs(gyro())%90
    #         g = min(r-0,90-r)
    #         c = csf.value(3)
    #         log(c,g,f)
    #         if c <= 30:
    #             corner_detected = True
    #             break
    #         elif g <= 20 and f <= 200:
    #             corner_detected = False
    #             break
    #         drive.on(50,50)
    #     log('corner_detected',corner_detected)

    #     if i == 0: #determine entry side length
    #         dist = (abs(ml.position - pos[0]) + abs(mr.position - pos[1]))/2
    #         side = 1 if dist <= 1500 else 2 #short for 1, long for 2
    #         log("dist", dist, "side", side)

    #     if corner_detected: 
    #         corner_i = i
    #         break

    #     else:
    #         #get out of wall corner
    #         drive.on_for_degrees(30,30,350)
    #         gyroTurn(-90)

    # l = []
    # scanning_side = 0 #determine the side to scan
    # if side == 1:
    #     l = [1,2]
    #     scanning_side = 2
    # elif side == 2:
    #     l = [2,1]
    #     scanning_side = 1
    # corner_side = l[corner_i % 2] #determine corner side length
    # log('corner_side ',corner_side)

    # do = 0
    # turn = 0
    # if corner_side == 1: #if approached corner from short side
    #     if scanning_side == 1:
    #         do = 1
    #     else:
    #         do = 2
    # else:
    #     if scanning_side == 1:
    #         do = 2
    #     else:
    #         do = 1

    # if do ==1: #turn back
    #     gyroTurn(-200)
    #     turn = 1
    # else: #get out of corner from left
    #     getOutOfCorner(-1)
    #     turn = -1
    
    #robot in position, start scanning balls

    scanning_side = 2
    turn = -1
    shiftClaw(1)
    #scan
    num = 6 if scanning_side == 2 else 7
    for i in range(num):
        pos = [ml.position,mr.position]
        if i % 2 == 0:
            turn = -1
        else:
            turn = 1
        log('scanning ',i)
        if i == 1:
            a = gs.value(0)
            while True:
                v = usf.value(0)
                d = (abs(ml.position - pos[0]) + abs(mr.position - pos[1]))/2
                log(d)
                if d > 2700 and detecting_obstacle and v < 200:
                    drive.stop()
                    log('1 wall',v)
                    break
                scanBall(50,a)
            wallTurn(1,True)

        else:
            a = gs.value(0)
            while True:
                v = usf.value(0)
                if detecting_obstacle and v <= 50:
                    drive.stop()
                    log('wall',v)
                    break
                scanBall(50,a)
            log('turn',turn)
            if i == num-1:
                pass
            elif i == num-2:
                gyroTurn(turn*180)
                drive.on_for_degrees(50,50,200)
                gyroTurn(turn*20)
                drive.on_for_seconds(-50,-50,4)
            else:
                wallTurn(turn)
            

    exit()

    # if corner_side == 1:
    #     #turn left
    #     pass
    # else:
    #     #turn right
    #     pass

    return
 
def main():
    global kp, ki, kd, speed, mode, last_black_pos, modular, gkp, gki, gkd
    #line
    speed = 30
    kp, ki, kd = 1, 0, 0.2
    pid.setKp(kp)
    pid.setKi(ki)
    pid.setKd(kd)
    #gyro
    gkp ,gki ,gkd = 3, 0.2, 0
    gyropid.setKp(gkp)
    gyropid.setKi(gki)
    gyropid.setKd(gkd)
    modular = True
    mode = 2

    #test
    a = gs.value(0)
    while True:
        scanBall(50,gs.value(0))

    #code starts here
    last_black_pos = [ml.position,mr.position]
    while True:
        if mode == 1:
            lineTrackUnit()
            junctionDetection()
            # if detecting_obstacle == True:
            #     obstacleDetection()
            if detecting_evacuation_zone == True:
                evacuationZoneDetection()
        elif mode == 2: #run once
            evacuationZone()

if __name__ == '__main__':
    main()

#hitechnic:
#nothing 1-3
#corner < 5
#silver ball > 120
#black ball > 9

