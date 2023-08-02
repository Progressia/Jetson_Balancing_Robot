import robot
import time

import os
import json
from flask import Flask,request,jsonify
import threading

# ##################################################
data_string = ''
f = {}
f['O'] = '0'
f['T'] = '0'
f['S'] = '0'
f['M'] = '0'
f['V'] = '50'
f['D'] = '500'
f['X'] = '0'

up_comm = 0

def while_function():

    global data_string
    global up_comm
    i=0

    r = robot.Robot()
    time.sleep(2)

    r.ActiveRobot()
    time.sleep(2)
    print(r.getMotorsDriverInfo())
    time.sleep(2)
    r.DeactiveRobot()

    while True:
        
        # #######################################
        if(up_comm != 0):
            if(int(f['O']) == 1):
                r.active = 1
                r.ActiveRobot()
                print("Robot activated")
            if(int(f['O']) == 0):
                r.active = 0
                r.DeactiveRobot()
                print("Robot deactivated")
            up_comm = 0
        #######################################
        if (r.active == 1):
            if( (int(f['T']) != 0) or (int(f['S']) != 0) ):
            # Manual motion
                if( int(f['M']) == 0 ):
                    TLimit = int(f['T'])
                    SLimit = int(f['S'])
                    r.port.write(bytearray('T' + str(int(TLimit)) + '\n','ascii'))
                    r.port.write(bytearray('S' + str(int(SLimit)) + '\n','ascii'))
                # Automatic motions
                if( int(f['M']) > 0 ):
                    if(r.motion_state != 2):
                        r.motion(r, int(f['M']), int(f['V']), int(f['D']))
            else:
                r.stop_motors
                r.motion_state = 0
        # #######################################
        # Update parameters from Motors Driver
        if(i >= 20):
            data_string = r.getMotorsDriverInfo()
            i=0
        i += 1
        
        # Loop delay
        time.sleep(50/1000)
    # #######################################

# ###########################################
app = Flask(__name__)

# ###########################################
# API
@app.route('/api',methods=['GET', 'POST'])
def query_records():
    global f
    global up_comm
    
    if request.method == 'GET':
        if(request.args.get('Query')):
            d = {}
            d['Query'] = str(data_string)
            return jsonify(d)
    if request.method == 'POST':
        up_comm = 1
        # Active/Deactive
        if(request.args.get('O')):
            f['O'] = str(request.args.get('O'))
            print('O=' + f['O'])
            return jsonify(f)
        # Torque
        if(request.args.get('T')):
            f['T'] = str(request.args.get('T'))
            print('T=' + f['T'])
            return jsonify(f)
        # Steering
        if(request.args.get('S')):
            f['S'] = str(request.args.get('S'))
            print('S=' + f['S'])
            return jsonify(f)
        # Update
        if(request.args.get('U')):
            f['U'] = str(request.args.get('U'))
            print('U=' + f['U'])
            return jsonify(f)
        # Mode
        if(request.args.get('M')):
            f['M'] = str(request.args.get('M'))
            print('M=' + f['M'])
            return jsonify(f)
        # Velocity
        if(request.args.get('V')):
            f['V'] = str(request.args.get('V'))
            print('V=' + f['V'])
            return jsonify(f)
        # Duration
        if(request.args.get('D')):
            f['D'] = str(request.args.get('D'))
            print('D=' + f['D'])
            return jsonify(f)
        # AP (PID)
        if(request.args.get('AP')):
            f['AP'] = str(request.args.get('AP'))
            print(f['AP'])
            return jsonify(f)
        # AI (PID)
        if(request.args.get('AI')):
            f['AI'] = str(request.args.get('AI'))
            print(f['AI'])
            return jsonify(f)
        # AD (PID)
        if(request.args.get('AD')):
            f['AD'] = str(request.args.get('AD'))
            print(f['AD'])
            return jsonify(f)
        # A Ramp (PID)
        if(request.args.get('AR')):
            f['AR'] = str(request.args.get('AR'))
            print(f['AR'])
            return jsonify(f)
        # A Limit (PID)
        if(request.args.get('AL')):
            f['AL'] = str(request.args.get('AL'))
            print(f['AL'])
            return jsonify(f)
        # BP (PID)
        if(request.args.get('BP')):
            f['BP'] = str(request.args.get('BP'))
            print(f['BP'])
            return jsonify(f)
        # BI (PID)
        if(request.args.get('BI')):
            f['BI'] = str(request.args.get('BI'))
            print(f['BI'])
            return jsonify(f)
        # BD (PID)
        if(request.args.get('BD')):
            f['BD'] = str(request.args.get('BD'))
            print(f['BD'])
            return jsonify(f)
        # B Ramp (PID)
        if(request.args.get('BR')):
            f['BR'] = str(request.args.get('BR'))
            print(f['BR'])
            return jsonify(f)
        # B Limit (PID)
        if(request.args.get('BL')):
            f['BL'] = str(request.args.get('BL'))
            print(f['BL'])
            return jsonify(f)
        # Pitch filter
        if(request.args.get('CF')):
            f['CF'] = str(request.args.get('CF'))
            print(f['CF'])
            return jsonify(f)
        # Steering filter
        if(request.args.get('DF')):
            f['DF'] = str(request.args.get('DF'))
            print(f['DF'])
            return jsonify(f)
        # Throttle filter
        if(request.args.get('EF')):
            f['EF'] = str(request.args.get('EF'))
            print(f['EF'])
            return jsonify(f)
        # Close system
        if(request.args.get('X')):
            f['X'] = str(request.args.get('X'))
            print('X=' + f['X'])
            os.system('systemctl poweroff')
            return jsonify(f)
# ###########################################
def run_app():
    # app.run(debug=False, threaded=True)
    app.run(host='192.168.43.70', port=5000, debug=False, threaded=True)
# ###########################################

# ###########################################
if __name__ == '__main__':
    first_thread = threading.Thread(target=run_app)
    second_thread = threading.Thread(target=while_function)
    first_thread.start()
    second_thread.start()
# ###########################################