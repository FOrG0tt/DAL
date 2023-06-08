from flask import Flask, render_template, Response
import cv2
import pyzbar.pyzbar as pyzbar
import threading, socket, re, base64, hashlib, time, os, inspect, ctypes, select, math, numpy as np, YB_Pcb_Car, PID
import RPi.GPIO as GPIO
import tensorflow as tf
from PIL import Image, ImageDraw, ImageFont
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_utils
import HSV_Config, demjson
from aip import AipBodyAnalysis
###################################################################
#^ imports from different libraries because its easier and updated
###################################################################
APP_ID_Body = '18550528'
API_KEY_Body = 'K6PWqtiUTKYK1fYaz13O8E3i'
SECRET_KEY_Body = 'IDBUII1j6srF1XVNDX32I2WpuwBWczzK'
client_body = AipBodyAnalysis(APP_ID_Body, API_KEY_Body, SECRET_KEY_Body)
LED1 = 40
LED2 = 38
Buzzer = 32
EchoPin = 18
TrigPin = 16
AvoidSensorLeft = 21
AvoidSensorRight = 19
Avoid_ON = 22
Tracking_Right1 = 11
Tracking_Right2 = 7
Tracking_Left1 = 13
Tracking_Left2 = 15
SSID = ''
PASSWD = ''
meanshift_X = 140
meanshift_Y = 100
meanshift_width = 40
meanshift_high = 40
meanshift_update_flag = 0
prev_left = 0
prev_right = 0
LED1_state = False
g_init = False
leftrightpulse = 1400
updownpulse = 1400
###################################################################
#^Different IDs imported from GPIO Library
###################################################################
color_lower = np.array([156, 43, 46])
color_upper = np.array([180, 255, 255])
color_hsv = {
 'red': ((0, 70, 72), (7, 255, 255)), 
 'green': ((54, 109, 78), (77, 255, 255)), 
 'blue': ((92, 100, 62), (121, 251, 255)), 
 'yellow': ((26, 100, 91), (32, 255, 255))}
car_speed = 100
qrcode_data = '0'
gesture_date = '0'
g_mode = '0'
g_servormode = '0'
g_motormode = 'car_stop'
g_detdect_mode = '0'
g_target_mode = '0'
g_tag_select = '0'
g_tag_identify_switch = 'close'
g_presentation_mode = '0'
g_track_mode = '0'
g_detect_control_mode = '0'
g_drive_view_switch = 0
g_auto_drive_switch = 'close'
g_connect_wifi_switch = 'close'
###################################################################
#^different ID's imported from CV2
###################################################################
@app.route('/')
def index():
    print('index_start')
    return render_template('index.html')
@app.route('/video_feed')
def video_feed():
    print('video_feed_start')
    return Response((mode_handle()), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/init')
def init():
    global g_init
    global tid
    if g_init == False:
        tid = threading.Thread(target=start_tcp_server, args=(6000, ))
        tid.start()
    print('init websocket!!!!!!!!!')
    return render_template('init.html')
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    else:
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError('invalid thread id')
        else:
            if res != 1:
                ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
def stop_thread(thread):
    print('Turned off the Thread')
    _async_raise(thread.ident, SystemExit)
def camUpFunction(num):
    global updownpulse
    updownpulse -= num
    angle2 = int((updownpulse - 500) / 10)
    if angle2 < 20:
        angle2 = 20
    car.Ctrl_Servo(2, angle2)
def camDownFunction(num):
    global updownpulse
    updownpulse += num
    angle2 = int((updownpulse - 500) / 10)
    if angle2 > 170:
        angle2 = 170
    car.Ctrl_Servo(2, angle2)
def camLeftFunction(num):
    global leftrightpulse
    leftrightpulse += num
    angle1 = int((leftrightpulse - 500) / 10)
    if angle1 > 170:
        angle1 = 170
    car.Ctrl_Servo(1, angle1)
def camRightFunction(num):
    global leftrightpulse
    leftrightpulse -= num
    angle1 = int((leftrightpulse - 500) / 10)
    if angle1 < 20:
        angle1 = 20
    car.Ctrl_Servo(1, angle1)
def camservoInitFunction():
    global leftrightpulse
    global updownpulse
    leftrightpulse = 1400
    updownpulse = 1400
    angle1 = int((leftrightpulse - 500) / 10)
    angle2 = int((updownpulse - 500) / 10)
    car.Ctrl_Servo(1, angle1)
    car.Ctrl_Servo(2, angle2)
###################################################################
#^Camera Controls
###################################################################
def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])
def cv2ImgAddText(img, text, left, top, textColor=(0, 255, 0), textSize=20):
    if isinstance(img, np.ndarray):
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    fontStyle = ImageFont.truetype('/home/pi/Yahboom_project/Raspbot/raspbot/simhei.ttf',
      textSize, encoding='utf-8')
    draw.text((left, top), text, textColor, font=fontStyle)
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
###################################################################
#^CV2 codes reused
###################################################################
def send_msg(conn, msg_bytes):
    import struct
    token = b'\x81'
    length = len(msg_bytes)
    if length < 126:
        token += struct.pack('B', length)
    else:
        if length <= 65535:
            token += struct.pack('!BH', 126, length)
        else:
            token += struct.pack('!BQ', 127, length)
    msg = token + msg_bytes
    conn.send(msg)
    return True
def getip():
    try:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('www.google.com', 0))
            ip = s.getsockname()[0]
        except:
            ip = 'x.x.x.x'
    finally:
        s.close()
    return ip
def start_tcp_server(port):
    global g_init
    global g_socket
    try:
        try:
            g_init = True
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            ip = getip()
            print('检测IP')
            while ip == 'x.x.x.x':
                time.sleep(5)
                ip = getip()
            print(ip)
            sock.bind((ip, port))
            sock.listen(5)
            r_list = [sock]
            while 1:
                rl, wl, error = select.selectr_list[][]1
                for fd in rl:
                    if fd == sock:
                        print('等待sock连接')
                        conn, address = sock.accept()
                        g_socket = conn
                        request = conn.recv(2048)
                        print(request.decode())
                        ret = re.search('Sec-WebSocket-Key: (.*==)', str(request.decode()))
                        if ret:
                            key = ret.group(1)
                        else:
                            print('retu')
                            return
                            Sec_WebSocket_Key = key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
                            response_key = base64.b64encode(hashlib.sha1(bytes(Sec_WebSocket_Key, encoding='utf8')).digest())
                            response_key_str = str(response_key)
                            response_key_str = response_key_str[2:30]
                            response = HANDSHAKE_STRING.replace('{1}', response_key_str).replace('{2}', ip + ':' + str(port))
                            conn.send(response.encode())
                            print('手机已连接')
                            handleTid = threading.Thread(target=message_handle, args=[conn])
                            handleTid.setDaemon(True)
                            handleTid.start()
        except:
            print('退出start_tcp_server循环')
    finally:
        print('finally')
        g_init = False
        sock.close()
def waitClose(sock):
    time.sleep(10)
    print('sock close')
    sock.close()
def message_handle(client):
    lastCmd = ''
    while True:
        try:
            info = client.recv(8096)
        except Exception as e:
            try:
                info = None
            finally:
                e = None
                del e
        if not info:
            print('break thread')
            break
        else:
            payload_len = info[1] & 127
            print(payload_len)
            if payload_len == 126:
                extend_payload_len = info[2:4]
                mask = info[4:8]
                decoded = info[8:]
            else:
                if payload_len == 127:
                    extend_payload_len = info[2:10]
                    mask = info[10:14]
                    decoded = info[14:]
                else:
                    extend_payload_len = None
                    mask = info[2:6]
                    decoded = info[6:payload_len + 6]
        bytes_list = bytearray()
        for i in range(len(decoded)):
            chunk = decoded[i] ^ mask[i % 4]
            bytes_list.append(chunk)
        try:
            body = str(bytes_list, encoding='utf-8')
        except UnicodeDecodeError:
            body = '$01,X0.00Y0.00#'
            print(bytes_list)
            print('UnicodeDecodeError')
        gotdata = body
        print(body)
        dispatch(client, body)
        time.sleep(0.005)
def getwlanip():
    ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0:ip.find('\n')]
    if ip == '':
        print('no connect any!')
    return ip
###################################################################
def decodeDisplay(image):
    global PASSWD
    global SSID
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        x, y, w, h = barcode.rect
        cv2.rectangleimage(x, y)(x + w, y + h)(225, 225, 225)2
        barcodeData = barcode.data.decode('utf-8')
        barcodeType = barcode.type
        text = '{} ({})'.format(barcodeData, barcodeType)
        cv2.putTextimagetext(x, y - 10)cv2.FONT_HERSHEY_SIMPLEX0.5(0, 0, 0)2
        a = barcodeData.find('SSID')
        b = barcodeData.find('|')
        SSID = barcodeData[6:b - 1]
        PASSWD = barcodeData[b + 2:-1]
        print(SSID)
        print(PASSWD)
        print('[INFO] Found {} barcode: {}'.format(barcodeType, barcodeData))
    return image
###################################################################
#^pyzbar qr reader and decoder code
###################################################################
def detect():
    global LED1_state
    global PASSWD
    global SSID
    SSID = ''
    PASSWD = ''
    camera = cv2.VideoCapture(0)
    ret, frame = camera.read()
    while 1:
        GPIO.output(LED2, GPIO.LOW)
        time.sleep(0.02)
        GPIO.output(LED2, GPIO.HIGH)
        time.sleep(0.02)
        ret, frame = camera.read()
        if ret == False:
            LED1_state = bool(1 - LED1_state)
            time.sleep(0.05)
            GPIO.output(LED1, LED1_state)
            print('read error')
            continue
    camera.release()
###################################################################
#^GPIO code that is used to manipulate the LEDs in the expansion board
###################################################################
def Distance_test():
    num = 0
    ultrasonic = []
    while num < 5:
        distance = Distance()
        while int(distance) == -1:
            distance = Distance()
        while int(distance) >= 500 or int(distance) == 0:
            distance = Distance()
        ultrasonic.append(distance)
        num = num + 1
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3
    return distance
def ultrasonic_reply():
    distance = Distance_test()
    send_msg(g_socket, temp.encode('utf-8'))
def u_avoid():
    distance = Distance_test()
    if distance < 30:
        car.Car_Stop()
        time.sleep(0.1)
        car.Car_Spin_Right(100, 100)
        time.sleep(0.5)
    else:
        car.Car_Run(100, 100)
def IR_reply():
    LeftSensorValue = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    send_msg(g_socket, temp.encode('utf-8'))
def ui_avoid():
    distance = Distance_test()
    LeftSensorValue = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    if distance < 25 and LeftSensorValue == False and RightSensorValue == False:
        car.Car_Stop()
        time.sleep(0.1)
        car.Car_Spin_Right(100, 100)
        time.sleep(1)
    else:
        if distance < 25 and LeftSensorValue == True and RightSensorValue == False:
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Spin_Left(80, 80)
            time.sleep(1)
            if LeftSensorValue == False:
                if RightSensorValue == True:
                    car.Car_Stop()
                    time.sleep(0.1)
                    car.Car_Spin_Right(90, 90)
                    time.sleep(2)
        else:
            if distance < 25 and LeftSensorValue == False and RightSensorValue == True:
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Spin_Right(80, 80)
                time.sleep(1)
                if LeftSensorValue == True and RightSensorValue == False:
                    car.Car_Stop()
                    time.sleep(0.1)
                    car.Car_Spin_Left(90, 90)
                    time.sleep(2)
            elif distance < 25 and LeftSensorValue == True and RightSensorValue == True:
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Spin_Right(80, 80)
                time.sleep(0.5)
            else:
                if distance >= 25 and LeftSensorValue == False and RightSensorValue == False:
                    car.Car_Stop()
                    time.sleep(0.1)
                    car.Car_Spin_Right(90, 90)
                    time.sleep(1)
                else:
                    if distance >= 25 and LeftSensorValue == False and RightSensorValue == True:
                        car.Car_Stop()
                        time.sleep(0.1)
                        car.Car_Spin_Right(80, 80)
                        time.sleep(0.5)
                    else:
                        if distance >= 25 and LeftSensorValue == True and RightSensorValue == False:
                            car.Car_Stop()
                            time.sleep(0.1)
                            car.Car_Spin_Left(80, 80)
                            time.sleep(0.5)
                        else:
                            car.Car_Run(100, 100)
def Car_Motion():
    Tracking_Left1Value = GPIO.input(Tracking_Left1)
    Tracking_Left2Value = GPIO.input(Tracking_Left2)
    Tracking_Right1Value = GPIO.input(Tracking_Right1)
    Tracking_Right2Value = GPIO.input(Tracking_Right2)
    ###################################################################
    #This code refers to the GPIO ID of the raspberry pi
    #that correlates to the servo motor
    ###################################################################
    if Tracking_Left1Value == True
    if Tracking_Left2Value == True
    if Tracking_Right1Value == False 
    if Tracking_Right2Value == False:
        car.Car_Spin_Right(70, 70)
        time.sleep(1)
  else:
    if Tracking_Left1Value == False
    if Tracking_Left2Value == False
    if Tracking_Right1Value == True
    if Tracking_Right2Value == True:
            car.Car_Spin_Left(70, 70)
            time.sleep(1)
  else:
    if Tracking_Left1Value == False  
    if Tracking_Left2Value == True 
    if Tracking_Right1Value == True
    if Tracking_Right2Value == False:
            car.Car_Run(50, 50)
            time.sleep(1)
  else:
    if Tracking_Left1Value == True
    if Tracking_Left2Value == False
    if Tracking_Right1Value == False
    if Tracking_Right2Value == True:
            car.Car_Run(50, 50)
            time.sleep(1)
  else:
    if Tracking_Left1Value == False
    if Tracking_Left2Value == True
    if Tracking_Right1Value == True
    if Tracking_Right2Value == True:
            car.Car_Spin_Left(50, 50)
            time.sleep(1)
  else:
    if Tracking_Left1Value == True 
    if Tracking_Left1Value == True 
    if Tracking_Right1Value == True
    if Tracking_Right2Value == False:
            car.Car_Spin_Right(50, 50)
            time.sleep(1)
  else:
    if Tracking_Left1Value == True 
    if Tracking_Left2Value == True 
    if Tracking_Right1Value == True 
    if Tracking_Right2Value == True:
              car.Car_Run(70, 70)
              time.sleep(1)

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(LED2, (GPIO.OUT), initial=(GPIO.HIGH))
    GPIO.setup(LED1, (GPIO.OUT), initial=(GPIO.LOW))
    GPIO.setup(Buzzer, GPIO.OUT)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    p = GPIO.PWM(Buzzer, 440)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    GPIO.setup(Avoid_ON, GPIO.OUT)
    GPIO.setup(Tracking_Left1, GPIO.IN)
    GPIO.setup(Tracking_Left2, GPIO.IN)
    GPIO.setup(Tracking_Right1, GPIO.IN)
    GPIO.setup(Tracking_Right2, GPIO.IN)
    camservoInitFunction()
    state_refresh_id = threading.Thread(target=state_reflash)
    state_refresh_id.setDaemon(True)
    state_refresh_id.start()
    app.run(host='0.0.0.0', port=6001, debug=False)