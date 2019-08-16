########### 模块引入 ##############
import sensor, image, time
from pyb import LED
from pyb import UART,Timer
from pyb import Pin

########## 变量定义 ##############

class code_msg_to_send():
    quadrotor_is_to_start = 0
    pole_is_near = 0

class code_msg_to_recieve():
    quadrotor_is_stopped = 1

class receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0


########## 基础配置 #############

send_flag = code_msg_to_send()
rec_flag = code_msg_to_recieve()
R = receive()

uart = UART(3,115200)#初始化串口 波特率 115200

sensor.reset()
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

barcode_detected = 0
qrcode_detected = 0


barcode_pixels_threshold = [(49, 100, -20, 0, 16, 59)]    #条形码识别阈值
qrcode_pixels_threshold = [600, 800]     #二维码识别阈值
near_the_pole_threshold = [1000, 1500]   #到达杆识别阈值
see_the_pole = False          #起飞后是否看见杆，通过线段拟合get_regression实现


#BINARY_THRESHOLD = (208, 224)

x_width = 80
y_height = 60

had_finished_barcode = 0
had_finished_qrcode = 0


########### 串口 ##############
# 串口数据解析
def Receive_Anl(data_buf,num):

    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1

    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过

    #if data_buf[2]==0x01:
    #    print("receive 1 ok!")

    #if data_buf[2]==0x02:
    #   print("receive 2 ok!")

    if data_buf[2]==0xFD:     #控制字

        #接收各种标志
        rec_flag.quadrotor_is_stopped = data_buf[4]
        barcode_detected = data_buf[5]
        qrcode_detected = data_buf[6]

        #print("Recieve flags success!")


# 串口通信协议接收
def Receive_Prepare(data):

    if R.state==0:

        if data == 0xAA:#帧头
            R.state = 1
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==1:
        if data == 0xAF:#帧头
            R.state = 2
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==2:
        if data <= 0xFF:#控制字
            R.state = 3
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==3:#数据个数
        if data <= 33:
            R.state = 4
            R.uart_buf.append(data)
            R._data_len = data
            R._data_cnt = 0
        else:
            R.state = 0

    elif R.state==4:
        if R._data_len > 0:
            R. _data_len = R._data_len - 1
            R.uart_buf.append(data)
            if R._data_len == 0:
                R.state = 5
        else:
            R.state = 0

    elif R.state==5:
        R.state = 0
        R.uart_buf.append(data)
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state = 0

# 读取串口缓存
def uart_read_buf():
    i = 0
    buf_size = uart.any()
    while i<buf_size:
        Receive_Prepare(uart.readchar())
        i = i + 1


# 标志位数据打包
def pack_flag_data():
    pack_data=bytearray([0xAA,0xAF,0xF0,0x00,   #修改！！！！！！！！！！！
        send_flag.quadrotor_is_to_start,send_flag.pole_is_near,
        0x00,0x00,0x00,0x00,
        0x00,0x00])

    send_flag.quadrotor_is_to_start = 0
    #send_flag.pole_is_near = 0

    print("aaaa: ", send_flag.pole_is_near)

    lens = len(pack_data)#数据包大小
    pack_data[3] = lens-5;#有效数据个数

    i = 0
    sum = 0

    #和校验
    while i<(lens-1):
        sum = sum + pack_data[i]
        i = i+1
    pack_data[lens-1] = sum;

    return pack_data

############ 像素识别 ###########

barcode_cnt = 0
qrcode_cnt = 0
pole_cnt = 0
LED_ON = 0

def count_pixels_with_movement(img):
    global x_width, y_height
    global barcode_detected, qrcode_detected
    global barcode_cnt, qrcode_cnt, pole_cnt
    global LED_ON

    blob = img.find_blobs(barcode_pixels_threshold, pixels_threshold=150, area_threshold=150, merge=True, margin=5)
    if blob:
        pass
        for i in blob:
            img.draw_rectangle(i.rect(), color = 127)
            barcode_detected = 1
            LED_ON = 1
            print("132123132131313131112311231")
            print("aaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            return
    else:
        barcode_detected = 0
        LED_ON = 0
        return
    x_pos = 0
    y_pos = 0
    total_white_pixels = 0
    for x_pos in range(x_width):
        for y_pos in range(y_height):
            if img.get_pixel(x_pos, y_pos) == 255:
                total_white_pixels += 1

    #print("total white pixels are", total_white_pixels)
#    if total_white_pixels >= verticle_pixels_threshold[0] and \
#            total_white_pixels <= verticle_pixels_threshold[1]:       #直角
#        singleline_check.is_verticle = 2

#    elif total_white_pixels >= track_line_pixels_threshold[0] and \
#            total_white_pixels <= track_line_pixels_threshold[1]:     #巡线
#        singleline_check.is_verticle = 1

    if total_white_pixels >= barcode_pixels_threshold[0] and \
            total_white_pixels <= barcode_pixels_threshold[1]:        #条形码
       if barcode_cnt >= 0 and barcode_cnt <= 49:
            barcode_cnt += 1
       if barcode_cnt == 50:
            barcode_cnt = -1
            barcode_detected = 1

    #        LED(1).toggle()  #红灯
    #        time.sleep(200)
    #        LED(1).toggle()

    #if total_white_pixels >= qrcode_pixels_threshold[0] and \
    #        total_white_pixels <= qrcode_pixels_threshold[1]:         #二维码
    #    if qrcode_cnt >= 0 and qrcode_cnt <= 49:
    #        qrcode_cnt += 1
    #    if qrcode_cnt == 50:
    #        qrcode_cnt = -1
    #        qrcode_detected = 1

    #        LED(3).toggle()   #蓝灯
    #        time.sleep(200)
    #        LED(3).toggle()

    #elif total_white_pixels >= near_the_pole_threshold[0] and \
    #        total_white_pixels <= near_the_pole_threshold[1]:         #到达杆
    #    if pole_cnt >= 0 and pole_cnt <= 49:
    #        pole_cnt += 1
    #    if pole_cnt == 50:
    #        pole_cnt = -1
    #        send_flag.pole_is_near = 1




    print("the flags are: %d %d %d"%(barcode_detected, qrcode_detected, send_flag.pole_is_near))


############ 拍照 ###############

barcode_num = 0
qrcode_num = 0

# 拍摄照片
def shot_images_while_tracking_lines():
    #global had_finished_barcode, had_finished_qrcode
    global barcode_detected, qrcode_detected, barcode_num, qrcode_num

    if barcode_detected: #and (not had_finished_barcode):
        #sensor.set_framesize(sensor.QVGA)
        #barcode_detected = 0

        if barcode_num <= 3:
            barcode_num += 1
            barcode_name = "barcode_" + str(barcode_num)
            sensor.snapshot().save(barcode_name + ".jpg")

            print(barcode_name + ".jpg is saved!")
            time.sleep(5)


            if barcode_num >= 3:
                barcode_num = 0
                #barcode_detected = 0
                print("bar:", barcode_detected)
                #had_finished_barcode = 1
                return True

    if qrcode_detected: #and (not had_finished_qrcode):
        #sensor.set_framesize(sensor.QVGA)
        #qrcode_detected = 0

        if qrcode_num <= 4:
            qrcode_num += 1
            qrcode_name = "qrcode_" + str(qrcode_num)
            sensor.snapshot().save(qrcode_name + ".jpg")

            print(qrcode_name + ".jpg is saved!")
            time.sleep(5)

            if qrcode_num >= 3:
                qrcode_num = 0
                #qrcode_detected = 0
                print("qr:", qrcode_detected)
                #had_finished_qrcode = 1
                return True


pin2 = Pin('P2', Pin.OUT_PP, Pin.PULL_NONE)

############# 主函数 #########
while True:
    clock.tick()

    img = sensor.snapshot()#.binary([BINARY_THRESHOLD])
    count_pixels_with_movement(img)

    #uart_read_buf()
    print(send_flag.quadrotor_is_to_start)
    if not send_flag.quadrotor_is_to_start:

        print(rec_flag.quadrotor_is_stopped)
        if rec_flag.quadrotor_is_stopped:
            #rec_flag.quadrotor_is_stopped = 0

            #shot_images_while_tracking_lines()
            #send_flag.quadrotor_is_to_start = 1
            print("ok!")

            uart.write(pack_flag_data())

    if send_flag.pole_is_near:
        send_flag.pole_is_near = 0
        LED(1).off()
        LED(3).off()
        LED(4).off()

        LED(3).on()   #?
        #time.sleep(20)
    if LED_ON:
        #LED_ON = 0
        #lock_flag = 1
        LED(1).off()
        LED(2).off()
        LED(3).off()
        LED(4).off()

        shot_images_while_tracking_lines()   #?
        LED(2).on()

        pin2.value(1)    # 点灯

        sensor.skip_frames(time = 200)
    else:
        LED(2).off()

        pin2.value(0)

    if qrcode_detected:
        qrcode_detected = 0
        LED(1).off()
        LED(3).off()
        LED(4).off()

        LED(1).toggle()   #?
        sensor.skip_frames(time = 200)




