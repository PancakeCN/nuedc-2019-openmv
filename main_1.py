import sensor, image, time
from pyb import LED
from pyb import UART,Timer

#import hello

#import car
#from pid import PID
#rho_pid = PID(p=0.4, i=0)
#theta_pid = PID(p=0.001, i=0)

uart = UART(3,115200)#初始化串口 波特率 115200

#LED(1).on()
#LED(2).on()
#LED(3).on()
sensor.reset()
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

up_roi   = [5, 0, 70, 5]#上采样区0  [0,   0, 80, 15]
down_roi = [5, 55, 70, 5]#下采样区0  [0, 55, 80, 15]
mid_roi  = [5, 5, 70, 50]#中心横向采样区 [15, 15, 50, 30]
left_roi = [0,  0,  5, 60]#左采样区0  [0,   0, 25, 60]
righ_roi = [75, 0,  5, 60]#右采样区0  [55, 0,  25, 40]

BINARY_THRESHOLD = (30, 120) # Grayscale threshold for dark things... (5, 70, -23, 15, -57, 0)(18, 100, 31, -24, -21, 70)   (0, 100)
thresholds = [(30, 0, -128, 127, -128, 127)]# [0, 144]#自定义灰度阈值
centre_thresholds = [0, 144]

centre_area_x = [35, 45]  #自定义中心检测区域  #
centre_area_y = [25, 35]

################ 自定义类 #######################

class Dot(object):
    x = 0
    y = 0
    pixels = 0
    num = 0
    ok = 0
    flag = 0

class Line(Dot):
    x_angle = 0
    y_angle = 0
    flag = 0
    centre_x = 0
    centre_y = 0

class singleline_check():
    flager = 0
    rho_err = 0
    theta_err = 0

class receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0
R=receive()

class ctrl(object):
    work_mode = 0x02 #工作模式.默认是点检测，可以通过串口设置成其他模式 0x01:点模式 0x02:线模式
    check_show = 1   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

ctr=ctrl()

dot  = Dot()
up   = Line()
down = Line()
left = Line()
righ = Line()
line = Line()
mid  = Line()
singleline_check = singleline_check()
line.flag = 0

old_cross_x = 0
old_cross_y = 0

no_vertical_angle = 1

#img_to_vertical =
#lines_to_vertical = 0

CONVERT_TO_VERTICAL = 1

muti_lines_err_cnt = 0

first_time_vertical = False

################### 数据打包 #############################

#线检测数据打包
def pack_line_data():

    pack_data=bytearray([0xAA,0xAF,0xF3,0x00,
        singleline_check.rho_err>>8,singleline_check.rho_err,
        singleline_check.theta_err>>8,singleline_check.theta_err,
        line.flag,0x00,0x00,0x00])
    singleline_check.rho_err = 0
    singleline_check.theta_err = 0

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

#点检测数据打包
def pack_dot_data():
    pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
        dot.x>>8,dot.x,
        dot.y>>8,dot.y,dot.num>>8,dot.num,
        dot.flag,0x00])

    dot.x = 0
    dot.y = 0

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

################### 串口 ####################

#串口数据解析
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

    if data_buf[2]==0xFC:

        #设置模块工作模式
        ctr.work_mode = data_buf[4]

        #print("Set work mode success!")


#串口通信协议接收
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

#读取串口缓存
def uart_read_buf():
    i = 0
    buf_size = uart.any()
    while i<buf_size:
        Receive_Prepare(uart.readchar())
        i = i + 1

################## 点检测 #############################
#点检测函数
def check_dot(img):
    #thresholds为黑色物体颜色的阈值，是一个元组，需要用括号［ ］括起来可以根据不同的颜色阈值更改；pixels_threshold 像素个数阈值，
    #如果色块像素数量小于这个值，会被过滤掉area_threshold 面积阈值，如果色块被框起来的面积小于这个值，会被过滤掉；merge 合并，如果
    #设置为True，那么合并所有重叠的blob为一个；margin 边界，如果设置为5，那么两个blobs如果间距5一个像素点，也会被合并。
    for blob in img.find_blobs(thresholds, pixels_threshold=80, area_threshold=80, merge=True, margin=5):
        if dot.pixels<blob.pixels():#寻找最大的黑点
            ##先对图像进行分割，二值化，将在阈值内的区域变为白色，阈值外区域变为黑色
            img.binary(thresholds)
            #对图像边缘进行侵蚀，侵蚀函数erode(size, threshold=Auto)，size为kernal的大小，去除边缘相邻处多余的点。threshold用
            #来设置去除相邻点的个数，threshold数值越大，被侵蚀掉的边缘点越多，边缘旁边白色杂点少；数值越小，被侵蚀掉的边缘点越少，边缘
            #旁边的白色杂点越多。
            img.erode(2)
            dot.pixels=blob.pixels() #将像素值赋值给dot.pixels
            dot.x = blob.cx() #将识别到的物体的中心点x坐标赋值给dot.x
            dot.y = blob.cy() #将识别到的物体的中心点x坐标赋值给dot.x
            dot.ok= 1
            #在图像中画一个十字；x,y是坐标；size是两侧的尺寸；color可根据自己的喜好设置
            img.draw_cross(dot.x, dot.y, color=127, size = 10)
            #在图像中画一个圆；x,y是坐标；5是圆的半径；color可根据自己的喜好设置
            img.draw_circle(dot.x, dot.y, 5, color = 127)

            print("centre_x = %d, centre_y = %d"%(dot.x, dot.y))

    #判断标志位 赋值像素点数据
    dot.flag = dot.ok
    dot.num = dot.pixels

    #清零标志位
    dot.pixels = 0
    dot.ok = 0

    #发送数据
    uart.write(pack_dot_data())

############################### 直角检测 #####################################


# min_degree = 0 # 直线最小角度
# max_degree = 179 # 直线最大角度

# 判断是否为直角的阈值
right_angle_threshold = (70, 100)
binary_threshold = [(0, 60)]
forget_ratio = 0.8
move_threshold = 5

def calculate_angle(line1, line2):
    # 利用四边形的角公式， 计算出直线夹角
    angle  = (180 - abs(line1.theta() - line2.theta()))
    if angle > 90:
        angle = 180 - angle

    return angle


def is_right_angle(line1, line2):
    global right_angle_threshold
    # 判断两个直线之间的夹角是否为直角
    angle = calculate_angle(line1, line2)

    if angle >= right_angle_threshold[0] and angle <=  right_angle_threshold[1]:
        # 判断在阈值范围内
        print("The angle is", angle)
        return True
    return False

def find_verticle_lines(lines):
    line_num = len(lines)
    for i in range(line_num -1):
        for j in range(i, line_num):
            if is_right_angle(lines[i], lines[j]):
                return (lines[i], lines[j])
    return (None, None)


def calculate_intersection(line1, line2):
    # 计算两条线的交点
    a1 = line1.y2() - line1.y1()
    b1 = line1.x1() - line1.x2()
    c1 = line1.x2()*line1.y1() - line1.x1()*line1.y2()

    a2 = line2.y2() - line2.y1()
    b2 = line2.x1() - line2.x2()
    c2 = line2.x2() * line2.y1() - line2.x1()*line2.y2()

    if (a1 * b2 - a2 * b1) != 0 and (a2 * b1 - a1 * b2) != 0:
        cross_x = int((b1*c2-b2*c1)/(a1*b2-a2*b1))
        cross_y = int((c1*a2-c2*a1)/(a1*b2-a2*b1))
        return (cross_x, cross_y)
    return (-1, -1)


def draw_cross_point(cross_x, cross_y):
    img.draw_cross(cross_x, cross_y, color = 127)
    img.draw_circle(cross_x, cross_y, 5, color = 127)
    img.draw_circle(cross_x, cross_y, 10, color = 127)
# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.



def check_whether_verticle_lines(img, lines, first_time_vertical):
    global  old_cross_x, old_cross_y

    if first_time_vertical:
        first_time_vertical = False
        img = lines = None

    else:
        lines =  img.find_lines(threshold = 2000, theta_margin = 50, rho_margin = 50, roi = mid_roi)  #roi=(5, 5, 150,110)



    for line in lines:
        pass
        #img.draw_line(line.line(), color = (255, 0, 0))
    # 如果画面中有两条直线

    if len(lines) >= 2:
        (line1, line2) = find_verticle_lines(lines)
        if (line1 == None or line2 == None):
            # 没有垂直的直线
            draw_cross_point(old_cross_x, old_cross_y)

            LED(2).off()  #熄灯
            LED(3).off()  #熄灯
            no_vertical_angle = 1
            return False

        LED(2).toggle()  #亮灯

        no_vertical_angle = 0
        # 画线
        img.draw_line(line1.line(), color = 127)
        img.draw_line(line2.line(), color = 127)

        # 计算交点
        (cross_x, cross_y) = calculate_intersection(line1, line2)
        print("cross_x:  %d, cross_y: %d"%(old_cross_x, old_cross_y))

        if cross_x != -1 and cross_y != -1:
            if abs(cross_x - old_cross_x) < move_threshold and abs(cross_y - old_cross_y) < move_threshold:
            # 小于移动阈值， 不移动
                pass
            else:
                old_cross_x = int(old_cross_x * (1 - forget_ratio) + cross_x * forget_ratio)
                old_cross_y = int(old_cross_y * (1 - forget_ratio) + cross_y * forget_ratio)


        draw_cross_point(old_cross_x, old_cross_y)

    #print("FPS %f" % clock.fps())

################################### 找线 ########################################

# 寻找区域中心坐标   #
def find_area_centre(img,area,area_roi):
    img.invert()
    blobs = img.find_blobs([centre_thresholds], roi=area_roi, pixels_threshold=3, area_threshold=3, merge=True, margin=5)
    if blobs:
        most_pixels = 0
        largest_blob = 0

        for i in range(len(blobs)):
            if blobs[i].pixels() > most_pixels:
                most_pixels = blobs[i].pixels()
                largest_blob = i

        img.draw_rectangle(blobs[largest_blob].rect())

        img.draw_cross(blobs[largest_blob].cx(),
                       blobs[largest_blob].cy())

        area.centre_x = blobs[largest_blob].cx()
        area.centre_y = blobs[largest_blob].cy()

    img.invert()


def fine_border(img,area,area_roi):

    line.flag = img.get_regression([(255,255)],roi=area_roi, robust = True)
    if (line.flag):
        area.ok=1
    #判断标志位
    #dot.flag = dot.ok
    #清零标志位

    #dot.pixels = 0
    #dot.ok = 0

    find_area_centre(img,area,area_roi)

#找线
def found_line(img):
    singleline_check.flager = img.get_regression([(255,255)], robust = True)
    if (singleline_check.flager):
        #print(clock.fps())
        singleline_check.rho_err = abs(singleline_check.flager.rho())-0
        if singleline_check.flager.theta()>90:
            singleline_check.theta_err = singleline_check.flager.theta()-0
        else:
            singleline_check.theta_err = singleline_check.flager.theta()-0
        img.draw_line(singleline_check.flager.line(), color = 127)
        print("rho: %d, theta: %d"%(singleline_check.rho_err, singleline_check.theta_err))

# 检测是否有两条以上直线
def check_whether_muti_lines(img):
    lines =  img.find_lines(threshold = 2000, theta_margin = 50, rho_margin = 50, roi = mid_roi)  #threshold = 2000, theta_margin = 40, rho_margin = 20, roi=(5, 5, 150,110)

    for line in lines:
        pass

    if len(lines) >= 2:
        return [True, lines]
    else:
        return [False, None]



def check_line(img):

    global muti_lines_err_cnt, first_time_vertical

    fine_border(img,up,up_roi)
    fine_border(img,down,down_roi)
    fine_border(img,left,left_roi)
    fine_border(img,righ,righ_roi)
    fine_border(img,mid,mid_roi)    #

    line.flag = 0
    if up.ok:
        line.flag = line.flag | 0x01
    if down.ok:
        line.flag = line.flag | 0x02
    if left.ok:
        line.flag = line.flag | 0x04
    if righ.ok:
        line.flag = line.flag | 0x08
    if mid.ok:
        line.flag = line.flag | 0x10  #
    #print(line.flag)

    up.ok = down.ok = left.ok = righ.ok = mid.ok = 0
    up.num = down.num = left.num = righ.num = mid.num = 0
    up.pixels = down.pixels = left.pixels = righ.pixels = mid.pixels = 0

    [Flag, lines] = check_whether_muti_lines(img)
    if Flag:       #检测是否有两条以上的线
        muti_lines_err_cnt = muti_lines_err_cnt + 1
        if muti_lines_err_cnt < 100:
            return False

        muti_lines_err_cnt = 0
        #time.sleep(50)     #延时150ms
        #LED(2).off()
        ctr.work_mode = 0x03    #换到直角检测模式
        img_to_vertical = img
        lines_to_vertical = lines
        first_time_vertical = True
        return (img_to_vertical, lines_to_vertical)
        #check_whether_verticle_lines(img, lines)  # 寻找直角
    else:
        LED(2).off()
        LED(3).toggle()  #亮灯
        #time.sleep(50)     #延时150ms
        #LED(3).off()
        found_line(img)   #巡线

    #发送数据
    uart.write(pack_line_data())

########################## 主函数 ##############################

while(True):
    clock.tick()

    #img = sensor.snapshot().binary([THRESHOLD])
    #img = sensor.snapshot()

    if (ctr.work_mode&0x01)!=0:
        sensor.set_pixformat(sensor.RGB565)
        img = sensor.snapshot()
        check_dot(img)
        LED(1).toggle()      #亮灯
        #time.sleep(50)     #延时150ms
        #LED(1).off()

    #线检测
    if (ctr.work_mode&0x02)!=0:
        sensor.set_pixformat(sensor.GRAYSCALE)
        img = sensor.snapshot().binary([BINARY_THRESHOLD])
        #img.erode(1)
        #LED(3).toggle()  #亮灯
        #time.sleep(50)     #延时150ms
        #LED(3).off()
        if (no_vertical_angle):
            (img_to_vertical, lines_to_vertical) = check_line(img)
        #LED(3).on()        #亮灯
        #time.sleep(10)     #延时150ms
        #LED(3).off()

    if (ctr.work_mode&0x03)!=0:
        sensor.set_pixformat(sensor.GRAYSCALE)
        img = sensor.snapshot().binary([BINARY_THRESHOLD])

        LED(2).off()
        LED(3).off()
        check_whether_verticle_lines(img_to_vertical, lines_to_vertical, first_time_vertical)  # 寻找直角



    #接收串口数据
    uart_read_buf()
    #found_line(img)
    #print(clock.fps())


