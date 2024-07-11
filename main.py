from maix import uart,camera, display, image,time
import math
import numpy as np


""" sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     #延时跳过一些帧，等待感光元件变稳定
sensor.set_auto_gain(True)          #黑线不易识别时，将此处写False
sensor.set_auto_whitebal(False) """

            # Create a clock object to track the FPS.
#sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光sensor.get_exposure_us()

cam=camera.Camera(640,480)
cam.skip_frames(30)


devices=uart.list_devices()
#uart = uart.UART(devices[0],115200)

uart = uart.UART(devices[0],115200)
THRESHOLD = (0,100) # Grayscale threshold for dark things... (5, 70, -23, 15, -57, 0)(18, 100, 31, -24, -21, 70)
#IMAGE_WIDTH=sensor.snapshot().width()
#IMAGE_HEIGHT=sensor.snapshot().height()
IMAGE_WIDTH=640
IMAGE_HEIGHT=480
IMAGE_DIS_MAX=(int)(math.sqrt(IMAGE_WIDTH*IMAGE_WIDTH+IMAGE_HEIGHT*IMAGE_HEIGHT)/2)



class target_check(object):
    x=0          #int16_t
    y=0          #int16_t
    pixel=0      #uint16_t
    flag=0       #uint8_t
    state=0      #uint8_t
    angle=0      #int16_t
    distance=0   #uint16_t
    apriltag_id=0#uint16_t
    img_width=0  #uint16_t
    img_height=0 #uint16_t
    reserved1=0  #uint8_t
    reserved2=0  #uint8_t
    reserved3=0  #uint8_t
    reserved4=0  #uint8_t
    fps=0        #uint8_t
    range_sensor1=0
    range_sensor2=0
    range_sensor3=0
    range_sensor4=0
    camera_id=0
    reserved1_int32=0
    reserved2_int32=0
    reserved3_int32=0
    reserved4_int32=0

class uart_buf_prase(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class mode_ctrl(object):
    work_mode = 0x01 #工作模式.默认是点检测，可以通过串口设置成其他模式
    check_show = 0   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

ctr=mode_ctrl()
if ctr.check_show==1:
    dis=display.Display()
R=uart_buf_prase()
target=target_check();
target.camera_id=0x01
target.reserved1_int32=65536
target.reserved2_int32=105536
target.reserved3_int32=65537
target.reserved4_int32=105537

HEADER=[0xFF,0xFC]
MODE=[0xF1,0xF2,0xF3]
#__________________________________________________________________
def package_blobs_data(mode):
    #数据打包封装
    data=bytearray([HEADER[0],HEADER[1],0xA0+mode,0x00,
                   target.x>>8&0xff,target.x&0xff,        #将整形数据拆分成两个8位
                   target.y>>8&0xff,target.y&0xff,        #将整形数据拆分成两个8位
                   target.pixel>>8&0xff,target.pixel&0xff,#将整形数据拆分成两个8位
                   target.flag,                 #数据有效标志位
                   target.state,                #数据有效标志位
                   target.angle>>8&0xff,target.angle&0xff,#将整形数据拆分成两个8位
                   target.distance>>8&0xff,target.distance&0xff,#将整形数据拆分成两个8位
                   target.apriltag_id>>8&0xff,target.apriltag_id&0xff,#将整形数据拆分成两个8位
                   target.img_width>>8&0xff,target.img_width&0xff,    #将整形数据拆分成两个8位
                   target.img_height>>8&0xff,target.img_height&0xff,  #将整形数据拆分成两个8位
                   target.fps,      #数据有效标志位
                   target.reserved1,#数据有效标志位
                   target.reserved2,#数据有效标志位
                   target.reserved3,#数据有效标志位
                   target.reserved4,#数据有效标志位
                   target.range_sensor1>>8&0xff,target.range_sensor1&0xff,
                   target.range_sensor2>>8&0xff,target.range_sensor2&0xff,
                   target.range_sensor3>>8&0xff,target.range_sensor3&0xff,
                   target.range_sensor4>>8&0xff,target.range_sensor4&0xff,
                   target.camera_id,
                   target.reserved1_int32>>24&0xff,target.reserved1_int32>>16&0xff,
                   target.reserved1_int32>>8&0xff,target.reserved1_int32&0xff,
                   target.reserved2_int32>>24&0xff,target.reserved2_int32>>16&0xff,
                   target.reserved2_int32>>8&0xff,target.reserved2_int32&0xff,
                   target.reserved3_int32>>24&0xff,target.reserved3_int32>>16&0xff,
                   target.reserved3_int32>>8&0xff,target.reserved3_int32&0xff,
                   target.reserved4_int32>>24&0xff,target.reserved4_int32>>16&0xff,
                   target.reserved4_int32>>8&0xff,target.reserved4_int32&0xff,
                   0x00])
    #数据包的长度
    data_len=len(data)
    data[3]=(data_len-5)#有效数据的长度
    #和校验
    sum=0
    for i in range(0,data_len-1):
        sum=sum+data[i]
    data[data_len-1]=sum&0xff
    #返回打包好的数据
    return data
#__________________________________________________________________

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
    if data_buf[2]==0xA0:
        #设置模块工作模式
        ctr.work_mode = data_buf[4]
        print(ctr.work_mode)
        print("Set work mode success!")

#__________________________________________________________________
def uart_data_prase(buf):
    if R.state==0 and buf==0xFF:#帧头1
        R.state=1
        R.uart_buf.append(buf)
    elif R.state==1 and buf==0xFE:#帧头2
        R.state=2
        R.uart_buf.append(buf)
    elif R.state==2 and buf<0xFF:#功能字
        R.state=3
        R.uart_buf.append(buf)
    elif R.state==3 and buf<50:#数据长度小于50
        R.state=4
        R._data_len=buf  #有效数据长度
        R._data_cnt=buf+5#总数据长度
        R.uart_buf.append(buf)
    elif R.state==4 and R._data_len>0:#存储对应长度数据
        R._data_len=R._data_len-1
        R.uart_buf.append(buf)
        if R._data_len==0:
            R.state=5
    elif R.state==5:
        R.uart_buf.append(buf)
        R.state=0
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
#        print(R.uart_buf)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state=0
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
#__________________________________________________________________

def uart_data_read():
    uart_data_prase(uart.read())

# 绘制水平线
def draw_hori_line(img, x0, x1, y, color):
    for x in range(x0, x1):
        img.set_pixel(x, y, color)
# 绘制竖直线
def draw_vec_line(img, x, y0, y1, color):
    for y in range(y0, y1):
        img.set_pixel(x, y, color)
# 绘制矩形
def draw_rect(img, x, y, w, h, color):
    draw_hori_line(img, x, x+w, y, color)
    draw_hori_line(img, x, x+w, y+h, color)
    draw_vec_line(img, x, y, y+h, color)
    draw_vec_line(img, x+w, y, y+h, color)


blob_threshold_rgb=[40, 100,30,127,0,127]#(L Min, L Max, A Min, A Max, B Min, B Max)
#blob_threshold_rgb=[30, 100,15,127,15,127]#(L Min, L Max, A Min, A Max, B Min, B Max)
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds_rgb = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
                  (30, 100, -64, -8, -32, 32), # generic_green_thresholds
                  (0, 30, 0, 64, -128, 0)]     # generic_blue_thresholds

#寻色块
def opv_find_color_blob():
    target.flag=0
    if (ctr.work_mode&0x01)!=0:
        img=cam.read()
        target.img_width=IMAGE_WIDTH
        target.img_height=IMAGE_HEIGHT
        pixels_max=0
        for b in img.find_blobs([blob_threshold_rgb],pixels_threshold=30,merge=True,margin=50):
    #        img.draw_rect(b[0:4])#圈出搜索到的目标
#            img.draw_rect(b[0:4],(255,0,0),thickness=-1)
            img.draw_rect(b[0],b[1],b[2],b[3], image.Color.from_rgb(255,0,0),thickness=1)
            if pixels_max<b.pixels():
                pixels_max=b.pixels()
                target.x = b.cx()
                target.y = b.cy()
                target.pixel=pixels_max
                target.reserved1=b.w()>>8
                target.reserved2=b.w()
                target.flag = 1
        if target.flag==1:
            img.draw_cross(target.x,target.y,image.Color.from_rgb(127,0,0), size = 15,thickness=1)
            img.draw_circle(target.x,target.y, 15, image.Color.from_rgb(127,0,0))
        if ctr.check_show==1:
            dis.show(img)
#        print(target.x,target.y,target.pixel,target.reserved1,target.reserved2)

b=0
#寻Apriltag
def opv_find_april_tag():
    img=cam.read()
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    apriltag_area=0
    apriltag_dis=IMAGE_DIS_MAX
    target.flag = 0
    for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
        mytect=tag.rect()
        img.draw_rect(tag.rect()[0],tag.rect()[1],tag.rect()[2],tag.rect()[3], color = image.Color.from_rgb(255, 0, 0),thickness=-1)
        b=(tag.rect()[2]/(abs(math.sin(tag.rotation()))+abs(math.cos(tag.rotation()))))
        #print(tag.rect()[2],b*b)
        #保存最大像素面积得apritag信息
        apriltag_dis_tmp=math.sqrt((tag.cx()-80)*(tag.cx()-80)+(tag.cy()-60)*(tag.cy()-60))
        apriltag_area_tmp=tag.w()*tag.h()
        if apriltag_dis>apriltag_dis_tmp:
            apriltag_area=tag.w()*tag.h()
            target.x = tag.cx()
            target.y = tag.cy()
            target.apriltag_id=tag.id()
            target.pixel=int(b*b)#apriltag_area
            apriltag_dis=apriltag_dis_tmp
            target.flag = 1
    if target.flag==1:
        img.draw_cross(target.x,target.y, image.Color.from_rgb(127,0,0), size = 15)
#        img.draw_circle(target.x,target.y, 15, color = 127)
#    print(target.x,target.y,target.pixel,target.apriltag_id,apriltag_dis)

class singleline_check():
    rho_err = 0
    theta_err = 0
    state = 0

singleline = singleline_check()
THRESHOLD = (0,100) # Grayscale threshold for dark things
thresholds =[[0, 30, -30, 30, -30, 30]]
#找线
def found_line():
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    target.flag = 0
    #sensor.set_pixformat(sensor.GRAYSCALE)
    #img=sensor.snapshot().binary([THRESHOLD])
    img=cam.read()
    target.img_width =IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    pixels_max=0
    lines = img.get_regression(thresholds, area_threshold = 100)
    for a in lines:
       img.draw_line(a.x1(), a.y1(), a.x2(), a.y2(), image.COLOR_GREEN, 2)
       theta = a.theta()
       rho = a.rho()
       if theta > 90:
          theta = 270 - theta
       else:
          theta = 90 - theta
       target.angle = int(theta)
       target.x=int(rho)
       target.flag=1
#crops_threshold=(40, 70, -30, 10, -15, 30)
crops_threshold=[[40, 70, -40, 0, 0, 40]]
def find_crops():
    target.flag=0
    if (ctr.work_mode&0x01)!=0:
        img=cam.read()
        target.img_width=IMAGE_WIDTH
        target.img_height=IMAGE_HEIGHT
        pixels_max=0
        crops_roi=(int(target.img_width/2)-5,int(target.img_height/2)-5,10,10)#(x,y,w,h)
        for b in img.find_blobs(crops_threshold,pixels_threshold=70,merge=True,margin=10):
            img.draw_rect(b[0],b[1],b[2],b[3], image.Color.from_rgb(255,0,0),thickness=-1)
            if pixels_max<b.pixels():
                pixels_max=b.pixels()
                target.x = b.cx()
                target.y = b.cy()
                target.pixel=pixels_max
                target.reserved1=b.w()>>8
                target.reserved2=b.w()
                target.flag = 1
        img.draw_rect(crops_roi[0],crops_roi[1],crops_roi[2],crops_roi[3], image.Color.from_rgb(255,0,0),thickness=1)        
        if target.flag==1:
            img.draw_cross(target.x,target.y, image.Color.from_rgb(127,0,0), size = 15)
            img.draw_circle(target.x,target.y, 15, image.Color.from_rgb(127,0,0))
        #print(target.x,target.y,target.pixel,target.flag,target.reserved1,target.reserved2)

def send_data_via_uart(data):
    for byte in data:
        uart.write(byte)
        # 等待 UART 发送缓冲区空闲（具体方法取决于你的平台和代码结构）

ctr.work_mode=0x01
last_ticks=0
ticks=0
ticks_delta=0;
while True:
    if ctr.work_mode==0x00:#空闲模式
        img=cam.read()
    elif ctr.work_mode==0x01:#色块模式
        opv_find_color_blob()

    elif ctr.work_mode==0x02:#AprilTag模式
        opv_find_april_tag()
    elif ctr.work_mode==0x03:#巡线模式
        found_line()

    elif ctr.work_mode==0x04:#AprilTag模式
        opv_find_april_tag()

    elif ctr.work_mode==0x05:#预留模式1
        img=cam.read()

    elif ctr.work_mode==0x06:#预留模式2
        img=cam.read()
 
    elif ctr.work_mode==0x07:#识别底部颜色，用于2021年国赛植保飞行器
        find_crops()

    elif ctr.work_mode==0x0B:#识别底部颜色，用于2021年国赛植保飞行器
        find_crops()
    else:
        pass

    uart.write(bytes(package_blobs_data(ctr.work_mode)))
#    uart.write(bytes(target.x))
#    send_data_via_uart(package_blobs_data(ctr.work_mode))
    uart_data_read()

#__________________________________________________________________
    #计算fps
    last_ticks=ticks
    ticks=time.time_ms()#ticks=time.ticks_ms()
                      #新版本OPENMV固件使用time.ticks_ms()
                      #旧版本OPENMV固件使用time.ticks()
    ticks_delta=ticks-last_ticks
    if ticks_delta<1:
        ticks_delta=1
    target.fps=(int)(1000/ticks_delta)
    #target.fps = (int)(clock.fps())
#__________________________________________________________________
    print(target.fps,ticks-last_ticks,ctr.work_mode)
