import sensor, image, time ,json
from pid import PID
from pyb import UART
THRESHOLD =(29, 63, 38, 81, 3, 66)           #红线
uart = UART(3, 115200)                       #p4为TX,p5为RX
rho_pid = PID(p=-20, i=0)                    #rho为距离
theta_pid = PID(p=-0.2, i=0)
sensor.reset()                               # 初始化摄像头
sensor.set_vflip(True)                       #左右镜像
sensor.set_hmirror(True)                     #上下镜像
sensor.set_pixformat(sensor.RGB565)          #使用RGB565模式
sensor.set_framesize(sensor.QQQVGA)          #80*60分辨率
sensor.skip_frames(time = 2000)              # 跳过2000帧
clock = time.clock()                         # 返回一个时钟对象

while True:
    clock.tick()                                                        # 跟踪FPS帧率，开始追踪运行时间
    img = sensor.snapshot().binary([THRESHOLD])                         # 二值化图像
    line = img.get_regression([(100,100)],roi = ROI, robust = True)     #100,100代表色值max和min
    ROI_L = (0,0,30,50)
    ROI_R= (46,0,30,50)
    ROI = (39, 4, 15, 59)
    blobs1 = img.find_blobs([(100,100)],roi = ROI_L,area_threshold=15,merge=True)
    blobs = img.find_blobs([(100,100)],roi = ROI,area_threshold=15,merge=True)
    if blobs1:
        blobs2 = img.find_blobs([(100,100)],roi = ROI_R,area_threshold=15,merge=True)
        if blobs2:
            print('2')
            obj = [0]
            output = json.dumps(obj)
            uart.write("2"+"\r\n")          #识别到十字就停止运动

    if blobs:
        if line:
            rho_err = abs(line.rho())-47    #计算偏差，47为手动调整的值
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            img.draw_line(line.line(), color = 127) # 画出line线
            if line.magnitude()>8:  #如果线长大于8，进行PID控制
                rho_output = rho_pid.get_pid(rho_err,1)         #rho输出
                theta_output = theta_pid.get_pid(theta_err,1)   #theta计算输出
                output = rho_output+theta_output                #相加为输出
                obj = [output]
                output = json.dumps(obj)
                print("1"+output +"\r\n")
                uart.write("1"+output +"\r\n")
            else:
                print('you send:',0)
                obj = [0]
                output = json.dumps(obj)
                uart.write("1"+output+"\r\n")   #线的长度小于8，停
        else:
            print('you send:',0)
            obj = [0]
            output = json.dumps(obj)
            uart.write("1"+output+"\r\n")    #没有检测到线，停
            pass





