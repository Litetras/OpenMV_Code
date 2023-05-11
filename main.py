# Description: 本程序为OpenMV的主程序，通过串口与单片机通信，接收单片机发送的图像数据，进行图像处理，然后发送处理结果给单片机
import sensor, image, time, json, pyb
from pid import PID
from pyb import UART,Pin, Timer
THRESHOLD = (29, 63, 38, 81, 3, 66) # 红线
uart = UART(3, 115200)  # p4为TX,p5为RX
rho_pid = PID(p=-20, i=0)   # rho是直线到图像正中心的距离
theta_pid = PID(p=-0.2, i=0)
sensor.reset()
sensor.set_vflip(True)      # 左右镜像
sensor.set_hmirror(True)    # 上下镜像
sensor.set_pixformat(sensor.RGB565) # 使用RGB565模式
sensor.set_framesize(sensor.QQQVGA) # 80*60分辨率
sensor.skip_frames(time=1000)   # 跳过1000帧
clock = time.clock()        # 跟踪FPS帧率
global cross    #十字路口标志位


# 照明模块50kHz pin6 timer2 channel1#####
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(75) # 控制亮度 0~100
############

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) # 二值化图像
    ROI = (39, 0, 15, 59)  						 # 47为小车摄像头正中间
    blobs = img.find_blobs([(100, 100)], roi=ROI, area_threshold=15, merge=True)    # 100,100代表色值max和min
    line = img.get_regression([(100, 100)], roi=ROI, robust=True)
    ROI_L = (0, 45, 32, 50)
    ROI_R = (59, 45, 21, 50)
    blobs1 = img.find_blobs([(100, 100)], roi=ROI_L, area_threshold=15, merge=True)
    blobs2 = img.find_blobs([(100, 100)], roi=ROI_R, area_threshold=15, merge=True)

    #试试img.draw_rectangle(ROI)
    img.draw_line((35, 0, 35, 60), color=(0, 255, 0))   # !注意画的线颜色也会被色块查找函数使用，所以不要画白线
    img.draw_line((59, 0, 59, 60), color=(0, 255, 0))

    img.draw_line((0, 45, 35, 45), color=(0, 255, 0))
    img.draw_line((59, 45, 80, 45), color=(0, 255, 0))
    if blobs1 and blobs2:
        cross = 1	#十字路口标志位
        print('2')
        uart.write("2" + "\r\n")
        pyb.delay(500)
    else:
        cross = 0
        pass
    if blobs:
        if line:
            rho_err = abs(line.rho()) - 47	# 47为小车摄像头正中间
            if line.theta() > 90:
                theta_err = line.theta() - 180#为了便于控制小车的运动，我们将角度值减去180度，将其转换为水平方向上的相反角度
            else:
                theta_err = line.theta()
            img.draw_line(line.line(), color=127)
            if line.magnitude() > 8:
                rho_output = rho_pid.get_pid(rho_err, 1)
                theta_output = theta_pid.get_pid(theta_err, 1)
                output = rho_output + theta_output
                obj = [output]
                output = json.dumps(obj)
                print("1" + output + "\r\n")
                uart.write("1" + output + "\r\n")
            else:	#线太短，停止
                print(3)
                uart.write("3"  + "\r\n")
                pass
        else:	#没有找到线
            print(3)
            uart.write("3" + "\r\n")
            pass


