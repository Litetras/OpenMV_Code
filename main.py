THRESHOLD = (5, 70, -23, 15, -57, 0) # 参数分别代表左阈值，右阈值，上阈值，下阈值，噪声阈值，噪声阈值
# 请根据实际情况调整阈值
import sensor, image, time
from pyb import LED      # 用于指示程序运行状态
from pid import PID      #导入PID模块
from pyb import UART     #导入串口模块
uart = UART(3, 19200)    #初始化串口3，波特率为19200


rho_pid = PID(p=0.4, i=0)  #rho代表距离，theta代表角度
theta_pid = PID(p=0.001, i=0)

LED(1).on() # 指示程序正在运行
LED(2).on()
LED(3).on()

sensor.reset()              # 初始化摄像头
sensor.set_vflip(True)   #消除镜像
sensor.set_hmirror(True) #消除镜像

sensor.set_pixformat(sensor.RGB565) #使用RGB565模式
sensor.set_framesize(sensor.QQQVGA) #QQQVGA（80*60）分辨率
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # 等待摄像头初始化
clock = time.clock()                # 跟踪FPS帧率

while True:
    clock.tick()                                            # 跟踪FPS帧率
    img = sensor.snapshot().binary([THRESHOLD])             # 二值化图像
    line = img.get_regression([(100,100)], robust = True)   # 使用robust = True可以忽略噪声

    if line:
        rho_err = abs(line.rho())-img.width()/2 #计算偏差
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()    
        img.draw_line(line.line(), color = 127) # 画出线
        #print(rho_err,line.magnitude(),rho_err) # 打印偏差

        if line.magnitude()>8:  #如果线的长度大于8，就进行PID控制
            #if -40<b_err<40 and -30<t_err<30:
            rho_output = rho_pid.get_pid(rho_err,1)         #计算输出
            theta_output = theta_pid.get_pid(theta_err,1)   #计算输出
            output = rho_output+theta_output    #计算输出
            print('you send:',50+output,50-output)
            uart.write(50+output,50-output)
        else:
            print('you send:',0,0)
            uart.write(0,0)   #如果线的长度小于8，就停止

    else:
        print('you send:',50,-50)
        uart.write(50,-50)    #如果没有检测到线，就转弯
        pass

