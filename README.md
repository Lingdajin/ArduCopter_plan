# ArduCopter_plan
Control-system based on dronekit-python.

你好！这是一个与无人机相关的项目，旨在使用简单廉价的传感器实现无人机自动追踪特定物体。

本人使用的是大疆F450基础无人机，树莓派3B,raspi摄像头，HC-SR04超声波模块。

实际上，只要你使用带有apm固件的无人机，配合不低于树莓派2B的开发版和摄像头、超声波模块，都应该可以使用这段代码。

以下是本项目内各个代码文件的使用说明。

1.example1.py 这是基础的测试dronekit包的代码，运行这段代码可以让无人机解锁起飞至10m高度，停滞30s后降落。你应该把代码中的 'connection_string='/dev/ttyUSB0''改为你自己的端口。

2.distance.py 这是测试超声波模块的代码，把TRIG和ECHO改为你想要的端口号，运行后终端会循环显示超声波测距的距离。

3.fly.py 这是使用摄像头识别追踪红色物体的代码，当红色物体在视野左端，无人机会向左偏航，右端同理，保持红色物体始终在视野中央。

代码未完工，接下来会结合摄像头追踪和测距继续完善。
