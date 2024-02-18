一：如何使用（这里只是一个测试DEMO，真实使用时拿出LQR.cpp和LQR.h即可）

	1.编译该包
	
	2.启动节点roslaunch lqr_track main.launch
	
	3.给定初始位姿 cd bagandshell  
	             ./pubvirtualpose.sh
	
	4.给定路径 cd bagandshell
	          rosbag play 2024-02-07-09-15-35.bag --clock 
	5.仿真时将launch参数内   is_simulate 写 true ,上真车时将/navi/base/pose/dist 替换为你的机器人发布的位姿     


​	





