std_msgs/Header header

uint8 type 				              #底盘类型

uint8 lidar_num                 #激光雷达数量
uint8 camera_num                #相机数量
uint8 ultrasonic_num            #超声波数量
uint8 gps_num                   #gps数量

bool emergency_stop_status 			#急停按钮状态   false：没触发  true：触发

bool odom_status				        #里程计状态     false:异常    true：正常
bool gps_status				          #gps状态       false:异常    true：正常
bool lidar_status				        #激光雷达状态    false:异常    true：正常
bool imu_status			    	      #imu状态        false:异常    true：正常
uint8 ultrasonic_status        #超声波状态      false:异常    true：正常
uint8 cameras_status             #摄像头状态       0:前后摄像头都异常  1：前摄像头正常，后摄像头异常   2：前摄像头异常，后摄像头正常

uint8 ultrasonic_get_obs 			    #超声波触不触发    
                                  # 7 6 5 4 3 2 1 0   一个字节8位，某个位为1表示触发，某个位为0表示没触发， 共8个超声波，第0位对应第1个超声波，
                                  # 第7位对应第8个超声波

uint8 cameras_get_obs             #前后相机检测障碍物结果   0：都没检测到障碍物   1：前摄像头检测到障碍物   2：后摄像头检测到障碍物    3：前后都检测到障碍物

uint8 anticollision_status 	      #防撞条状态       0：没触发   1：前防撞条触发   2：后防撞条触发   3：前后防撞条都触发

uint16 error_code 				        #故障代码         0：正常状态

uint8 battery_percentage          #电量百分比

uint16 io_status 				          #io状态          0：io没动作

uint8 charge_status 				      #充电状态         0:没处于充电中    1:处于充电中
