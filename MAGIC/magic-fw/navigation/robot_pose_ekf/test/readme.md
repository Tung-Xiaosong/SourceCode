### odom_raw_file (islam_can_node.cpp)

| 列号 |           属性            |
| :--: | :-----------------------: |
|  1   | ros::Time::now().toSec()  |
|  2   | odom.pose.pose.position.x |
|  3   | odom.pose.pose.position.y |
|  4   | odom.pose.pose.position.z |
|  5   | odom.twist.twist.linear.x |
|  6   | odom.twist.twist.linear.y |
|  7   | odom.twist.twist.linear.z |

|  1   |  2  |  3  |  4  |    5     |    6     |    7     |
| :--: | :-: | :-: | :-: | :------: | :------: | :------: |
| Time |  x  |  y  |  z  | linear.x | linear.y | linear.z |

---

### odom_meas_file (odom_estimation.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |        odom_rel(1)       |
|  3   |        odom_rel(2)       |
|  4   |        odom_rel(3)       |
|  5   |        odom_rel(4)       |
|  6   |        odom_rel(5)       |
|  7   |        odom_rel(6)       |
|  8   |           msg            |

msg=odom_covariance_(odom_covariance_.rows(),odom_covariance_.columns())

|  1   |   2    |   3    |   4    |   5    |   6    |   7    |  8  |
| :--: | :----: | :----: | :----: | :----: | :----: | :----: | :-: |
| Time | rel(1) | rel(2) | rel(3) | rel(4) | rel(5) | rel(6) | msg |

---

### ekf_odom_file (odom_estimation_node.cpp)

| 列号 |              属性              |
| :--: | :----------------------------: |
|  1   |    ros::Time::now().toSec()    |
|  2   |   odom_meas_.getOrigin().x()   |
|  3   |   odom_meas_.getOrigin().y()   |
|  4   |              msg               |

|  1   |  2  |  3  |  4  |
| :--: | :-: | :-: | :-: |
| Time |  x  |  y  | yaw |

---

### imu_raw_file (islam_imu_node.cpp)

| 列号 |              属性              |
| :--: | :----------------------------: |
|  1   |    ros::Time::now().toSec()    |
|  2   |              roll              |
|  3   |             pitch              |
|  4   |              yaw               |
|  5   |  imu_data.angular_velocity.x   |
|  6   |  imu_data.angular_velocity.y   |
|  7   |  imu_data.angular_velocity.z   |
|  8   | imu_data.linear_acceleration.x |
|  9   | imu_data.linear_acceleration.y |
|  10  | imu_data.linear_acceleration.z |

|  1   |  2   |   3   |  4  |     5     |     6     |     7     |    8     |    9     |    10    |
| :--: | :--: | :---: | :-: | :-------: | :-------: | :-------: | :------: | :------: | :------: |
| Time | roll | pitch | yaw | angular.x | angular.y | angular.z | linear.x | linear.y | linear.z |

---

### imu_meas_file (odom_estimation.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |        imu_rel(1)        |
|  3   |        imu_rel(2)        |
|  4   |        imu_rel(3)        |
|  5   |           msg            |

msg=imu_covariance_(imu_covariance_.rows(),imu_covariance_.columns())

|  1   |   2    |   3    |   4    |  5  |
| :--: | :----: | :----: | :----: | :-: |
| Time | rel(1) | rel(2) | rel(3) | msg |

---

### ekf_imu_file (odom_estimation_node.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |           roll           |
|  3   |          pitch           |
|  4   |           yaw            |

|  1   |  2   |   3   |  4  |
| :--: | :--: | :---: | :-: |
| Time | roll | pitch | yaw |

---

### gps_raw_file (utm_odometry_node.cpp)

| 列号 |           属性            |
| :--: | :-----------------------: |
|  1   | ros::Time::now().toSec()  |
|  2   | odom.pose.pose.position.x |
|  3   | odom.pose.pose.position.y |
|  4   | odom.pose.pose.position.z |
|  5   |         roll_gps          |
|  6   |         pitch_gps         |
|  7   |          yaw_gps          |
|  8   |            buf            |

|  1   |  2  |  3  |  4  |  5   |   6   |  7  |  8  |
| :--: | :-: | :-: | :-: | :--: | :---: | :-: | :-: |
| Time |  x  |  y  |  z  | roll | pitch | yaw | buf |

---

### gps_meas_file (odom_estimation.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |        gps_vec(1)        |
|  3   |        gps_vec(2)        |
|  4   |        gps_vec(3)        |
|  5   |           msg            |

msg=gps_covariance_(gps_covariance_.rows(),gps_covariance_.columns())

|  1   |   2    |   3    |   4    |  5  |
| :--: | :----: | :----: | :----: | :-: |
| Time | vec(1) | vec(2) | vec(3) | msg |

---

### ekf_gps_file (odom_estimation_node.cpp)

| 列号 |              属性              |
| :--: | :----------------------------: |
|  1   |    ros::Time::now().toSec()    |
|  2   |   gps_meas_.getOrigin().x()    |
|  3   |   gps_meas_.getOrigin().y()    |
|  4   |   gps_meas_.getOrigin().z()    |
|  5   | gps_meas_.getBasis().getRPY(r) |
|  6   | gps_meas_.getBasis().getRPY(p) |
|  7   | gps_meas_.getBasis().getRPY(y) |

|  1   |  2  |  3  |  4  |  5  |  6  |  7  |
| :--: | :-: | :-: | :-: | :-: | :-: | :-: |
| Time |  x  |  y  |  z  |  r  |  p  |  y  |

---

### vo_meas_file (odom_estimation.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |        vo_rel(1)         |
|  3   |        vo_rel(2)         |
|  4   |        vo_rel(3)         |
|  5   |        vo_rel(4)         |
|  6   |        vo_rel(5)         |
|  7   |        vo_rel(6)         |
|  8   |           msg            |

msg=vo_covariance_(vo_covariance_.rows(),vo_covariance_.columns())

|  1   |   2    |   3    |   4    |   5    |   6    |   7    |  8  |
| :--: | :----: | :----: | :----: | :----: | :----: | :----: | :-: |
| Time | rel(1) | rel(2) | rel(3) | rel(4) | rel(5) | rel(6) | msg |

---

### ekf_vo_file (odom_estimation_node.cpp)

| 列号 |                 属性                |
| :--: | :---------------------------------: |
|  1   |      ros::Time::now().toSec()       |
|  2   |      vo_meas_.getOrigin().x()       |
|  3   |      vo_meas_.getOrigin().y()       |
|  4   |      vo_meas_.getOrigin().z()       |
|  5   | vo_meas_.getBasis().getEulerYPR(Rx) |
|  6   | vo_meas_.getBasis().getEulerYPR(Ry) |
|  7   | vo_meas_.getBasis().getEulerYPR(Rz) |

|  1   |  2  |  3  |  4  |  5  |  6  |  7  |
| :--: | :-: | :-: | :-: | :-: | :-: | :-: |
| Time |  x  |  y  |  z  |  r  |  p  |  y  |

---

### slam_meas_file (slam_gmapping.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |         mpose.x          |
|  3   |         mpose.y          |
|  4   |       mpose.theta        |

|  1   |  2  |  3  |   4   |
| :--: | :-: | :-: | :---: |
| Time |  x  |  y  | theta |

---

### slam2_meas_file (slam_gmapping.cpp)

| 列号 |           属性           |
| :--: | :----------------------: |
|  1   | ros::Time::now().toSec() |
|  2   |       gmap_pose.x        |
|  3   |       gmap_pose.y        |
|  4   |     gmap_pose.theta      |

|  1   |  2  |  3  |   4   |
| :--: | :-: | :-: | :---: |
| Time |  x  |  y  | theta |

---

### ekf_output_file (odom_estimation_node.cpp)

| 列号 |               属性               |
| :--: | :------------------------------: |
|  1   |     ros::Time::now().toSec()     |
|  2   | my_filter_.getEstimate(estimate) |

|  1   |    2     |
| :--: | :------: |
| Time | estimate |

---

