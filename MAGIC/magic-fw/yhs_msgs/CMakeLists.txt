cmake_minimum_required(VERSION 3.0.2)
project(yhs_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)


 add_message_files(
   FILES
   FwBmsFb.msg
   FwBmsFlagFb.msg
   FwCtrlCmd.msg
   FwCtrlFb.msg
   FwIoCmd.msg
   FwIoFb.msg
   FwChassisInfoFb.msg
   DgtCtrlFb.msg
   DgtIoFb.msg
   DgtBmsFlagFb.msg
   DgtBmsFb.msg
   DgtCtrlCmd.msg
   DgtIoCmd.msg
   DgtChassisInfoFb.msg
   FrBmsFlagInfoFb.msg
   FrBmsInfoFb.msg
   FrCtrlCmd.msg
   FrCtrlFb.msg
   FrIoCmd.msg
   FrIoFb.msg
   FrVehDiagFb.msg
   FrChassisInfoFb.msg
   RobotPosition.msg
   RobotHardwreStatus.msg
   Version.msg
   SetCostmaps.msg
   DynamicParams.msg
   Ultrasonic.msg
   SetRegion.msg
 )

 add_service_files(
   FILES
   InitialPose.srv
   Pause.srv
   Goal.srv
   Cancel.srv
   PathIsValid.srv
   Recharge.srv
   DisRecharge.srv
 )
 
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs nav_msgs
 )


catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation move_base_msgs nav_msgs pcl_msgs sensor_msgs std_msgs tf
)

