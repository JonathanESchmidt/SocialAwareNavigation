cmake_minimum_required(VERSION 2.8.3)
project(neo_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 ##Generate messages in the 'msg' folder
 add_message_files(
   FILES
    	EmergencyStopState.msg
	GyroBoard.msg
	IOAnalogIn.msg
	IOOut.msg
	IRSensors.msg
	Keypad.msg
	LCDOutput.msg
	RadarBoard.msg
	USBoard.msg
	PowerState.msg
	PowerBoardState.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)


