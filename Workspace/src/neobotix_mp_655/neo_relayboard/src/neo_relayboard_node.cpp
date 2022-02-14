#include <../include/neo_relayboard_node.h>

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "neo_relayboard_node");
	neo_relayboard_node node;
	if(node.init() != 0) return 1;
	double requestRate = node.getRequestRate();
	ros::Rate r(requestRate); //Cycle-Rate: Frequency of publishing States
	while(node.n.ok())
	{
		node.requestBoardStatus();
		node.sendEmergencyStopStates();
		node.sendAnalogIn();
		//node.sendRelayBoardDigOut(); // Neobotix should go ahead and die for this...
 		node.sendDriveStates();
		node.sendGyroBoard();
		node.sendRadarBoard();
		node.sendUSBoard();
		node.sendIOBoardDigIn();
		node.sendIOBoardDigOut();
		node.sendIOBoardAnalogIn();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

