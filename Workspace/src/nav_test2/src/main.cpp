#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Odometry.h>

ros::Time timeNow;
float lowestDistance;
int counter = 0;
float xO = 0.0f, yO = 0.0f, zO = 0.0f;
float vel = 0.1f; //0.1f

//void callback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //float x,y,z, xDelta, yDelta, zDelta;
  //x = msg->feedback.base_position.pose.position.x;
  //y = msg->feedback.base_position.pose.position.y;
  //z = msg->feedback.base_position.pose.position.z;

  float x,y,z, xDelta, yDelta, zDelta;
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;

  ros::Time time = msg -> header.stamp;
  ros::Duration delta = time - timeNow;
  float deltaFloat = delta.toSec();
  timeNow = msg -> header.stamp;
  std::cout<<"\nDelta is "<<delta<<std::endl;
  std::cout<<"DeltaFloat is "<<deltaFloat<<std::endl;

  if(counter==0){
    std::cout<<"Starting... Using 0,0,0 for the obstacle coordinates"<<std::endl;
  } else {
    xO = xO + vel*deltaFloat;
    std::cout<<"xO is "<<xO<<std::endl;
  }

  xDelta = fabs (xO - x);
  yDelta = fabs (yO - y);
  zDelta = fabs (zO - z);
  float distance = sqrt(pow(xDelta,2.0f) + pow(yDelta,2.0f));
  std::cout << "Distance is " << distance << std::endl;


  if(counter==0){
    counter++;
    lowestDistance = distance;
  } else {
    if (distance < lowestDistance) lowestDistance = distance;
  }
  std::cout << "Lowest distance calculated is " << lowestDistance << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "nav_test");
  ros::NodeHandle n;
  timeNow = ros::Time::now();
  //ros::Subscriber sub = n.subscribe("move_base/feedback", 1000, callback);
  ros::Subscriber sub = n.subscribe("odom", 1000, callback);
  ros::spin();
return 0;
}
