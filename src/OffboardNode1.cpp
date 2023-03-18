#include "OffboardWrapper.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "offb1_node");
  // ros::NodeHandle nh;
  geometry_msgs::PoseStamped g_position_setpoint0,g_position_setpoint1;
  g_position_setpoint0.pose.position.x = 0;
  g_position_setpoint0.pose.position.y = 0;
  g_position_setpoint0.pose.position.z = 1;
  g_position_setpoint1.pose.position.x = -2;
  g_position_setpoint1.pose.position.y = 1;
  g_position_setpoint1.pose.position.z = 1;
  
  // OffboardWrapper wrapper0(g_position_setpoint, "/uav0");
  // wrapper0.run();
  OffboardWrapper wrapper(g_position_setpoint1, "/uav1", "/offb1_node", "/home/zhoujin/time_optimal_trajectory/example/result4.csv");
  wrapper.run();

  return 0;
}

