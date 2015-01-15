#include <move_base_msgs/MoveBaseAction.h>
#include <bwi_interruptable_action_server/interruptable_action_server.h>
#include <ros/ros.h>

using namespace bwi_interruptable_action_server;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "move_base_interruptable_server");
  ros::NodeHandle nh;

  InterruptableActionServer<move_base_msgs::MoveBaseAction> as(nh, "move_base");
  as.spin();
  
  return 0;
}
