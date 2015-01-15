/**
 * \file  test_door_detector.cpp
 * \brief  A small tester for the door state detector
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 05/06/2013 02:05:01 PM piyushk $
 *
 **/

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multi_level_map_msgs/ChangeCurrentLevel.h>
#include <multi_level_map_msgs/LevelMetaData.h>
#include <multi_level_map_msgs/MultiLevelMapData.h>
#include <multi_level_map_utils/utils.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <segbot_logical_translator/LogicalNavigationAction.h>
#include <segbot_logical_translator/segbot_logical_translator.h>

using bwi_planning_common::PlannerAtom;
using bwi_planning_common::NO_DOOR_IDX;

class SegbotLogicalNavigator : public segbot_logical_translator::SegbotLogicalTranslator {

  public:

    typedef actionlib::SimpleActionServer<segbot_logical_translator::LogicalNavigationAction> LogicalNavActionServer;

    SegbotLogicalNavigator();
    void execute(const segbot_logical_translator::LogicalNavigationGoalConstPtr &goal);

  protected:

    void senseState(std::vector<PlannerAtom>& observations, 
        size_t door_idx = NO_DOOR_IDX);
    bool approachDoor(const std::string& door_name, 
        std::vector<PlannerAtom>& observations,
        std::string& error_message, bool gothrough = false);
    bool senseDoor(const std::string& door_name, 
        std::vector<PlannerAtom>& observations,
        std::string& error_message);
    
    bool approachObject(const std::string& object_name,
        std::vector<PlannerAtom>& observations,
        std::string& error_message);

    bool changeFloor(const std::string& floor_name,
        const std::string& new_start_loc,
        std::vector<PlannerAtom>& observations,
        std::string& error_message);

    bool executeNavigationGoal(const geometry_msgs::PoseStamped& pose);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

    void currentLevelHandler(const multi_level_map_msgs::LevelMetaData::ConstPtr& current_level);
    void multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap);

    float robot_x_;
    float robot_y_;
    float robot_yaw_;
    std::string current_level_id_;

    double door_proximity_distance_;

    boost::shared_ptr<LogicalNavActionServer> execute_action_server_; 
    bool execute_action_server_started_;

    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > robot_controller_;

    boost::shared_ptr<tf::TransformListener> tf_;
    boost::shared_ptr<tf::MessageFilter<nav_msgs::Odometry> > tf_filter_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > odom_subscriber_;

    ros::Subscriber current_level_subscriber_;
    ros::Subscriber multimap_subscriber_;
    ros::ServiceClient change_level_client_;
    bool change_level_client_available_;
    std::vector<multi_level_map_msgs::LevelMetaData> all_levels_;
    std::map<std::string, std::map<std::string, geometry_msgs::Pose> > level_to_objects_map_;

};

SegbotLogicalNavigator::SegbotLogicalNavigator() : 
    robot_x_(0), robot_y_(0), robot_yaw_(0), current_level_id_(""), execute_action_server_started_(false) {

  ROS_INFO("SegbotLogicalNavigator: Advertising services!");

  ros::param::param("~door_proximity_distance", door_proximity_distance_, 2.0);

  robot_controller_.reset(
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
        "move_base", true));
  robot_controller_->waitForServer();
  
  tf_.reset(new tf::TransformListener);
  odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>);
  odom_subscriber_->subscribe(*nh_, "odom", 5);
  tf_filter_.reset(new tf::MessageFilter<nav_msgs::Odometry>(
        *odom_subscriber_, *tf_, global_frame_id_, 5));
  tf_filter_->registerCallback(
      boost::bind(&SegbotLogicalNavigator::odometryHandler, this, _1));

  execute_action_server_.reset(new LogicalNavActionServer(*nh_,
                                                          "execute_logical_goal",
                                                          boost::bind(&SegbotLogicalNavigator::execute, this, _1),
                                                          false));

  current_level_subscriber_ = nh_->subscribe("level_mux/current_level",
                                             1, 
                                             &SegbotLogicalNavigator::currentLevelHandler,
                                             this);

  multimap_subscriber_ = nh_->subscribe("map_metadata",
                                        1, 
                                        &SegbotLogicalNavigator::multimapHandler,
                                        this);

}

void SegbotLogicalNavigator::currentLevelHandler(const multi_level_map_msgs::LevelMetaData::ConstPtr& current_level) {
  if (current_level_id_ != current_level->level_id) {
    current_level_id_ = current_level->level_id;
    ros::param::set("~map_file", current_level->map_file);
    ros::param::set("~data_directory", current_level->data_directory);
    SegbotLogicalTranslator::initialize();
  }
}

void SegbotLogicalNavigator::multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap) {

  // Read in the objects for each level.
  BOOST_FOREACH(const multi_level_map_msgs::LevelMetaData &level, multimap->levels) {
    std::string objects_file = level.data_directory + "/objects.yaml";
    bwi_planning_common::readObjectApproachFile(objects_file, level_to_objects_map_[level.level_id]);
  }

  // Start the change level service client.
  if (!change_level_client_available_) {
    bool change_level_client_available_ = ros::service::waitForService("level_mux/change_current_level", ros::Duration(5.0));
    if (change_level_client_available_) {
      change_level_client_ = nh_->serviceClient<multi_level_map_msgs::ChangeCurrentLevel>("level_mux/change_current_level");
    }
  }
  
}

void SegbotLogicalNavigator::senseState(std::vector<PlannerAtom>& observations, size_t door_idx) {

  PlannerAtom at;
  at.name = "at";
  size_t location_idx = getLocationIdx(bwi::Point2f(robot_x_, robot_y_));
  at.value.push_back(getLocationString(location_idx));
  observations.push_back(at);

  size_t num_doors = getNumDoors();
  bwi::Point2f robot_loc(robot_x_, robot_y_);
  bool first_facing = false;
  bool first_beside = false;
  size_t facing_idx = NO_DOOR_IDX;

  for (size_t door = 0; door < num_doors; ++door) {
    if (door_idx != NO_DOOR_IDX && door != door_idx) {
      continue;
    }
    bool facing_door = !first_facing && 
      isRobotFacingDoor(robot_loc, robot_yaw_, door_proximity_distance_, door);
    bool beside_door = !first_beside && 
      isRobotBesideDoor(robot_loc, robot_yaw_, door_proximity_distance_, door);
    if (!facing_door) {
      PlannerAtom n_facing;
      n_facing.name = "-facing";
      n_facing.value.push_back(getDoorString(door));
      observations.push_back(n_facing);
    } else {
      PlannerAtom facing;
      facing.name = "facing";
      facing.value.push_back(getDoorString(door));
      observations.push_back(facing);
      first_facing = true;
      facing_idx = door_idx;
    }
    if (!beside_door) {
      PlannerAtom n_beside;
      n_beside.name = "-beside";
      n_beside.value.push_back(getDoorString(door));
      observations.push_back(n_beside);
    } else {
      PlannerAtom beside;
      beside.name = "beside";
      beside.value.push_back(getDoorString(door));
      observations.push_back(beside);
      first_beside = true;
    }
  }

  // If we are facing a door, also sense whether it is open
  if (facing_idx != NO_DOOR_IDX) {
    PlannerAtom door_open;
    door_open.value.push_back(getDoorString(facing_idx));
    if (isDoorOpen(facing_idx)) {
      door_open.name = "open";
      observations.push_back(door_open);
    } else {
      door_open.name = "-open";
      observations.push_back(door_open);
    }
  }
}

bool SegbotLogicalNavigator::executeNavigationGoal(
    const geometry_msgs::PoseStamped& pose) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  robot_controller_->sendGoal(goal);
  bool navigation_request_complete = false;
  while (!navigation_request_complete) {
    if (execute_action_server_->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("SegbotLogicalNavigator: Got pre-empted. Cancelling low level navigation task...");
      robot_controller_->cancelGoal();
      break;
    }
    navigation_request_complete = robot_controller_->waitForResult(ros::Duration(0.5));
  }

  if (navigation_request_complete) {
    actionlib::SimpleClientGoalState state = robot_controller_->getState();
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  // If we're here, then we preempted the request ourselves. Let's mark our current request as not successful.
  return false;
}

void SegbotLogicalNavigator::odometryHandler(
    const nav_msgs::Odometry::ConstPtr& odom) {
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = odom->header;
  pose_in.pose = odom->pose.pose;
  tf_->transformPose(global_frame_id_, pose_in, pose_out);
  robot_x_ = pose_out.pose.position.x;
  robot_y_ = pose_out.pose.position.y;
  //ROS_INFO("OdometryHandler X:%f Y:%f" , robot_x_, robot_y_);
  
  robot_yaw_ = tf::getYaw(pose_out.pose.orientation);
  
  if(!execute_action_server_started_) {
    execute_action_server_->start();
    execute_action_server_started_ = true;
  }
}

bool SegbotLogicalNavigator::approachDoor(const std::string& door_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message, bool gothrough) {
  error_message = "";

  size_t door_idx = getDoorIdx(door_name);
  if (door_idx == NO_DOOR_IDX) {
    // Interface failure
    error_message = "Could not resolve argument: " + door_name;
    return false;
  } else {

    bwi::Point2f approach_pt;
    float approach_yaw = 0;
    bool door_approachable = false;

    if (!gothrough) {
      door_approachable = getApproachPoint(door_idx, 
          bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);
    } else {
      door_approachable = getThroughDoorPoint(door_idx,
          bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);
    }
    
    if (door_approachable) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = global_frame_id_;
      pose.pose.position.x = approach_pt.x;
      pose.pose.position.y = approach_pt.y;
      tf::quaternionTFToMsg(
          tf::createQuaternionFromYaw(approach_yaw), pose.pose.orientation); 
      bool success = executeNavigationGoal(pose);

      // Publish the observable fluents
      senseState(observations, door_idx);

      return success;
    } else {
      // Planning failure
      error_message = "Cannot interact with " + door_name + " from here.";
      return false;
    }
  }
}

bool SegbotLogicalNavigator::approachObject(const std::string& object_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message) {

  error_message = "";
  observations.clear();

  if (object_approach_map_.find(object_name) != object_approach_map_.end()) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = global_frame_id_;
    pose.pose = object_approach_map_[object_name];
    bool success = executeNavigationGoal(pose);

    // Publish the observable fluents
    senseState(observations, NO_DOOR_IDX);
    PlannerAtom closeto;
    closeto.name = "closeto";
    closeto.value.push_back(object_name);
    if (!success) {
      closeto.name = "-closeto";
    }
    observations.push_back(closeto);
    return success;
  }

  error_message = object_name + " does not exist.";
  return false;
}

bool SegbotLogicalNavigator::changeFloor(const std::string& floor_name,
                                         const std::string& new_start_loc,
                                         std::vector<PlannerAtom>& observations,
                                         std::string& error_message) {

  error_message = "";
  observations.clear();

  // Make sure we can change floors and all arguments are correct.
  if (change_level_client_available_) {
    error_message = "SegbotLogicalNavigator has not received the multimap. Cannot change floors!";
    return false;
  } else if (current_level_id_ == floor_name) {
    error_message = "The robot is already on " + floor_name + "!";
    return false;
  } else if (level_to_objects_map_.find(floor_name) == level_to_objects_map_.end()) {
    error_message = "Floor " + floor_name + " does not exist!";
    return false;
  } else if (level_to_objects_map_[floor_name].find(new_start_loc) == level_to_objects_map_[floor_name].end()) {
    error_message = "Location " + new_start_loc + " on floor " + floor_name + " has not been defined as a valid object!";
    return false;
  }

  multi_level_map_msgs::ChangeCurrentLevel srv;
  srv.request.new_level_id = floor_name;
  srv.request.publish_initial_pose = true;
  srv.request.initial_pose.header.stamp = ros::Time::now();
  srv.request.initial_pose.header.frame_id = multi_level_map::frameIdFromLevelId(floor_name);
  srv.request.initial_pose.pose.pose = level_to_objects_map_[floor_name][new_start_loc];
  srv.request.initial_pose.pose.covariance[0] = 1.0f;
  srv.request.initial_pose.pose.covariance[7] = 1.0f;
  srv.request.initial_pose.pose.covariance[35] = 1.0f;
  
  if (change_level_client_.call(srv)) {
    if (!srv.response.success) {
      error_message = srv.response.error_message;
      return false;
    }

    // Yea! the call succeeded! Wait for current_level_id_ to be changed once everything gets updated.
    ros::Rate r(1);
    while (current_level_id_ != floor_name) r.sleep();

    // Publish the observable fluents
    senseState(observations);
    return true;
  } else {
    error_message = "ChangeCurrentLevel service call failed for unknown reason.";
    return false;
  }
}

bool SegbotLogicalNavigator::senseDoor(const std::string& door_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message) {
  error_message = "";
  size_t door_idx = 
    bwi_planning_common::resolveDoor(door_name, doors_);
  if (door_idx == bwi_planning_common::NO_DOOR_IDX) {
    error_message = "Door " + door_name + " does not exist!";
    return false;
  } 
  bool door_open = isDoorOpen(door_idx);
  PlannerAtom open;
  open.name = "open";
  if (!door_open) {
    open.name = "-" + open.name;
  }
  open.value.push_back(door_name);
  observations.push_back(open);
  return true;
}

void SegbotLogicalNavigator::execute(const segbot_logical_translator::LogicalNavigationGoalConstPtr& goal) {

  segbot_logical_translator::LogicalNavigationResult res;
  res.observations.clear();

  if (goal->command.name == "approach") {
    res.success = approachDoor(goal->command.value[0], res.observations, res.status, false);
  } else if (goal->command.name == "gothrough") {
    res.success = approachDoor(goal->command.value[0], res.observations,
        res.status, true);
  } else if (goal->command.name == "sensedoor") {
    res.success = senseDoor(goal->command.value[0], res.observations,
        res.status);
  } else if (goal->command.name == "goto") {
    res.success = approachObject(goal->command.value[0], res.observations,
        res.status);
  } else if (goal->command.name == "changefloor") {
    res.success = changeFloor(goal->command.value[0], goal->command.value[0], res.observations, res.status);
  } else {
    res.success = true;
    res.status = "";
    senseState(res.observations);
  }

  if (res.success) {
    execute_action_server_->setSucceeded(res);
  } else if (execute_action_server_->isPreemptRequested()) {
    execute_action_server_->setPreempted(res);
  } else {
    execute_action_server_->setAborted(res);
  }

}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "segbot_logical_translator");
  ros::NodeHandle nh;

  ROS_INFO("SegbotLogicalNavigator: Starting up node...");
  SegbotLogicalNavigator handler;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  ROS_INFO("SegbotLogicalNavigator: Stopping node.");

  return 0;
}
