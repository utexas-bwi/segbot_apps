/**
 * \file  segbot_logical_translator.cpp
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
 * $ Id: 12/14/2013 04:35:22 PM piyushk $
 *
 **/

#include <tf/transform_datatypes.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/point_utils.h>

#include <segbot_logical_translator/segbot_logical_translator.h>

namespace segbot_logical_translator {

  SegbotLogicalTranslator::SegbotLogicalTranslator() {
    ROS_INFO_STREAM("SegbotLogicalTranslator: Initializing...");
    nh_.reset(new ros::NodeHandle);
    
    ROS_INFO_STREAM("SegbotLogicalTranslator: Waiting for make_plan service..");
    make_plan_client_ = 
      nh_->serviceClient<nav_msgs::GetPlan>("move_base/make_plan"); 
    ROS_INFO_STREAM("SegbotLogicalTranslator: make_plan service found!");
    make_plan_client_.waitForExistence();
    ros::param::param<std::string>(
        "~global_frame_id", global_frame_id_, "/map");
    initialize();
  }

  void SegbotLogicalTranslator::initialize() {
    ROS_INFO_STREAM("SegbotLogicalTranslator: RE-Initializing...");

    std::string map_file, door_file, location_file;
    std::vector<std::string> required_parameters;
    if (!ros::param::get("~map_file", map_file)) {
      required_parameters.push_back("~map_file");
    }
    if (!ros::param::get("~door_file", door_file)) {
      required_parameters.push_back("~door_file");
    }
    if (!ros::param::get("~location_file", location_file)) {
      required_parameters.push_back("~location_file");
      ROS_INFO_STREAM("SegbotLogicalTranslator: changed location file");
    }
    if (required_parameters.size() != 0) {
      std::string message = "SegbotLogicalTranslator: Required parameters [" + 
        boost::algorithm::join(required_parameters, ", ") + "] missing!";
      ROS_FATAL_STREAM(message);
      throw std::runtime_error(message);
    }

    std::string object_file;
    if (ros::param::get("~object_file", object_file)) {
      ROS_INFO_STREAM("Reading in object file from: " << object_file);
      bwi_planning_common::readObjectApproachFile( object_file,
          object_approach_map_);
    }
    bwi_planning_common::readDoorFile(door_file, doors_);
    bwi_planning_common::readLocationFile(location_file, 
        locations_, location_map_);
    mapper_.reset(new bwi_mapper::MapLoader(map_file));
    nav_msgs::OccupancyGrid grid;
    mapper_->getMap(grid);
    info_ = grid.info;

    ROS_INFO("Map_statistics  Height:%d Width:%d" , info_.height, info_.width);
    ROS_INFO("Map_statistics  LocationsArraySize:%d" , (int)location_map_.size());
  }

  bool SegbotLogicalTranslator::isDoorOpen(size_t idx) {

    if (idx > doors_.size()) {
      return false;
    }

    bwi_mapper::Point2f start_pt, goal_pt;
    float start_yaw, goal_yaw;

    start_pt = doors_[idx].approach_points[0];
    goal_pt = doors_[idx].approach_points[1];
    start_yaw = doors_[idx].approach_yaw[0];
    goal_yaw = doors_[idx].approach_yaw[1];

    nav_msgs::GetPlan srv;
    geometry_msgs::PoseStamped &start = srv.request.start;
    geometry_msgs::PoseStamped &goal = srv.request.goal;
    start.header.frame_id = goal.header.frame_id = global_frame_id_;
    start.header.stamp = goal.header.stamp = ros::Time::now();

    start.pose.position.x = start_pt.x;
    start.pose.position.y = start_pt.y;
    start.pose.position.z = 0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw); 

    goal.pose.position.x = goal_pt.x;
    goal.pose.position.y = goal_pt.y;
    goal.pose.position.z = 0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
    srv.request.tolerance = 0.5 + 1e-6;

    float min_distance = cv::norm(start_pt - goal_pt);

    int counter = 0;

    for( int  i = 0; i < 3 ; i++){
        if (make_plan_client_.call(srv)) {
          if (srv.response.plan.poses.size() != 0) {
            // Valid plan received. Check if plan distance seems reasonable
            float distance = 0;
            geometry_msgs::Point old_pt = 
              srv.response.plan.poses[0].pose.position;
            for (size_t i = 1; i < srv.response.plan.poses.size(); ++i) {
              geometry_msgs::Point current_pt = 
                srv.response.plan.poses[i].pose.position;
              distance += sqrt(pow(current_pt.x - old_pt.x, 2) + 
                               pow(current_pt.y - old_pt.y, 2));
              old_pt = current_pt;
            }
            if (distance < 3 * min_distance) {
              //return true;
              counter++;
            } else {
               //return false; // returned path probably through some other door
               counter = 0;
            }
          } else {
               //return false; // this is ok. it means the door is closed
               counter = 0;
          }
        } else {
              //return false; // shouldn't be here. the service has failed
              counter = 0;
        }
    }
    if (counter == 3){
        // we have see the door open 3 consequitive times
        return true;
    }else{
        return false;
    }
  }



  bool SegbotLogicalTranslator::getApproachPoint(size_t idx, 
      const bwi_mapper::Point2f& current_location,
      bwi_mapper::Point2f& point, float &yaw) {

    if (idx > doors_.size()) {
      return false;
    }

    for (size_t pt = 0; pt < 2; ++pt) {
      std::vector<std::string> approach_locations;
      boost::split(approach_locations, doors_[idx].approach_names[pt], 
          boost::is_any_of(","), boost::token_compress_on);
      BOOST_FOREACH(const std::string& location, approach_locations) {
        if (getLocationIdx(location) == 
            getLocationIdx(current_location)) {
          point = doors_[idx].approach_points[pt];
          yaw = doors_[idx].approach_yaw[pt];
          return true;
        }
      }
    }

    /* The door is not approachable from the current location */
    return false;
  }

  bool SegbotLogicalTranslator::getThroughDoorPoint(size_t idx, 
      const bwi_mapper::Point2f& current_location,
      bwi_mapper::Point2f& point, float& yaw) {

    if (idx > doors_.size()) {
      return false;
    }

    for (size_t pt = 0; pt < 2; ++pt) {
      std::vector<std::string> approach_locations;
      boost::split(approach_locations, doors_[idx].approach_names[pt], 
          boost::is_any_of(","), boost::token_compress_on);
      BOOST_FOREACH(const std::string& location, approach_locations) {
        if (getLocationIdx(location) == 
            getLocationIdx(current_location)) {
          point = doors_[idx].approach_points[1 - pt];
          yaw = M_PI + doors_[idx].approach_yaw[1 - pt];
          while (yaw > M_PI) yaw -= 2 * M_PI;
          while (yaw <= M_PI) yaw += 2 * M_PI;
          return true;
        }
      }
    }

    return false;
  }

  bool SegbotLogicalTranslator::isRobotFacingDoor(
      const bwi_mapper::Point2f& current_location,
      float yaw, float threshold, size_t idx) {

    bwi_mapper::Point2f center_pt = 0.5 * 
      (doors_[idx].approach_points[0] + doors_[idx].approach_points[1]);
    if (bwi_mapper::getMagnitude(center_pt - current_location) >
        threshold) {
      return false;
    }

    bwi_mapper::Point2f diff_pt = center_pt - current_location;
    float orientation_to_door = atan2f(diff_pt.y, diff_pt.x);
    while (orientation_to_door > yaw + M_PI) orientation_to_door -= 2*M_PI;
    while (orientation_to_door <= yaw - M_PI) orientation_to_door += 2*M_PI;
    if (fabs(orientation_to_door - yaw) > M_PI / 3) {
      return false;
    }

    return true;
  }

  bool SegbotLogicalTranslator::isRobotBesideDoor(
      const bwi_mapper::Point2f& current_location,
      float yaw, float threshold, size_t idx) {

    bwi_mapper::Point2f center_pt = 0.5 * 
      (doors_[idx].approach_points[0] + doors_[idx].approach_points[1]);
    if (bwi_mapper::getMagnitude(center_pt - current_location) >
        threshold) {
      return false;
    }

    return true;
  }

  size_t SegbotLogicalTranslator::getLocationIdx(
      const bwi_mapper::Point2f& current_location) {

    bwi_mapper::Point2f grid = bwi_mapper::toGrid(current_location, info_);
    size_t map_idx = MAP_IDX(info_.width, (int) grid.x, (int) grid.y);
    if (map_idx > location_map_.size()) {
      return (size_t) -1;
    }
    return (size_t) location_map_[map_idx];

  }

} /* namespace segbot_logical_translator */
