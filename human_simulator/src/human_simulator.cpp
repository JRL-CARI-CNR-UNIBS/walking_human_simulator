/*
Copyright (c) 2023, Cesare Tonola, c.tonola001@unibs.it
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
*/


#include <ros/ros.h>
#include <human_simulator/HumanSimulatorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <random>

class HumanSimulatorAction
{
protected:
  enum Status{stop, move};

  ros::NodeHandle nh_;
  ros::Publisher poses_pub_;
  actionlib::SimpleActionServer<human_simulator::HumanSimulatorAction> as_;

  moveit_msgs::CollisionObject collision_object_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string base_frame_;

  geometry_msgs::Pose head_pose_;
  geometry_msgs::Pose chest_pose_;
  geometry_msgs::Pose left_arm_pose_;
  geometry_msgs::Pose right_arm_pose_;
  geometry_msgs::Pose left_hand_pose_;
  geometry_msgs::Pose right_hand_pose_;

  std::string action_name_;
  human_simulator::HumanSimulatorResult result_;
  human_simulator::HumanSimulatorFeedback feedback_;

  double noise_x_, noise_y_, noise_z_, rate_;
  std::vector<double> x_, y_, z_, pauses_duration_, motions_duration_;

public:

  HumanSimulatorAction(std::string name):
    as_(nh_, name, boost::bind(&HumanSimulatorAction::executeCB, this, _1), false),
    action_name_(name)
  {
    std::string poses_topic;
    if(not nh_.getParam("poses_topic",poses_topic))
    {
      poses_topic = "/poses";
      ROS_ERROR_STREAM("poses_topic set to "<<poses_topic);
    }

    poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(poses_topic, 1);

    if(not nh_.getParam("base_frame",base_frame_))
    {
      base_frame_ = "world";
      ROS_ERROR_STREAM("default base_frame "<<base_frame_);
    }

    if(not nh_.getParam("rate",rate_))
    {
      rate_ = 30.0;
      ROS_ERROR_STREAM("default rate_ "<<rate_);
    }
    else
    {
      if(rate_<=0.0)
      {
        rate_ = 30.0;
        ROS_ERROR_STREAM("default rate_ "<<rate_);
      }
    }

    if(not nh_.getParam("x",x_))
      throw std::invalid_argument("no x param defined");

    if(not nh_.getParam("y",y_))
      throw std::invalid_argument("no y param defined");

    if(not nh_.getParam("z",z_))
      throw std::invalid_argument("no z param defined");

    if(not nh_.getParam("noise_x",noise_x_))
    {
      ROS_ERROR_STREAM("noise_x set to 0");
      noise_x_ = 0.0;
    }

    if(not nh_.getParam("noise_y",noise_y_))
    {
      ROS_ERROR_STREAM("noise_y set to 0");
      noise_y_ = 0.0;
    }

    if(not nh_.getParam("noise_z",noise_z_))
    {
      ROS_ERROR_STREAM("noise_z set to 0");
      noise_z_ = 0.0;
    }

    if(not nh_.getParam("pauses_duration",pauses_duration_))
      throw std::invalid_argument("no pauses_duration param defined");

    if(not nh_.getParam("motions_duration",motions_duration_))
      throw std::invalid_argument("no motions_duration param defined");

    std::vector<double> chest_size;
    if(not nh_.getParam("chest_size",chest_size))
    {
      chest_size = {0.1,0.2,0.4};
      ROS_ERROR_STREAM("default chest_size");
    }
    else
    {
      if(chest_size.size() != 3)
      {
        throw std::invalid_argument("chest_size should be a vector of size 3");
      }
    }

    double head_size;
    if(not nh_.getParam("head_size",head_size))
    {
      head_size = 0.08;
      ROS_ERROR_STREAM("default arm_size");
    }

    std::vector<double> arm_size;
    if(not nh_.getParam("arm_size",arm_size))
    {
      arm_size = {0.35,0.06,0.06};
      ROS_ERROR_STREAM("default arm_size");
    }
    else
    {
      if(arm_size.size() != 3)
      {
        throw std::invalid_argument("arm_size should be a vector of size 3");
      }
    }

    if(x_.size() != y_.size() || x_.size() != z_.size() || y_.size() != z_.size())
      throw std::invalid_argument("x,y,z must have the same size");

    if((x_.size() != pauses_duration_.size()) || (motions_duration_.size() != (x_.size()-1)))
      throw std::invalid_argument("motions_duration and/or pauses_duration and/or x,y,z sizes are wrong");

    // PRIMITIVES
    shape_msgs::SolidPrimitive chest;
    chest.type = chest.BOX;
    chest.dimensions = chest_size;

    shape_msgs::SolidPrimitive head;
    head.type = head.SPHERE;
    head.dimensions.resize(1);
    head.dimensions[0] = head_size;

    shape_msgs::SolidPrimitive arm;
    arm.type = arm.BOX;
    arm.dimensions = arm_size;

    shape_msgs::SolidPrimitive hand;
    hand.type = hand.SPHERE;
    hand.dimensions.resize(1);
    hand.dimensions[0] = arm_size[2]/2.0;

    // POSES
    chest_pose_.position.x = 0.0;
    chest_pose_.position.y = 0.0;
    chest_pose_.position.z = 0.0;
    chest_pose_.orientation.w = 1.0;

    head_pose_.position.x = 0.0;
    head_pose_.position.y = 0.0;
    head_pose_.position.z = (chest_size[2]/2.0)+head_size;
    head_pose_.orientation.w = 1.0;

    right_arm_pose_.position.x = arm_size[0]/2-chest_size[0]/2;
    right_arm_pose_.position.y = -(chest_size[1]/2.0+arm_size[1]/2.0);
    right_arm_pose_.position.z = chest_size[2]/2-arm_size[2]/2.0;
    right_arm_pose_.orientation.w = 1.0;

    left_arm_pose_.position.x = arm_size[0]/2-chest_size[0]/2;
    left_arm_pose_.position.y = chest_size[1]/2.0+arm_size[1]/2.0;
    left_arm_pose_.position.z = chest_size[2]/2-arm_size[2]/2.0;
    left_arm_pose_.orientation.w = 1.0;

    left_hand_pose_.position.x = arm_size[0]-chest_size[0]/2;
    left_hand_pose_.position.y = chest_size[1]/2.0+arm_size[1]/2.0;
    left_hand_pose_.position.z = chest_size[2]/2-arm_size[2]/2.0;
    left_hand_pose_.orientation.w = 1.0;

    right_hand_pose_.position.x = arm_size[0]-chest_size[0]/2;
    right_hand_pose_.position.y = -(chest_size[1]/2.0+arm_size[1]/2.0);
    right_hand_pose_.position.z = chest_size[2]/2-arm_size[2]/2.0;
    right_hand_pose_.orientation.w = 1.0;

    // COLLISION OBJECT
    collision_object_.header.frame_id=base_frame_;
    collision_object_.header.stamp=ros::Time::now();

    collision_object_.id = "human_simulator";

    collision_object_.pose.orientation.x = 0.0;
    collision_object_.pose.orientation.y = 0.0;
    collision_object_.pose.orientation.z = 0.0;
    collision_object_.pose.orientation.w = 1.0;

    collision_object_.primitive_poses.push_back(chest_pose_);
    collision_object_.primitive_poses.push_back(head_pose_);
    collision_object_.primitive_poses.push_back(left_arm_pose_);
    collision_object_.primitive_poses.push_back(right_arm_pose_);
    collision_object_.primitive_poses.push_back(left_hand_pose_);
    collision_object_.primitive_poses.push_back(right_hand_pose_);

    collision_object_.primitives.push_back(chest);
    collision_object_.primitives.push_back(head);
    collision_object_.primitives.push_back(arm);
    collision_object_.primitives.push_back(arm);
    collision_object_.primitives.push_back(hand);
    collision_object_.primitives.push_back(hand);

    collision_object_.meshes        .clear(); //remove the warnings
    collision_object_.planes        .clear(); //remove the warnings
    collision_object_.mesh_poses    .clear(); //remove the warnings
    collision_object_.plane_poses   .clear(); //remove the warnings
    collision_object_.subframe_names.clear(); //remove the warnings
    collision_object_.subframe_poses.clear(); //remove the warnings

    as_.start();
  }

  ~HumanSimulatorAction(void)
  {
  }

  void executeCB(const human_simulator::HumanSimulatorGoalConstPtr &goal)
  {
    uint max_loops;
    if(goal->queried_loops<-1)
    {
      ROS_INFO("%s: Aborted, goal wrong", action_name_.c_str());

      result_.executed_loops = 0;
      result_.result = result_.GoalWrong;
      as_.setAborted(result_);
      return;
    }

    if(goal->queried_loops == -1)
      max_loops = std::numeric_limits<int>::max();
    else
      max_loops = goal->queried_loops;

    ROS_INFO("%s: Goal accepted", action_name_.c_str());

    double t = 0.0;
    double dt = 1.0/(double)rate_;
    double pause_duration = 0.0;
    double motion_duration = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> noise_x(-noise_x_,noise_x_);
    std::uniform_real_distribution<double> noise_y(-noise_y_,noise_y_);
    std::uniform_real_distribution<double> noise_z(-noise_z_,noise_z_);

    ros::WallRate lp(rate_);
    Eigen::Vector3d current_location, next_location, velocity;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object = collision_object_;
    std::vector<double>::iterator it_pauses, it_motions, it_x, it_y, it_z;

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = base_frame_;
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
    geometry_msgs::Pose pose;
    pose.orientation = q;

    uint loops = 0;
    bool new_loop = true;
    Status status = stop;

    bool success = true;

    ros::WallTime tic, toc;

    while(loops<max_loops)
    {
      tic =ros::WallTime::now();

      if (as_.isPreemptRequested() || not ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());

        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      t += dt;

      feedback_.current_loop = loops;
      feedback_.current_location = {current_location[0],current_location[1],current_location[2]};
      if(status == stop)
        feedback_.status = feedback_.Stop;
      if(status == move)
        feedback_.status = feedback_.Move;

      as_.publishFeedback(feedback_);

      if(new_loop)
      {
        it_motions = motions_duration_.begin();
        it_pauses  = pauses_duration_ .begin();
        it_x = x_.begin();
        it_y = y_.begin();
        it_z = z_.begin();

        current_location<<*it_x,*it_y,*it_z;
        it_x++; it_y++; it_z++;

        collision_object.pose.position.x = current_location[0];
        collision_object.pose.position.y = current_location[1];
        collision_object.pose.position.z = current_location[2];

        collision_objects.clear();

        if(loops == 0)
          collision_object.operation = moveit_msgs::CollisionObject::ADD;
        else
          collision_object.operation = moveit_msgs::CollisionObject::MOVE;

        collision_objects.push_back(collision_object);
        planning_scene_interface_.addCollisionObjects(collision_objects);

        next_location<<*it_x,*it_y,*it_z;
        it_x++; it_y++; it_z++;

        pause_duration  = *it_pauses;
        it_pauses ++;

        if(not motions_duration_.empty())
        {
          motion_duration = *it_motions;
          velocity = (next_location-current_location)/(motion_duration);
          it_motions++;
        }

        t = 0.0;
        status = stop;
        new_loop = false;
      }
      else
      {
        if(status == stop)
        {
          if(t>pause_duration)
          {
            if(it_pauses == pauses_duration_.end())
            {
              loops++;
              new_loop = true;
            }
            else
            {
              pause_duration = *it_pauses;
              it_pauses++;
            }

            if(not motions_duration_.empty())
              status = move;

            t = 0.0;
            continue;
          }
          else //add noise to current position during pause
          {
            collision_object.pose.position.x = current_location[0]+noise_x(gen);
            collision_object.pose.position.y = current_location[1]+noise_y(gen);
            collision_object.pose.position.z = current_location[2]+noise_z(gen);

            collision_object.operation = moveit_msgs::CollisionObject::MOVE;

            collision_object.primitives     .clear(); //remove the warnings
            collision_object.primitive_poses.clear(); //remove the warnings

            collision_objects.clear();
            collision_objects.push_back(collision_object);
            planning_scene_interface_.applyCollisionObjects(collision_objects);
          }
        }

        if(status == move)
        {
          if(t>motion_duration)
          {
            if(it_motions < motions_duration_.end())
            {
              motion_duration = *it_motions;
              it_motions++;

              next_location<<*it_x,*it_y,*it_z;
              it_x++; it_y++; it_z++;

              velocity = (next_location-current_location)/(motion_duration);
            }

            status = stop;
            t = 0.0;
            continue;
          }
          else
          {
            current_location = current_location + dt*velocity;

            collision_object.pose.position.x = current_location[0];
            collision_object.pose.position.y = current_location[1];
            collision_object.pose.position.z = current_location[2];

            collision_object.operation = moveit_msgs::CollisionObject::MOVE;

            collision_object.primitives     .clear(); //remove the warnings
            collision_object.primitive_poses.clear(); //remove the warnings

            collision_objects.clear();
            collision_objects.push_back(collision_object);
            planning_scene_interface_.applyCollisionObjects(collision_objects);
          }
        }
      }

      // publish current poses (e.g. for SSM module)
      pose_array.header.stamp = ros::Time::now();
      pose_array.poses.clear();

      // head
      pose.position.x = collision_object.pose.position.x+head_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+head_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+head_pose_.position.z;
      pose_array.poses.push_back(pose);

      // chest
      pose.position.x = collision_object.pose.position.x+chest_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+chest_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+chest_pose_.position.z;
      pose_array.poses.push_back(pose);

      // left arm
      pose.position.x = collision_object.pose.position.x+left_arm_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+left_arm_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+left_arm_pose_.position.z;
      pose_array.poses.push_back(pose);

      // right arm
      pose.position.x = collision_object.pose.position.x+right_arm_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+right_arm_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+right_arm_pose_.position.z;
      pose_array.poses.push_back(pose);

      // left hand
      pose.position.x = collision_object.pose.position.x+left_hand_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+left_hand_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+left_hand_pose_.position.z;
      pose_array.poses.push_back(pose);

      // right hand
      pose.position.x = collision_object.pose.position.x+right_hand_pose_.position.x;
      pose.position.y = collision_object.pose.position.y+right_hand_pose_.position.y;
      pose.position.z = collision_object.pose.position.z+right_hand_pose_.position.z;
      pose_array.poses.push_back(pose);

      poses_pub_.publish(pose_array);

      toc =ros::WallTime::now();
      if((toc-tic).toSec()>2.0*dt)
        ROS_WARN_STREAM("slow loop, duration -> "<<(toc-tic).toSec()<<" seconds");

      lp.sleep();
    }

    collision_object.operation = moveit_msgs::CollisionObject::REMOVE;

    std::vector<std::string> objects_ids;
    objects_ids.push_back(collision_object.id);
    planning_scene_interface_.removeCollisionObjects(objects_ids);

    // send the result
    result_.executed_loops = loops;
    result_.final_location = {current_location[0],current_location[1],current_location[2]};

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());

      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
      ROS_INFO("%s: Aborted", action_name_.c_str());

      // set the action state to aborted
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_simulator");

  HumanSimulatorAction hs("human_simulator");
  ros::spin();

  return 0;
}
