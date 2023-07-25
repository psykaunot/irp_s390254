//Author: Yanis DIALLO
//Date: 29/06/2023
//Contact: yanis.diallo.254@cranfield.ac.uk
//Objective: Sending a robot to precise location using the move_base library in ROS

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>  //Action library
#include <move_base_msgs/MoveBaseAction.h>          //Move base library
#include "math.h" 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

//Gloabl Variables
move_base_msgs::MoveBaseGoal goal;   //Variable to access position and orientation of the robot using move base

vector<vector<double>> points = { 
      {9.17, 19.30, 0},
      {9.40, 32.45, 0},
      {18.0, 25.52, 0},
      {28.14, 30.47, 0},
      {25.65, 23.15, 0},
      {28.12, 13.44, 0},
      {18.19, 12.48, 0},
      {16.79, 19, 0},
      {9.29, 15.58, 0}, 
      {4, 2, 0}
  };
vector<double> navTime(points.size(), 0.0); //Store time taken for each point

// Function to check that the action server is working
void action_server_state()
{
  ROS_INFO("Goal pose is being processed");
}

//Give the current location of the robot in the cartesian coordinate frame
void current_robot_state(const move_base_msgs::MoveBaseFeedbackConstPtr& current_pose)
{
   ROS_INFO("Current location: x = %f, y = %f", current_pose->base_position.pose.position.x, current_pose->base_position.pose.position.y);
}

//GIve information when the navigation is over or when a problem occured
void done_navigation(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Goal reached");
  }
  else if (state == actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_INFO("Goal aborted");
  }
  else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_INFO("Goal preempted");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_motion");
  ros::NodeHandle nh;

  //Action
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navclient("move_base", true);
  navclient.waitForServer();  //Wait for the server to be available

  ros::Time start_time = ros::Time::now();

  for (size_t i = 0; i < points.size(); ++i)
  {
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = points[i][0];
    goal.target_pose.pose.position.y = points[i][1];
    goal.target_pose.pose.position.z = points[i][2];
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.662;
    goal.target_pose.pose.orientation.w = 0.750;
    
    navclient.sendGoal(goal, &done_navigation, &action_server_state, &current_robot_state);

    // Wait for action result
    bool finished = navclient.waitForResult();

    if (!finished) //If server not available 
    {
      ROS_ERROR("Action server not available!");
    }
    else          //Otherwise print the result status
    {
    
    double timeTaken = (ros::Time::now() - start_time).toSec();
    navTime[i] = timeTaken;
    
    ROS_INFO("Result: %s", navclient.getState().toString().c_str());    
    ros::Duration(3.0).sleep();
    }
  }
   
    ROS_INFO("The navigation time between each point was: ");
    for (size_t i = 0; i < navTime.size(); ++i)
    {
      ROS_INFO("Point %zu: %.2f minutes", i + 1, navTime[i]/60);
    }
 
  return 0;
}