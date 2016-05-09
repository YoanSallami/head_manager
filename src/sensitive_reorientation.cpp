#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_MoveAction.h>
#include <geometry_msgs/PointStamped.h>
#include "head_manager/InhibitionOfReturn.h"

using namespace std;

typedef actionlib::SimpleActionClient<pr2motion::InitAction> InitActionClient;
typedef actionlib::SimpleActionClient<pr2motion::Head_MoveAction> HeadActionClient;

class SensitiveReorientation
{
private:
  ros::Subscriber salient_stim_sub_;
  ros::ServiceClient inhibition_client_ ;
  //InitActionClient * init_action_client_; //!< initialisation client
  //HeadActionClient * head_action_client_; //!< interface to head controller client
public:
  /**
   * Default constructor
   */
  SensitiveReorientation(ros::NodeHandle& node)
  {
    inhibition_client_ = node.serviceClient<head_manager::InhibitionOfReturn>("head_manager/inhibition_of_return");
    salient_stim_sub_ = node.subscribe("/head_manager/salient_stimuli", 5, &SensitiveReorientation::salientStimuliCallback, this);

    //init_action_client_ = new InitActionClient("pr2motion/Init", true);
    // Initialize action client for the action interface to the head controller
    //head_action_client_ = new HeadActionClient("pr2motion/Head_Move_Target", true);
    // Connection to the pr2motion client
    ros::ServiceClient connect = node.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    //ROS_INFO("Waiting for pr2motion action server to start.");
    //init_action_client_->waitForServer(); //will wait for infinite time
    //head_action_client_->waitForServer(); //will wait for infinite time
    //ROS_INFO("pr2motion action server started.");

    //pr2motion::InitGoal goal_init;
    //init_action_client_->sendGoal(goal_init);

    pr2motion::connect_port srv;
    srv.request.local = "joint_state";
    srv.request.remote = "joint_states";
    if (!connect.call(srv)){
      ROS_ERROR("[head_manager] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect.call(srv)){
      ROS_ERROR("[head_manager] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_desired_position";
    srv.request.remote = "/head_manager/salient_stimuli";
    if (!connect.call(srv)){
      ROS_ERROR("[head_manager] Failed to call service pr2motion/connect_port");
    }
  }
  /** 
   * Default destructor
   */
  ~SensitiveReorientation()
  {
    // delete(init_action_client_);
    // delete(head_action_client_);
  }
private:
  void salientStimuliCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    head_manager::InhibitionOfReturn srv;
    if (inhibition_client_.exists())
      ROS_INFO("service ready");
    else
      ROS_ERROR("service not ready");

    if (inhibition_client_.call(srv))
    {
      if (srv.response.success==true)
      ROS_INFO("inhibition_of_return : SUCCESS ");
      else
        ROS_INFO("inhibition_of_return : FAIL ");
    }
    else
    {
      ROS_ERROR("Failed to call service inhibition_of_return");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensitive_reorientation");
  ros::NodeHandle n;
  SensitiveReorientation * sr = new SensitiveReorientation(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}