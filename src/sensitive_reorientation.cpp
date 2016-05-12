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
  InitActionClient * init_action_client_; //!< initialisation client
  HeadActionClient * head_action_client_; //!< interface to head controller client
public:
  /**
   * Default constructor
   */
  SensitiveReorientation(ros::NodeHandle& node)
  {
    inhibition_client_ = node.serviceClient<head_manager::InhibitionOfReturn>("head_manager/inhibition_of_return");
    salient_stim_sub_ = node.subscribe("/head_manager/salient_stimuli", 5, &SensitiveReorientation::salientStimuliCallback, this);

    init_action_client_ = new InitActionClient("pr2motion/Init", true);
    // Initialize action client for the action interface to the head controller
    head_action_client_ = new HeadActionClient("pr2motion/Head_Move_Target", true);
    // Connection to the pr2motion client
    ros::ServiceClient connect = node.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    ROS_INFO("Waiting for pr2motion action server to start.");
    init_action_client_->waitForServer(); //will wait for infinite time
    head_action_client_->waitForServer(); //will wait for infinite time
    ROS_INFO("pr2motion action server started.");

    pr2motion::InitGoal goal_init;
    init_action_client_->sendGoal(goal_init);

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
  ~SensitiveReorientation(){}
private:

  void lookAt(std::string target_frame, float target_x, float target_y, float target_z)
  {
    pr2motion::Head_MoveGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = target_frame;
    goal.head_target_x = target_x;
    goal.head_target_y = target_y;
    goal.head_target_z = target_z;
    head_action_client_->sendGoal(goal);
    
    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(300.0));

    if (finishedBeforeTimeout)
    {
      actionlib::SimpleClientGoalState state = head_action_client_->getState();
      ROS_DEBUG("[head_manager] Action finished: %s",state.toString().c_str());
    }
    else
      ROS_DEBUG("[head_manager] Action did not finish before the time out.");
  }

  void salientStimuliCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    head_manager::InhibitionOfReturn srv;
    srv.request.point.header=msg->header;
    srv.request.point.point=msg->point;
    if (inhibition_client_.exists())
      ROS_INFO("service ready");
    else
      ROS_ERROR("service not ready");

    if (!inhibition_client_.call(srv))
    {
      ROS_ERROR("[head_manager] Failed to call service : inhibition_of_return");
    }
    lookAt(msg->header.frame_id,msg->point.x,msg->point.y,msg->point.z);
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