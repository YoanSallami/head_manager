#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_MoveAction.h>
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/Object.h"
#include <geometry_msgs/PointStamped.h>
#include "head_manager/InhibitionOfReturn.h"

using namespace std;

typedef actionlib::SimpleActionClient<pr2motion::InitAction> InitActionClient;
typedef actionlib::SimpleActionClient<pr2motion::Head_MoveAction> HeadActionClient;

class HeadManagerNode
{
private:
  InitActionClient * init_action_client_; //!< initialisation client
  HeadActionClient * head_action_client_; //!< interface to head controller client
  std::vector<toaster_msgs::Object> object_list_; //!< object list from toaster
  std::vector<toaster_msgs::Human> human_list_; //!< human list from toaster
  std::vector<toaster_msgs::Robot> robot_list_; //!< robot list from toaster
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber salient_stim_sub_; //!<
  ros::ServiceClient inhibition_client_ ;
public:
  /** 
   * Default constructor
   */
  HeadManagerNode(ros::NodeHandle& node)
  {
    inhibition_client_ = node.serviceClient<head_manager::InhibitionOfReturn>("head_manager/inhibition_of_return");

    salient_stim_sub_ = node.subscribe("/head_manager/salient_stimuli", 5, &HeadManagerNode::salientStimuliCallback, this);
    //
    object_list_sub_ = node.subscribe("/toaster_simu/objectList", 20, &HeadManagerNode::objectListCallback, this);
    // Initialize action client for the action interface to the robot controller
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
    // srv.request.local = "traj";
    // srv.request.remote = "gtp_trajectory";
    // if (!connect.call(srv)){
    //   ROS_ERROR("[head_manager] Failed to call service pr2motion/connect_port");
    // }
  }
  /** 
   * Default destructor
   */
  ~HeadManagerNode()
  {
    delete(init_action_client_);
    delete(head_action_client_);
    delete(&object_list_);
  }

  void lookAtObject(std::string id)
  {
    bool isExisting=false;
    if (!object_list_.empty()){
      for (int i = 0; i < object_list_.size()-1; ++i)
      {
        if (object_list_[i].meEntity.id==id)
        {
          this->lookAt("map",object_list_[i].meEntity.positionX,object_list_[i].meEntity.positionY,object_list_[i].meEntity.positionZ);
          isExisting=true;
          break;
        }
      }
    }
    if (!isExisting)
    {
      ROS_WARN("[head_manager] '%s' is not existing",id.c_str());
    }
  }

  void lookAtHuman(std::string id)
  {

  }

  void lootAtHumanHand(std::string id)
  {
    
  }

  void lookRightGripper()
  {
    this->lookAt("r_gripper_frame", 0.0, 0.0, 0.0);
  }
  void lookLeftGrepper()
  {
    this->lookAt("l_gripper_frame", 0.0, 0.0, 0.0);
  }

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
private:

  void objectListCallback(const toaster_msgs::ObjectList::ConstPtr& msg)
  {
    object_list_.clear();
    for (unsigned int i = 0; i < msg->objectList.size()-1; ++i)
    {
      object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));
    }
  }

  void salientStimuliCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {

    lookAt(msg->header.frame_id, msg->point.x, msg->point.y, msg->point.z);

    head_manager::InhibitionOfReturn srv;
    srv.request.point.header.frame_id=msg->header.frame_id;
    srv.request.point.header.stamp=ros::Time::now();
    srv.request.point.point.x=msg->point.x;
    srv.request.point.point.y=msg->point.y;
    srv.request.point.point.z=msg->point.z;
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
  ros::init(argc, argv, "head_driver");
  ros::NodeHandle n;
  HeadManagerNode * head = new HeadManagerNode(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}