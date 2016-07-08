#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include <pr2motion/Head_Stop.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include "head_manager/Focus.h"
#include "head_manager/AttentionStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef actionlib::SimpleActionClient<pr2motion::InitAction> InitActionClient_t;
typedef actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction> HeadActionClient_t;
typedef message_filters::sync_policies::ApproximateTime<head_manager::AttentionStamped, head_manager::AttentionStamped, head_manager::Focus> MySyncPolicy;

class SensitiveReorientation
{
private:
  ros::NodeHandle node_; //!<
  ros::Publisher effective_attention_pub_; //!<
  ros::Publisher tag_detection_pub_; //!<
  message_filters::Subscriber<head_manager::AttentionStamped> salient_stim_sub_; //!<
  message_filters::Subscriber<head_manager::AttentionStamped> goal_directed_attention_sub_;
  message_filters::Subscriber<head_manager::Focus> focus_sub_; //!<
  InitActionClient_t * init_action_client_; //!< initialisation client
  HeadActionClient_t * head_action_client_; //!< interface to head controller client
  ros::ServiceClient connect_port_srv_;
  ros::ServiceClient head_stop_srv_;
  message_filters::Synchronizer<MySyncPolicy> * sync_;
  
public:
  /**
   * Default constructor
   */
  SensitiveReorientation(ros::NodeHandle& node)
  {
    node_=node;
    salient_stim_sub_.subscribe(node_, "/head_manager/salient_stimuli", 5);
    goal_directed_attention_sub_.subscribe(node_, "/head_manager/goal_directed_attention", 5);
    focus_sub_.subscribe(node_,"/head_manager/focus", 5);
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(5),salient_stim_sub_, goal_directed_attention_sub_, focus_sub_);
    sync_->registerCallback(boost::bind(&SensitiveReorientation::callback,this, _1, _2, _3));
    effective_attention_pub_ = node_.advertise <geometry_msgs::PointStamped>("head_manager/effective_attention", 5);
    tag_detection_pub_ = node_.advertise <std_msgs::Bool>("ar_track_alvar/enable_detection",1);
    init_action_client_ = new InitActionClient_t("pr2motion/Init", true);
    // Initialize action client for the action interface to the head controller
    head_action_client_ = new HeadActionClient_t("pr2motion/Head_Move_Target", true);
    // Connection to the pr2motion client
    connect_port_srv_ = node_.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    head_stop_srv_ = node_.serviceClient<pr2motion::connect_port>("pr2motion/Head_Stop");
    ROS_INFO("Waiting for pr2motion action server to start.");
    init_action_client_->waitForServer(); //will wait for infinite time
    head_action_client_->waitForServer(); //will wait for infinite time
    ROS_INFO("pr2motion action server started.");

    pr2motion::InitGoal goal_init;
    init_action_client_->sendGoal(goal_init);

    pr2motion::connect_port srv;
    srv.request.local = "joint_state";
    srv.request.remote = "joint_states";
    if (!connect_port_srv_.call(srv)){
      ROS_ERROR("[sensitive_reorientation] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect_port_srv_.call(srv)){
      ROS_ERROR("[sensitive_reorientation] Failed to call service pr2motion/connect_port");
    }
    // srv.request.local = "head_desired_position";
    // srv.request.remote = "/head_manager/salient_stimuli";
    // if (!connect_port_srv_.call(srv)){
    //   ROS_ERROR("[sensitive_reorientation] Failed to call service pr2motion/connect_port");
    // }
  }
  /** 
   * Default destructor
   */
  ~SensitiveReorientation(){}
private:

  void lookAtObject(geometry_msgs::PointStamped p)
  {
    std_msgs::Bool enable_detect_tag;
    enable_detect_tag.data=false;
    tag_detection_pub_.publish(enable_detect_tag);
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = p.header.frame_id;
    goal.head_target_x = p.point.x;
    goal.head_target_y = p.point.y;
    goal.head_target_z = p.point.z;
    head_action_client_->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(300.0));
    enable_detect_tag.data=true;
    tag_detection_pub_.publish(enable_detect_tag);
    if (finishedBeforeTimeout)
    {
      actionlib::SimpleClientGoalState state = head_action_client_->getState();
      ROS_INFO("[sensitive_reorientation] Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("[sensitive_reorientation] Action did not finish before the time out.");
  }

  void lookAt(geometry_msgs::PointStamped p)
  {
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = p.header.frame_id;
    goal.head_target_x = p.point.x;
    goal.head_target_y = p.point.y;
    goal.head_target_z = p.point.z;
    head_action_client_->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(300.0));

    if (finishedBeforeTimeout)
    {
      actionlib::SimpleClientGoalState state = head_action_client_->getState();
      ROS_INFO("[sensitive_reorientation] Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("[sensitive_reorientation] Action did not finish before the time out.");
  }

  void callback(const head_manager::AttentionStamped::ConstPtr& salient_stimuli,
    const head_manager::AttentionStamped::ConstPtr& goal_directed_attention,
    const head_manager::Focus::ConstPtr& focus)
  {
    geometry_msgs::PointStamped effective_attention;
    pr2motion::Head_Stop stop;
    effective_attention.header.stamp = ros::Time::now();
    effective_attention.header.frame_id = "map";
    effective_attention.point.x=((1-focus->data)*salient_stimuli->point.x)+(focus->data*goal_directed_attention->point.x);
    effective_attention.point.y=((1-focus->data)*salient_stimuli->point.y)+(focus->data*goal_directed_attention->point.y);
    effective_attention.point.z=((1-focus->data)*salient_stimuli->point.z)+(focus->data*goal_directed_attention->point.z);
    if (!ros::service::call("pr2motion/Head_Stop", stop))
      ROS_ERROR("[sensitive_reorientation] Failed to call service pr2motion/Head_Stop");
    if (focus->data==0)
    {
      if (salient_stimuli->object)
      {
        lookAtObject(effective_attention);
      } else {
        lookAt(effective_attention);
      }
    } else {
      if (goal_directed_attention->object)
      {
        lookAtObject(effective_attention);
      } else {
        lookAt(effective_attention);
      }
    }
    
    effective_attention_pub_.publish(effective_attention);
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