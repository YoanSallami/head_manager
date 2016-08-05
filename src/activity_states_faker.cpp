#include <ros/ros.h>
#include <string>
#include "supervisor_msgs/AgentActivity.h"

using namespace std;

class ActivityStatesFaker
{
private:
	ros::NodeHandle node_;
	ros::Publisher pr2_pub_;
	ros::Publisher human_pub_;
	ros::Timer timer_;
	ros::ServiceServer Action_srv_;
	ros::ServiceServer setObjectFocus_srv_;
	bool robotActing_;
	std::string object_;

public:
	ActivityStatesFaker(ros::NodeHandle node)
	{
		node_ = node;
		bool robotActing_=false;
		ros::ServiceServer Action_srv_ = node_.advertiseService("robot_action", &ActivityStatesFaker::Action,this);
		pr2_pub_ = node_.advertise <supervisor_msgs::AgentActivity> ("supervisor/activity_states/PR2_ROBOT", 1);
		human_pub_ = node_.advertise <supervisor_msgs::AgentActivity> ("supervisor/activity_states/HERAKLES_HUMAN1", 1);
		timer_ = node_.createTimer(ros::Duration(1.0/30.0), &ActivityStatesFaker::timerCallback, this);
	}
private:
	void timerCallback(const ros::TimerEvent&)
	{
		supervisor_msgs::ActivityState pr2_msg;
		supervisor_msgs::ActivityState human_msg;

		if (robotActing_)
		{
			pr2_msg.activityState="ACTING";
			pr2_msg.object=object_;
			pr2_msg.importancy=0.8;
			pr2_msg.unexpected=false;
			pr2_msg.stopable=true;
			pr2_pub_.publish(pr2_msg);
		}else{
			pr2_msg.activityState="WAITING";
			pr2_msg.object="";
			pr2_msg.importancy=0.8;
			pr2_msg.unexpected=false;
			pr2_msg.stopable=true;
			pr2_pub_.publish(pr2_msg);
		}
		
		human_msg.activityState="WAITING";
		human_msg.object="";
		human_msg.importancy=0.8;
		human_msg.unexpected=false;
		human_msg.stopable=true;
		human_pub_.publish(human_msg);
	}

	bool Action(head_manager::Action::Request &req
				head_manager::Action::Response &res)
	{
		robotActing_=req->acting;
		object_=req->object;
		return(true);
	}

};
/****************************************************
 * @brief : Main process function
 * @param : arguments count
 * @param : arguments values
 ****************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "activity_states_faker");
  ros::NodeHandle n;
  ActivityStatesFaker * co = new ActivityStatesFaker(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}