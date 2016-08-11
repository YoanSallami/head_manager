#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include "../include/head_manager/HeadManagerException.h"
#include "../include/optitrack/or_pose_estimator_state.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/Entity.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<optitrack::or_pose_estimator_state, toaster_msgs::ObjectListStamped> MySyncPolicy;
typedef std::vector < toaster_msgs::Object > ObjectList_t;

class MocapCalib
{
private:
	ros::NodeHandle node_;
	ros::Subscriber mocap_tag_sub_;
	ros::Subscriber object_list_sub_;
public:
	ObjectList_t object_list_;
	double mocap_x_;
	double mocap_y_;
	double mocap_z_;
public:
  MocapCalib(ros::NodeHandle node)
  {
  	node_ = node;
  	mocap_tag_sub_ = node_.subscribe("/optitrack/bodies/MIRE", 1, &MocapCalib::mocapCallback, this);
  	object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &MocapCalib::objectCallback, this);
  }
private:
  void mocapCallback(const optitrack::or_pose_estimator_state::ConstPtr &mocap)
  {
	mocap_x_=mocap->pos[0].x;
	mocap_y_=mocap->pos[0].y;
	mocap_z_=mocap->pos[0].z;
  }

  void objectCallback(const toaster_msgs::ObjectListStamped::ConstPtr &msg)
  {
  	try
    {
      if(!msg->objectList.empty())
      {
        object_list_.clear();
        for (unsigned int i = 0; i < msg->objectList.size(); ++i)
        {
          if (msg->objectList[i].meEntity.id!="unknown object")
          {
            object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));
          }
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[mocap_calib] Exception was caught : %s",e.description().c_str());
    }
  }
public:
   /****************************************************
   * @brief : Get object entity from object list
   * @param : object's id
   * @return : object entity
   ****************************************************/
  toaster_msgs::Entity getObject(std::string id)
  {
    double x,y,z;
    std::vector<float> offset;
    if(!object_list_.empty())
    {
      for (unsigned int i = 0; i < object_list_.size(); ++i)
      {
        if (object_list_[i].meEntity.id == id)
        {
          std::string offset_str = "offset_"+object_list_[i].meEntity.id;
          if (node_.hasParam(offset_str))
          {
            offset.clear();
            if (node_.getParam((const string) offset_str,offset))
            {
              object_list_[i].meEntity.pose.position.x+=offset[0];
              object_list_[i].meEntity.pose.position.y+=offset[1];
              object_list_[i].meEntity.pose.position.z+=offset[2];
            }
          }
          return (object_list_[i].meEntity);
        }
      }
    } else {
      throw HeadManagerException ( "Could not get object entity :\""+id+"\" in an empty object list." );
    }
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
  MocapCalib * cm = new MocapCalib(n);
  ROS_INFO("[mocap_calib] Make sure that ar_track_alvar and pdg is running !");
  ROS_INFO("[mocap_calib] Make sure that ar_track_alvar detection is enable and pdg stream managed !");
  ROS_INFO("[mocap_calib] Make sure that the calibration tag is visible by the robot and the mocap system !");
  while(ros::ok())
  {
  	ros::spinOnce();
  	if(!cm->object_list_.empty())
  	{
  		try
  		{

	  		toaster_msgs::Entity chess=cm->getObject("CHESSBOARD");
	  		double pos_x=chess.pose.position.x-cm->mocap_x_;
	  		double pos_y=chess.pose.position.y-cm->mocap_y_;
	  		double pos_z=chess.pose.position.z-cm->mocap_z_;
        if (pos_x<100.0 && pos_y<100.0 && pos_x<100.0)
        {
          ros::param::set("mocap_calib_world_x",pos_x);
          ros::param::set("mocap_calib_world_y",pos_y);
          ros::param::set("mocap_calib_world_z",pos_z);
          ROS_INFO("[mocap_calib] Position offset = [x:%f, y:%f, z: %f]",pos_x,pos_y,pos_z);
          break;
        }
	  	}
	  	catch (HeadManagerException& e )
	    {
	      ROS_ERROR("[mocap_calib] Exception was caught : %s",e.description().c_str());
	    }
  	}
  }
}