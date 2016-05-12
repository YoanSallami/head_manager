#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "geometry_msgs/PointStamped.h"

#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Robot.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/Entity.h"

#include "supervisor_msgs/AgentActivity.h"

using namespace std;

//for convenience
typedef std::map<std::string,ros::Subscriber> SubscriberMap_t;
typedef std::pair<std::string,ros::Subscriber> SubscriberPair_t;
typedef std::map<std::string,supervisor_msgs::AgentActivity> ActivityMap_t;
typedef std::pair<std::string,supervisor_msgs::AgentActivity> ActivityPair_t;
typedef std::vector < toaster_msgs::Object > ObjectList_t;
typedef std::vector < toaster_msgs::Robot > RobotList_t;
typedef std::vector < toaster_msgs::Human > HumanList_t;

class CognitiveOrientation
{
private:
  ros::NodeHandle node_;
  std::string my_id_;
  SubscriberMap_t agent_activity_sub_map_;
  std::vector<toaster_msgs::Object> object_list_; //!< object list from toaster
  std::vector<toaster_msgs::Human> human_list_; //!< human list from toaster
  std::vector<toaster_msgs::Robot> robot_list_; //!< robot list from toaster
  ActivityMap_t agent_activity_map_;
  ros::Subscriber object_list_sub_;
  ros::Subscriber human_list_sub_;
  ros::Subscriber robot_list_sub_;
public:
  /**
   * Default constructor
   */
  CognitiveOrientation(ros::NodeHandle& node)
  {
    node_=node;
    //Getting robot's id from ros param
    if(node_.hasParam("my_robot_id"))
    {
      node_.getParam("my_robot_id",my_id_);
    } else {
      my_id_="pr2";
    }
    object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &CognitiveOrientation::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &CognitiveOrientation::humanListCallback, this);
    robot_list_sub_ = node_.subscribe("/pdg/robotList", 1, &CognitiveOrientation::objectListCallback, this);
  }
  /**
   * Default destructor
   */
  ~CognitiveOrientation(){}

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
              object_list_[i].meEntity.positionX+=offset[0];
              object_list_[i].meEntity.positionY+=offset[1];
              object_list_[i].meEntity.positionZ+=offset[2];
            }
          }
          return (object_list_[i].meEntity);
        }
      }
    } else {
      throw HeadManagerException ( "Could not get object entity :\""+id+"\" in an empty object list." );
    }
  }
  /****************************************************
   * @brief : Get human entity from human list
   * @param : human's id
   * @return : human entity
   ****************************************************/
  toaster_msgs::Entity getHuman(std::string id)
  {
    if (!human_list_.empty())
    {
      for (unsigned int i = 0; i < human_list_.size(); ++i)
      {
        if (human_list_[i].meAgent.meEntity.id == id)
        {
          return (human_list_[i].meAgent.meEntity);
        }
      }
    } else {
      throw HeadManagerException ( "Could not get human entity :\""+id+"\" in an empty human list." );
    }
  }
  /****************************************************
   * @brief : Get human joint entity from human list
   * @param : object's id
   * @param : joint's id
   * @return : human joint entity
   ****************************************************/
  toaster_msgs::Entity getHumanJoint(std::string ownerId, std::string jointId)
  {
    if (!human_list_.empty())
    {
      for (unsigned int i = 0; i < human_list_.size(); ++i)
      {
        if (human_list_[i].meAgent.meEntity.id == ownerId)
        {
          if (!human_list_[i].meAgent.skeletonJoint.empty())
          {
            for (unsigned int j = 0; j < human_list_[i].meAgent.skeletonJoint.size(); ++j)
            {
              if (human_list_[i].meAgent.skeletonJoint[j].meEntity.id == jointId)
              {
                return (human_list_[i].meAgent.skeletonJoint[j].meEntity);
              }
            }
          } else {
            throw HeadManagerException ( "Could not get human joint entity \""+jointId+"\" in an empty skeletonJoint list." );
          }
        }
      }
    } else {
      throw HeadManagerException ( "Could not get human entity \""+ownerId+"\" in an empty human list." );
    }
  }
  /****************************************************
   * @brief : Get robot entity from robot list
   * @param : robot's id
   * @return : robot entity
   ****************************************************/
  toaster_msgs::Entity getRobot(std::string id)
  {
    if (!robot_list_.empty())
    {
      for (unsigned int i = 0; i < robot_list_.size(); ++i)
      {
        if (robot_list_[i].meAgent.meEntity.id == id)
        {
          return (robot_list_[i].meAgent.meEntity);
        }
      }
    } else {
      throw HeadManagerException ( "Could not get robot entity :\""+id+"\" in an empty robot list." );
    }
  }
  /****************************************************
   * @brief : Get robot joint entity from robot list
   * @param : robot's id
   * @param : joint's id
   * @return : robot joint entity
   ****************************************************/
  toaster_msgs::Entity getRobotJoint(std::string ownerId, std::string jointId)
  {
    if (!robot_list_.empty())
    {
      for (unsigned int i = 0; i < robot_list_.size(); ++i)
      {
        if (robot_list_[i].meAgent.meEntity.id == ownerId)
        {
          if (!robot_list_[i].meAgent.skeletonJoint.empty())
          {
            for (unsigned int j = 0; j < robot_list_[i].meAgent.skeletonJoint.size(); ++j)
            {
              if (robot_list_[i].meAgent.skeletonJoint[j].meEntity.id == jointId)
              {
                return (robot_list_[i].meAgent.skeletonJoint[j].meEntity);
              }

            }
          } else {
            throw HeadManagerException ( "Could not get robot joint entity :\""+jointId+"\" in an empty skeletonJoint list." );
          }
        }
      }
    } else {
      throw HeadManagerException ( "Could not get robot entity \""+ownerId+"\" in an empty robot list." );
    }
  }
  /****************************************************
   * @brief : Test if the object list contain this id
   * @param : tested id
   * @return : true if contain
   ****************************************************/
  bool isObject(std::string id)
  {
    if(!object_list_.empty())
    {
      for (unsigned int i = 0; i < object_list_.size(); ++i)
      {
        if (object_list_[i].meEntity.id == id)
        {
          return (true);
        }
      }
    } else {
      return (false);
    }
  }
  /****************************************************
   * @brief : Test if the id contain multiples ids
   * @param : tested id
   * @return : true if contain
   ****************************************************/
  bool isJoint(std::string id)
  {
    std::size_t found=id.find("::");
    if(id.find("::") != std::string::npos)
      return(true);
    return(false);
  }

private:
  /****************************************************
   * @brief : Update the activity state map
   * @param : object list
   ****************************************************/
  void activityCallback(const supervisor_msgs::AgentActivity::ConstPtr& msg,std::string id)
  {
    bool succeed=false;
    ActivityMap_t::iterator it = agent_activity_map_.begin();

    while( it != agent_activity_map_.end()  && succeed==false)
    {
      if (it->first == id)
      {
        it->second=*msg;
        succeed=true;
      }
    }
    if (succeed==false)
    {
      agent_activity_map_.insert(ActivityPair_t(id,*msg));
    }
  }
  /****************************************************
   * @brief : Update the object list
   * @param : object list
   ****************************************************/
  void objectListCallback(const toaster_msgs::ObjectList::ConstPtr& msg)
  {
    if(!msg->objectList.empty())
    {
      object_list_.clear();
      for (unsigned int i = 0; i < msg->objectList.size(); ++i)
      {
        object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));
      }
    }
  }
  /****************************************************
   * @brief : Update the robot list
   * @param : robot list
   ****************************************************/
  void robotListCallback(const toaster_msgs::RobotList::ConstPtr& msg)
  {
    if(!msg->robotList.empty())
    {
      robot_list_.clear();
      for (unsigned int i = 0; i < msg->robotList.size(); ++i)
      {
        robot_list_.push_back(*(new toaster_msgs::Robot(msg->robotList[i])));
        std::string id = msg->robotList[i].meAgent.meEntity.id;
        std::string topicName = "/supervisor/activity_state/"+id;
        if (agent_activity_sub_map_.find(id) == agent_activity_sub_map_.end())
        {
          ros::Subscriber sub = node_.subscribe<supervisor_msgs::AgentActivity>((const string)topicName, 1,boost::bind(&CognitiveOrientation::activityCallback,this,_1,id));
          agent_activity_sub_map_.insert(SubscriberPair_t(id,sub));
        }
      }
    }
  }
  /****************************************************
   * @brief : Update the human list & activity state
   *          subscriber map
   * @param : human list
   ****************************************************/
  void humanListCallback(const toaster_msgs::HumanList::ConstPtr& msg)
  {
    std::string ownerId,jointId;
    
    if (!msg->humanList.empty())
    {
      human_list_.clear();

      for (unsigned int i = 0; i < msg->humanList.size(); ++i)
      {
        human_list_.push_back(*(new toaster_msgs::Human(msg->humanList[i])));
        std::string id = msg->humanList[i].meAgent.meEntity.id;
        std::string topicName = "/supervisor/activity_state/"+id;
        if (agent_activity_sub_map_.find(id) == agent_activity_sub_map_.end())
        {
          ros::Subscriber sub = node_.subscribe<supervisor_msgs::AgentActivity>((const string)topicName, 1,boost::bind(&CognitiveOrientation::activityCallback,this,_1,id));
          agent_activity_sub_map_.insert(SubscriberPair_t(id,sub));
        }
      }
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
  ros::init(argc, argv, "cognitive_orientation");
  ros::NodeHandle n;
  CognitiveOrientation * co = new CognitiveOrientation(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}