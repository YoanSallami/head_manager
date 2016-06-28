#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "head_manager/Sync.h"
#include "head_manager/Signal.h"
#include "head_manager/Focus.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"

#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Robot.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/Entity.h"

#include "supervisor_msgs/AgentActivity.h"

using namespace std;

//for convenience
typedef std::map<std::string,ros::Subscriber> SubscriberMap_t;
typedef std::pair<std::string,ros::Subscriber> SubscriberPair_t;
typedef std::map<std::string,supervisor_msgs::AgentActivity> ActivityMap_t;
typedef std::pair<std::string,supervisor_msgs::AgentActivity> ActivityPair_t;
typedef std::map<std::string,bool> AckMap_t;
typedef std::pair<std::string,bool> AckPair_t;
typedef std::vector < toaster_msgs::Object > ObjectList_t;
typedef std::vector < toaster_msgs::Robot > RobotList_t;
typedef std::vector < toaster_msgs::Human > HumanList_t;
typedef std::vector < toaster_msgs::Fact > FactList_t;
typedef std::vector < head_manager::Signal > SignalList_t;

class GoalDirectedAttention
{
private:
  std::string my_id_; //!< robot's id
  ros::NodeHandle node_; //!< node handler
  SubscriberMap_t agent_activity_sub_map_; //!< agent activity state subscribers map
  ObjectList_t object_list_; //!< object list from pdg
  HumanList_t human_list_; //!< human list from pdg
  RobotList_t robot_list_; //!< robot list from pdg
  FactList_t fact_list_; //!< fact list from pdg
  AckMap_t ack_map_; //!< acknowledgement map;
  SignalList_t signal_list_; //!< signal list from supervisor
  ActivityMap_t agent_activity_map_; //!< agent activity state machines map
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber robot_list_sub_; //!< robot list subscriber
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber signal_sub_; //!< signal subscriber
  ros::Publisher goal_directed_attention_pub_; //!< goal_directed attention publisher
  ros::Publisher signal_pub_; //!<
  double focus_; //!< robot focus
  ros::Publisher focus_pub_; //!< focus publisher
  //bool surprised_; //!< true when an unexpected act occur
  head_manager::Signal current_signal_; //!<
  toaster_msgs::Entity current_entity_; //!<
  int current_signal_it_; //!<
  ros::Time signal_start_time_; //!<  
  bool signalling_; //!<
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  GoalDirectedAttention(ros::NodeHandle& node)
  {
    node_=node;
    //Getting robot's id from ros param
    if(node_.hasParam("my_robot_id"))
    {
      node_.getParam("my_robot_id",my_id_);
    } else {
      my_id_="pr2";
    }
    // Advertise subscribers
    object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &GoalDirectedAttention::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &GoalDirectedAttention::humanListCallback, this);
    robot_list_sub_ = node_.subscribe("/pdg/robotList", 1, &GoalDirectedAttention::objectListCallback, this);
    fact_list_sub_ = node_.subscribe("/pdg/factList", 1, &GoalDirectedAttention::factListCallback, this);
    signal_sub_ = node_.subscribe("head_manager/signal", 10, &GoalDirectedAttention::signalCallback, this);
    // Advertise publishers
    goal_directed_attention_pub_ = node_.advertise<geometry_msgs::PointStamped>("head_manager/goal_directed_attention", 5);
    focus_pub_ = node_.advertise <head_manager::Focus> ("head_manager/focus", 5);
    signal_pub_ = node_.advertise <head_manager::Signal> ("head_manager/signal", 10);
    //surprised_=false;
    signalling_=false;
    current_signal_it_=0;
    signal_start_time_;
    focus_=0.0;
  }
  /****************************************************
   * @brief : Default destructor
   ****************************************************/
  ~GoalDirectedAttention(){}

  /****************************************************
   * @brief : Get entity from object/human/robot list
   * @param : entity's id
   * @return : entity
   ****************************************************/
  toaster_msgs::Entity getEntity(std::string id)
  {
    toaster_msgs::Entity entity;
    if(!isObject(id))
    {
      if (isJoint(id))
      {
        size_t pos = id.find("::");
        if(isHuman(id.substr(0,pos)))
          entity = getHumanJoint(id.substr(0,pos),id.substr(pos+2,id.size()-1));
        else
          entity = getRobotJoint(id.substr(0,pos),id.substr(pos+2,id.size()-1));
      } else {
        if(isHuman(id))
          entity = getHuman(id);
        else
          entity = getRobot(id);
      }
    } else {
      entity = getObject(id);
    }
    return(entity);
  }
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
   * @brief : Test if the human list contain this id
   * @param : tested id
   * @return : true if contain
   ****************************************************/
  bool isHuman(std::string id)
  {
    if(!human_list_.empty())
    {
      for (unsigned int i = 0; i < human_list_.size(); ++i)
      {
        if (human_list_[i].meAgent.meEntity.id == id)
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
  /****************************************************
   * @brief : Compute goal-directed attention
   ****************************************************/
  void updateSignalList()
  {
    if(!signal_list_.empty())
    {
      for (SignalList_t::iterator it = signal_list_.begin(); it != signal_list_.end(); ++it)
      {
        if (it->urgency!=0.0)
        {
          it->importancy-=it->importancy*(1.0-it->urgency);
        } else {
          it->importancy-=it->importancy;
        }
        if(it->importancy<0.000001)
          signal_list_.erase(it);
      }
    } else {
      throw HeadManagerException ("Could not update an empty signal list.");
    }
  }
  /****************************************************
   * @brief : Test if the id contain multiples ids
   * @param : tested id
   * @return : true if current signal changed
   ****************************************************/
  bool selectSignal(head_manager::Signal & sig)
  {
    double max=sig.importancy;
    SignalList_t::iterator max_it;
    bool changed=false;
    if (!signal_list_.empty())
    {
      for (SignalList_t::iterator it = signal_list_.begin(); it != signal_list_.end(); ++it)
      {
        if(it->importancy > max)
        {
          max_it=it;
          max=it->importancy;
          changed=true;
        }
      }
      sig=*max_it;
      signal_list_.erase(max_it);
      return(changed);
    }
    return(false);
  }
  /****************************************************
   * @brief : Compute goal-directed attention
   ****************************************************/
  void updateAckMap()
  {
    if (!ack_map_.empty())
    {
      for (AckMap_t::iterator it_am = ack_map_.begin(); it_am != ack_map_.end(); ++it_am)
      {
        it_am->second=false;
      }
    }
    if (!fact_list_.empty())
    {
      for (FactList_t::iterator it_fl = fact_list_.begin(); it_fl != fact_list_.end(); ++it_fl)
      {
        if (it_fl->subjectId!=my_id_ && it_fl->targetId==my_id_)
        {
          if ( it_fl->property == "IsFacing")
          {
            if (ack_map_.find(it_fl->subjectId)!=ack_map_.end())
            {
              ack_map_.find(it_fl->subjectId)->second=true;
            } else {
              ack_map_.insert(AckPair_t(it_fl->subjectId,true));
            }
          }
        }
      }
    } else {
      throw HeadManagerException ("Could not read an empty fact list.");
    }
  }
  /****************************************************
   * @brief : Compute goal-directed attention
   ****************************************************/
  void computeAttention()
  {
    supervisor_msgs::AgentActivity robotState;
    //toaster_msgs::Entity current_entity;
    head_manager::Signal current_signal;
    geometry_msgs::PointStamped goal_directed_attention;
    bool succeed=false;
    float temporal_factor=0.2;

    if(!agent_activity_map_.empty())
    {
      if (agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
      {
        //process
        robotState = agent_activity_map_.find(my_id_)->second;
        if (robotState.activityState=="ACTING")
        {
          focus_= 1.0;
          current_entity_=getEntity(robotState.object);
        }else {
          // If the robot is waiting we allow reactive input by setting focus to zero
          focus_= 0.0;
        }
        // If signal received
        if (!signal_list_.empty())
        {
          focus_ = 1.0;
          updateSignalList();
          if(selectSignal(current_signal))
          {
            signal_start_time_ = ros::Time::now();
            if(signalling_==true)
            {
              //signal interuption
              signal_pub_.publish(current_signal_);
            } else {
              signalling_=true;
            }
            current_signal_=current_signal;
          }
        }

        if (signalling_=true)
        {
          if (current_signal_it_ < current_signal_.entities.size())
          {
            focus_=1.0;
            ros::Duration duration(current_signal_.durations[current_signal_it_]);
            if(ros::Time::now() > signal_start_time_+ duration)
            {
              if(current_signal_it_==current_signal_.entities.size()-1)
              {
                current_signal_it_=0;
                signalling_=false;
                focus_=0.0;
              }else{
                current_signal_it_++;
                signal_start_time_ = ros::Time::now();
              }
            }
            current_entity_=getEntity(current_signal.entities[current_signal_it_]);
          }
        }
        //Publish
        goal_directed_attention.header.stamp = ros::Time::now();
        goal_directed_attention.header.frame_id = "map";
        goal_directed_attention.point = current_entity_.pose.position;
        goal_directed_attention_pub_.publish(goal_directed_attention);
        head_manager::Focus focus;
        focus.header=goal_directed_attention.header;
        focus.data=focus_;
        focus_pub_.publish(focus);
      }
    }
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
    supervisor_msgs::AgentActivity robotState;

    if(agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
    {
      robotState = agent_activity_map_.find(my_id_)->second;
      if (robotState.activityState=="ACTING")
      {
        if(id!=my_id_ && msg->activityState=="ACTING" && msg->unexpected==false)
        {
          //send signal
          head_manager::Signal sig;
          sig.entities[0]=msg->object;
          sig.durations[0]=0.2;
          sig.entities[1]=id+"::rightHand";
          sig.durations[1]=0.0;
          sig.entities[2]=id+"::head";
          sig.durations[2]=0.5;
          sig.urgency=0.98;
          sig.importancy=1.0;
        }
        if(id!=my_id_ && msg->activityState=="ACTING" && msg->unexpected==true)
        {
          //send signal
          head_manager::Signal sig;
          sig.entities[0]=msg->object;
          sig.durations[0]=0.2;
          sig.urgency=0.98;
          sig.importancy=0.9;
        }
      }
    }
    // Update activity map
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
  void objectListCallback(const toaster_msgs::ObjectListStamped::ConstPtr& msg)
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
  void robotListCallback(const toaster_msgs::RobotListStamped::ConstPtr& msg)
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
          ros::Subscriber sub = node_.subscribe<supervisor_msgs::AgentActivity>((const string)topicName, 1,boost::bind(&GoalDirectedAttention::activityCallback,this,_1,id));
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
  void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg)
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
          ros::Subscriber sub = node_.subscribe<supervisor_msgs::AgentActivity>((const string)topicName, 1,boost::bind(&GoalDirectedAttention::activityCallback,this,_1,id));
          agent_activity_sub_map_.insert(SubscriberPair_t(id,sub));
        }
      }
    }
  }
  /****************************************************
   * @brief : Update the fact list
   * @param : fact list
   ****************************************************/
  void factListCallback(const toaster_msgs::FactList::ConstPtr& msg)
  {
    if (!msg->factList.empty())
    {
      fact_list_.clear();
      for (unsigned int i = 0; i < msg->factList.size(); ++i)
      {
        fact_list_.push_back(*(new toaster_msgs::Fact(msg->factList[i])));
      }
    }
    try
    {
      updateAckMap();
      computeAttention();
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update the signal list
   * @param : signal received
   ****************************************************/
  void signalCallback(const head_manager::Signal::ConstPtr& msg)
  {
    signal_list_.push_back(*msg);
  }

public:
  
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
  GoalDirectedAttention * co = new GoalDirectedAttention(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}