#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "head_manager/Sync.h"
#include "head_manager/Signal.h"
#include "head_manager/Focus.h"
#include "head_manager/AttentionStamped.h"
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
#include "tf/transform_datatypes.h"

#include "supervisor_msgs/AgentActivity.h"

#include <dynamic_reconfigure/server.h>
#include <head_manager/GoalDirectedAttentionConfig.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

using namespace std;
namespace msm = boost::msm;
namespace mpl = boost::mpl;

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
typedef dynamic_reconfigure::Server<head_manager::GoalDirectedAttentionConfig> ParamServer_t;

// Forward class definition
class GoalDirectedAttention;

/**************************************************
* Goal-directed state machine definition
***************************************************/
// Events definition
struct acting{};
struct waiting{};
struct signaling{};
struct start_signaling{
  start_signaling(head_manager::Signal signal_received):signal_received(signal_received){}
  head_manager::Signal signal_received;
};
struct stop_signaling{};

static char const* const state_names[] = { "Waiting", "Acting", "SignalingFromActing", "SignalingFromWaiting" };
// State machine front definition
struct GoalDirectedAttentionStateMachine_ : public msm::front::state_machine_def<GoalDirectedAttentionStateMachine_>
{
  GoalDirectedAttention * goal_directed_attention_; //<! 
  GoalDirectedAttentionStateMachine_(GoalDirectedAttention * goal_directed_attention)
  {
    goal_directed_attention_ = goal_directed_attention;
  }
  // Starting state machine messages
  template <class Event,class FSM>
  void on_entry(Event const& ,FSM&) 
  {
      ROS_INFO("[goal_directed_attention] Starting goal-directed state machine.");
  }

  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) 
  {
      ROS_INFO("[goal_directed_attention] Ending goal-directed state machine.");
  }

  // States definition
  struct Waiting : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Entering state: Waiting.");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Leaving state: Waiting.");}
  };

  struct Acting : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Entering state: Acting.");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Leaving state: Acting.");}
  };

  struct SignalingFromActing : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Entering state: SignalingFromActing.");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Leaving state: SignalingFromActing.");}
  };

  struct SignalingFromWaiting : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Entering state: SignalingFromWaiting.");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {ROS_INFO("[goal_directed_attention] Leaving state: SignalingFromWaiting.");}
  };

  // Initial state definition
  typedef Waiting initial_state;
  // Transition action definition
  void focus_action(acting const&); 
  void allow_distraction(waiting const&); 
  void start_focus_signal(start_signaling const&);
  void stop_focus_signal(stop_signaling const&);
  void focus_signal(signaling const&);
  // Guard transition definition
  bool signalUrgent(start_signaling const&);
  bool actionStopableAndSignalUrgent(start_signaling const&);
  bool signalAcknowledgement(stop_signaling const&);

  typedef GoalDirectedAttentionStateMachine_ sm;

  // Transition table for player
  struct transition_table : mpl::vector<
       //    Start                  Event             Next                   Action                     Guard
       //  +----------------------+-----------------+----------------------+---------------------------+------------------------------------+
     a_row < Waiting              , acting          , Acting               , &sm::focus_action                                               >,
     a_row < Waiting              , start_signaling , SignalingFromWaiting , &sm::start_focus_signal                                         >,
    a_irow < Waiting              , waiting                                , &sm::allow_distraction                                          >,
       //  +----------------------+-----------------+----------------------+---------------------------+------------------------------------+
     a_row < Acting               , waiting         , Waiting              , &sm::allow_distraction                                          >,
       row < Acting               , start_signaling , SignalingFromActing  , &sm::start_focus_signal   , &sm::actionStopableAndSignalUrgent  >,
    a_irow < Acting               , acting          ,                        &sm::focus_action                                               >,
       //  +----------------------+-----------------+----------------------+---------------------------+------------------------------------+
       row < SignalingFromActing  , stop_signaling  , Acting               , &sm::stop_focus_signal    , &sm::signalAcknowledgement          >,
      irow < SignalingFromActing  , start_signaling                        , &sm::start_focus_signal   , &sm::signalUrgent                   >,
    a_irow < SignalingFromActing  , signaling                              , &sm::focus_signal                                               >,
       //  +----------------------+-----------------+----------------------+---------------------------+------------------------------------+
       row < SignalingFromWaiting , stop_signaling  , Waiting              , &sm::stop_focus_signal    , &sm::signalAcknowledgement          >,
    a_irow < SignalingFromWaiting , start_signaling ,                        &sm::start_focus_signal                                         >,
    a_irow < SignalingFromWaiting , signaling                              , &sm::focus_signal                                               >
      //  +-----------------------+-----------------+----------------------+---------------------------+------------------------------------+
    > {};

  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
      ROS_INFO("no transition from state %s on event %s",state_names[state],typeid(e).name());
  }
};
// State machine back definition
typedef msm::back::state_machine<GoalDirectedAttentionStateMachine_> GoalDirectedAttentionStateMachine;
// Testing utilities

void pstate(GoalDirectedAttentionStateMachine const& sm)
{
    ROS_INFO(" -> %s", state_names[sm.current_state()[0]]);
}

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
  SignalList_t signal_list_; //!< signal list from supervisor
  ActivityMap_t agent_activity_map_; //!< agent activity state machines map
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber robot_list_sub_; //!< robot list subscriber
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber signal_sub_; //!< signal subscriber
  ros::Publisher goal_directed_attention_pub_; //!< goal_directed attention publisher
  ros::Publisher goal_directed_attention_vizu_pub_; //!< goal_directed attention publisher
  ros::Publisher signal_pub_; //!< signal publisher
  ros::Publisher focus_pub_; //!< focus publisher
  int current_signal_it_; //!< 
  ros::Time signal_it_time_; //!<
  GoalDirectedAttentionStateMachine * state_machine_; //!<
  double urgencyThreshold_; //!<
  ParamServer_t goal_directed_dyn_param_srv; //!<
  AckMap_t ack_map_; //!< acknowledgement map;
  head_manager::Signal current_signal_; //!<
  bool signaling_; //!<

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
      my_id_="PR2_ROBOT";
    }
    // Advertise subscribers
    object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &GoalDirectedAttention::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &GoalDirectedAttention::humanListCallback, this);
    robot_list_sub_ = node_.subscribe("/pdg/robotList", 1, &GoalDirectedAttention::robotListCallback, this);
    fact_list_sub_ = node_.subscribe("/pdg/factList", 1, &GoalDirectedAttention::factListCallback, this);
    signal_sub_ = node_.subscribe("head_manager/signal", 10, &GoalDirectedAttention::signalCallback, this);
    // Advertise publishers
    goal_directed_attention_pub_ = node_.advertise<head_manager::AttentionStamped>("head_manager/goal_directed_attention", 5);
    goal_directed_attention_vizu_pub_ = node_.advertise<geometry_msgs::PointStamped>("head_manager/goal_directed_attention_visualization", 5);
    focus_pub_ = node_.advertise <head_manager::Focus> ("head_manager/focus", 5);
    signal_pub_ = node_.advertise <head_manager::Signal> ("head_manager/signal", 10);
    // Dyn param server
    goal_directed_dyn_param_srv.setCallback(boost::bind(&GoalDirectedAttention::dynParamCallback, this, _1, _2));
    //surprised_=false;
    current_signal_it_=0;
    urgencyThreshold_=0.8;
    signaling_=false;
    state_machine_ = new GoalDirectedAttentionStateMachine(boost::cref(this));
    state_machine_->start();
    supervisor_msgs::AgentActivity my_agent;
    my_agent.activityState="IDLE";
    my_agent.object={""};
    my_agent.importancy=0.0;
    my_agent.unexpected=false;
    my_agent.stopable=true;
    agent_activity_map_.insert(ActivityPair_t(my_id_,my_agent));
    ROS_INFO("[goal_directed_attention] Adding %s to agent activity map",my_id_.c_str());
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
          it->weight-=it->weight*(1.0-it->urgency);
        } else {
          it->weight-=it->weight;
        }
        if(it->weight<0.000001)
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
    double max=sig.weight;
    SignalList_t::iterator max_it;
    bool changed=false;
    if (!signal_list_.empty())
    {
      for (SignalList_t::iterator it = signal_list_.begin(); it != signal_list_.end(); ++it)
      {
        if(it->weight > max)
        {
          max_it=it;
          max=it->weight;
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
public:
  void focusOnAction()
  {
    head_manager::AttentionStamped goal_directed_attention;
    geometry_msgs::PointStamped goal_directed_attention_vizu;
    head_manager::Focus focus;
    supervisor_msgs::AgentActivity robotActivityState;

    if(!agent_activity_map_.empty())
    {
      if (agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
      {
        robotActivityState = agent_activity_map_.find(my_id_)->second;
        if (robotActivityState.activityState=="ACTING")
        {
          ROS_INFO("[goal_directed_attention] Receiving activity object : %s",robotActivityState.object.c_str());
          goal_directed_attention_vizu.point = getEntity(robotActivityState.object).pose.position;
          goal_directed_attention_vizu.header.stamp = ros::Time::now();
          goal_directed_attention_vizu.header.frame_id = "map";

          goal_directed_attention.header = goal_directed_attention_vizu.header;
          goal_directed_attention.point = goal_directed_attention_vizu.point;
          goal_directed_attention.id = robotActivityState.object;
          goal_directed_attention.object = isObject(robotActivityState.object);

          focus.header=goal_directed_attention.header;
          focus.data=1.0;

          goal_directed_attention_vizu_pub_.publish(goal_directed_attention_vizu);
          goal_directed_attention_pub_.publish(goal_directed_attention);
          focus_pub_.publish(focus);
        } else {
          throw HeadManagerException("Could not focus to action, robot is not acting anymore !");
        }
      } else {
        throw HeadManagerException ("Could not read the robot activity state.");
      }
    } else {
      throw HeadManagerException ("Could not read an empty activity state map.");
    }
  }

  void allowDistraction()
  {
    geometry_msgs::PointStamped goal_directed_attention_vizu;
    head_manager::AttentionStamped goal_directed_attention;
    geometry_msgs::Vector3 tempPoint;
    tf::Vector3 tempPointTF;
    geometry_msgs::Vector3 resultVec;
    tf::Vector3 resultVecTF;
    head_manager::Focus focus;
    tf::Quaternion q;

    tempPoint.x = 1.0;
    tempPoint.y = 0.0;
    tempPoint.z = 1.2;
    
    tf::vector3MsgToTF(tempPoint,tempPointTF);
    
    tf::quaternionMsgToTF(getRobot(my_id_).pose.orientation,q);
  
    resultVecTF = tf::quatRotate((const tf::Quaternion)q,(const tf::Vector3)tempPointTF);
    tf::vector3TFToMsg(resultVecTF,resultVec);

    goal_directed_attention_vizu.point.x = resultVec.x+getRobot(my_id_).pose.position.x;
    goal_directed_attention_vizu.point.y = resultVec.y+getRobot(my_id_).pose.position.y;
    goal_directed_attention_vizu.point.z = resultVec.z+getRobot(my_id_).pose.position.z;
    goal_directed_attention_vizu.header.stamp = ros::Time::now();
    goal_directed_attention_vizu.header.frame_id = "map";

    goal_directed_attention.header = goal_directed_attention_vizu.header;
    goal_directed_attention.point = goal_directed_attention_vizu.point;
    goal_directed_attention.object = false;
    goal_directed_attention.id = "Waiting";

    focus.header=goal_directed_attention.header;
    focus.data=0.0;
    
    goal_directed_attention_vizu_pub_.publish(goal_directed_attention_vizu);
    goal_directed_attention_pub_.publish(goal_directed_attention);
    focus_pub_.publish(focus);
  }
  void startSignalFocusing()
  {
    signaling_=true;
    current_signal_it_=0;
    signal_it_time_=ros::Time::now();
  }
  void stopSignalFocusing()
  {
    signaling_=false;
  }
  void focusOnSignal()
  {
    geometry_msgs::PointStamped goal_directed_attention_vizu;
    head_manager::AttentionStamped goal_directed_attention;
    head_manager::Focus focus;
    if (current_signal_.entities.size()==current_signal_.durations.size())
    {
      if (current_signal_it_ <= current_signal_.entities.size()-1)
      {
        ROS_INFO("IT :%d",current_signal_it_);
        ros::Duration duration(current_signal_.durations[current_signal_it_]);
        if(ros::Time::now() > signal_it_time_+ duration)
        {
          //ROS_ERROR("Changement focus signal");
          if (current_signal_it_ == current_signal_.entities.size()-1)
          {
            ROS_ERROR("STOP SIGNALING");
            state_machine_->process_event(stop_signaling());
            ;;
            return;
          } else {
            current_signal_it_++;
            signal_it_time_ = ros::Time::now();
          } 
        }
        goal_directed_attention_vizu.header.stamp = ros::Time::now();
        goal_directed_attention_vizu.header.frame_id = "map";
        goal_directed_attention_vizu.point = getEntity(current_signal_.entities[current_signal_it_]).pose.position;

        goal_directed_attention.header = goal_directed_attention_vizu.header;
        goal_directed_attention.point = goal_directed_attention_vizu.point;
        goal_directed_attention.object = isObject(current_signal_.entities[current_signal_it_]);
        goal_directed_attention.id = current_signal_.entities[current_signal_it_];

        focus.header=goal_directed_attention.header;
        focus.data=1.0;

        goal_directed_attention_vizu_pub_.publish(goal_directed_attention_vizu);
        goal_directed_attention_pub_.publish(goal_directed_attention);
        focus_pub_.publish(focus);
      }
    } else {
      state_machine_->process_event(stop_signaling());
      throw HeadManagerException ("Current signal bad format !");
    }
  }
  bool isSignalUrgent(head_manager::Signal signal)
  {
    supervisor_msgs::AgentActivity robotState;
    if (!agent_activity_map_.empty())
    {
      if (agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
      {
        robotState = agent_activity_map_.find(my_id_)->second;
        if (signal.importancy>robotState.importancy)
        {
          if (signal.urgency>urgencyThreshold_)
          {
            return (true);
          }
        }
      } else {
        throw HeadManagerException ("Could not read the robot activity state.");
      }
    } else {
      throw HeadManagerException ("Could not read an empty activity state map.");
    }
    return(false);
  }
  bool isActionStopable()
  {
    supervisor_msgs::AgentActivity robotState;
    if (!agent_activity_map_.empty())
    {
      if (agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
      {
        robotState = agent_activity_map_.find(my_id_)->second;
        if(robotState.stopable==true)
        {
          return (true);
        } else {
          return(false);
        }
      } else {
        throw HeadManagerException ("Could not read the robot activity state.");
      }
    } else {
      throw HeadManagerException ("Could not read an empty activity state map.");
    }
  }
  bool signalAcknowledgement()
  {
    if(current_signal_.receivers.size()>0)
    {
      for (int i = 0; i < current_signal_.receivers.size(); ++i)
      {
        if(ack_map_.find(current_signal_.receivers[i])==ack_map_.end())
        {
          return(false);
          if(ack_map_.find(current_signal_.receivers[i])->second!=true)
          {
            return(false);
          }
        }
      }
    }
    return(true);
  }
private:
  /****************************************************
   * @brief : Update the activity state map
   * @param : object list
   ****************************************************/
  void activityCallback(const supervisor_msgs::AgentActivity::ConstPtr& msg,std::string id)
  {
    bool find=false;
    ActivityMap_t::iterator it = agent_activity_map_.begin();
    supervisor_msgs::AgentActivity robotState;
    bool robotWasActing=false;
    bool robotIsActing=false;
    head_manager::Signal sig;
    //ROS_INFO("[goal_directed_attention] Receiving activity state from %s",id.c_str());
    
    if (agent_activity_map_.find(id)!=agent_activity_map_.end())
    {
      agent_activity_map_.find(id)->second=*msg;
    } else {
      ROS_INFO("[goal_directed_attention] Adding %s to agent activity map",id.c_str());
      agent_activity_map_.insert(ActivityPair_t(id,*msg));
    }

    if (agent_activity_map_.find(my_id_)!=agent_activity_map_.end())
    {
      //ROS_INFO("test");
      if (signaling_==true)
      {
        ROS_INFO("[goal_directed_attention] Sending signaling event");
        state_machine_->process_event(signaling());
      } else {
        if (agent_activity_map_.find(my_id_)->second.activityState=="ACTING" &&
            agent_activity_map_.find(my_id_)->second.object!="")
        {
          //ROS_INFO("[goal_directed_attention] Sending acting event");
          state_machine_->process_event(acting());

          if (id!=my_id_ && msg->activityState=="ACTING")
          {
            if (msg->unexpected==false)
            {
              // If a human do an expected action regarding to the plan during
              // robot action we send a signal to queue
              sig.entities[0]=msg->object;
              sig.durations[0]=0.2;
              sig.urgency=0.98;
              sig.importancy=0.9;
            } else {
              // If a human do an unexpected action regarding to the plan during
              // robot action we send a signal to queue
              sig.entities[0]=msg->object;
              sig.durations[0]=0.2;
              sig.entities[1]=id+"::rightHand";
              sig.durations[1]=0.0;
              sig.entities[2]=id+"::head";
              sig.durations[2]=0.5;
              sig.urgency=0.68;
              sig.importancy=1.0;
            }
            signal_pub_.publish(sig);
          }
        } else {
          //ROS_INFO("[goal_directed_attention] Sending waiting event");
          state_machine_->process_event(waiting());
        }
      }
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
        if (msg->objectList[i].meEntity.id!="unknown_object")
        {
          object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));
        }
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
        toaster_msgs::Robot robot(msg->robotList[i]);
        if (robot.meAgent.meEntity.id =="pr2")
        {
          robot.meAgent.meEntity.id ="PR2_ROBOT";
        }
        robot_list_.push_back(*(new toaster_msgs::Robot(robot)));
        std::string id = robot.meAgent.meEntity.id;
        
        std::string topicName = "/supervisor/activity_state/"+id;
        if (agent_activity_sub_map_.find(id) == agent_activity_sub_map_.end())
        {
          ROS_INFO("[goal_directed_attention] Subscribing to %s robot activity state",id.c_str());
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
          ROS_INFO("[goal_directed_attention] Subscribing to %s human activity state",id.c_str());
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
    updateSignalList();
    if (selectSignal(current_signal_)==true)
    {
      state_machine_->process_event(start_signaling(current_signal_));
    }
  }

  void dynParamCallback(head_manager::GoalDirectedAttentionConfig &config, uint32_t level) 
  {
    urgencyThreshold_ = config.urgency_threshold;
  }
};

void GoalDirectedAttentionStateMachine_::focus_action(acting const&)
{
  try
  {
    goal_directed_attention_->focusOnAction();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
  }
}

void GoalDirectedAttentionStateMachine_::allow_distraction(waiting const&)
{
  try
  {
    goal_directed_attention_->allowDistraction();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
  }
}

void GoalDirectedAttentionStateMachine_::start_focus_signal(start_signaling const&)
{
  try
  {
    goal_directed_attention_->startSignalFocusing(); 
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
  }
}

void GoalDirectedAttentionStateMachine_::stop_focus_signal(stop_signaling const&)
{
  try
  {
    goal_directed_attention_->stopSignalFocusing(); 
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
  }
}

void GoalDirectedAttentionStateMachine_::focus_signal(signaling const&)
{
  try
  {
    goal_directed_attention_->focusOnSignal();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[goal_directed_attention] Exception was caught : %s",e.description().c_str());
  }
}

bool GoalDirectedAttentionStateMachine_::signalUrgent(start_signaling const& s)
{
  if (goal_directed_attention_->isSignalUrgent(s.signal_received)==true)
  {
    return(true);
  }
  return(false);
}
bool GoalDirectedAttentionStateMachine_::actionStopableAndSignalUrgent(start_signaling const& s)
{
  if (goal_directed_attention_->isActionStopable()==true)
  {
    if (goal_directed_attention_->isSignalUrgent(s.signal_received)==true)
    {
      return(true);
    }
    return(false);
  }
  return(false);
}
bool GoalDirectedAttentionStateMachine_::signalAcknowledgement(stop_signaling const&)
{
  goal_directed_attention_->signalAcknowledgement();
}
/****************************************************
 * @brief : Main process function
 * @param : arguments count
 * @param : arguments values
 ****************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_directed_attention");
  ros::NodeHandle n;
  GoalDirectedAttention * co = new GoalDirectedAttention(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}