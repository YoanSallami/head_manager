#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include <pr2motion/Head_Stop.h>
#include <pr2motion/Z_Head_SetMinDuration.h>
#include <geometry_msgs/PointStamped.h>

#include "../include/head_manager/HeadManagerException.h"
#include "geometry_msgs/PointStamped.h"

#include "toaster_msgs/ToasterHumanReader.h"
#include "toaster_msgs/ToasterRobotReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <head_manager/StimulusDrivenAttentionConfig.h>
#include "head_manager/AttentionStamped.h"

#include "head_manager/MapStamped.h"

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

using namespace std;
namespace msm = boost::msm;
namespace mpl = boost::mpl;

//for convenience
typedef std::map < std::string, float > SaliencyMap_t;
typedef std::pair < std::string, float > SaliencyPair_t;
typedef std::vector < toaster_msgs::Fact > FactList_t;
typedef dynamic_reconfigure::Server<head_manager::StimulusDrivenAttentionConfig> ParamServer_t;
typedef actionlib::SimpleActionClient<pr2motion::InitAction> InitActionClient_t;
typedef actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction> HeadActionClient_t;

// Forward class definition
class RobotObserver;

/**************************************************
* State machine definition
***************************************************/
// Events definition
struct humanNear{};
struct humanNotNear{};
struct humanHandOnTable{};
struct humanHandNotOnTable{};
struct humanHandStop{};
struct humanHandMove{};

static char const* const state_names[] = { "Waiting", "LookingHead", "LookingHand" };
/**
* @brief : State machine front definition
*/
struct ObserverStateMachine_ : public msm::front::state_machine_def<ObserverStateMachine_>
{
  RobotObserver * observer_ptr_; //<! 
  ObserverStateMachine_(RobotObserver * observer)
  {
    observer_ptr_ = observer;
  }
  // Starting state machine messages
  template <class Event,class FSM>
  void on_entry(Event const& ,FSM&) 
  {
      ROS_INFO("[robot_observer] Starting state machine.");
  }

  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) 
  {
      ROS_INFO("[robot_observer] Ending state machine.");
  }

  // States definition
  struct Waiting : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Entering state: \"Waiting\".");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Leaving state: \"Waiting\".");}
  };

  struct LookingHead : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Entering state: \"LookingHead\".");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Leaving state: \"LookingHead\".");}
  };

  struct LookingHand : public msm::front::state<> 
  {
      // every (optional) entry/exit methods get the event passed.
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Entering state: \"LookingHand\".");}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {}//ROS_INFO("[robot_observer] Leaving state: \"LookingHand\".");}
  };
  // Initial state definition
  typedef Waiting initial_state;
  // Transition action definition
  void focus_head(humanNear const&);
  void refocus_head(humanHandNotOnTable const&); 
  void focus_hand(humanHandOnTable const&);
  void rest(humanNotNear const&);
  void ack(humanHandStop const&);
  bool enable_ack_end(humanHandOnTable const&);
  // Guard transition definition

  typedef ObserverStateMachine_ sm;

  // Transition table
  struct transition_table : mpl::vector<
       //    Start                  Event                 Next                   Action                     Guard
       //  +----------------------+---------------------+----------------------+---------------------------+------------------------------------+
     a_row < Waiting              , humanNear           , LookingHead          , &sm::focus_head                                                 >,
    a_irow < Waiting              , humanNotNear                               , &sm::rest                                                       >,
       //  +----------------------+---------------------+----------------------+---------------------------+------------------------------------+
     a_row < LookingHead          , humanNotNear        , Waiting              , &sm::rest                                                       >,
       row < LookingHead          , humanHandOnTable    , LookingHand          , &sm::focus_hand           , &sm::enable_ack_end                 >,
    a_irow < LookingHead          , humanNear                                  , &sm::focus_head                                                 >,
       //  +----------------------+-----------------+--------------------------+---------------------------+------------------------------------+
     a_row < LookingHand          , humanHandNotOnTable , LookingHead          , &sm::refocus_head                                               >,
     //a_row < LookingHand          , humanHandStop       , LookingHead          , &sm::ack                                                        >,
    a_irow < LookingHand          , humanHandOnTable                           , &sm::focus_hand                                                 >
      //  +-----------------------+---------------------+-----------------------+---------------------------+------------------------------------+
    > {};

  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
      //ROS_INFO("[robot_observer] No transition from state %s on event %s",state_names[state],typeid(e).name());
  }
};
// State machine back definition
typedef msm::back::state_machine<ObserverStateMachine_> ObserverStateMachine;
// Testing utilities

void pstate(ObserverStateMachine const& sm)
{
    ROS_INFO(" -> %s", state_names[sm.current_state()[0]]);
}

class RobotObserver
{
public:
  FactList_t fact_list_; //!< fact list from agent_monitor
  FactList_t fact_area_list_; //!< fact list from area_manager
  bool human_is_moving_;
  bool enable_event_;
  ros::Timer waiting_timer_;
private:
  
  string my_id_; //!< robot id
  ros::NodeHandle node_; //!< node handler
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber fact_area_list_sub_;
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Publisher attention_pub_; //!< sensitive goal publisher
  ros::Publisher attention_vizu_pub_; //!< sensitive goal publisher
  ParamServer_t stimulu_driven_dyn_param_srv; //!<
  std::string attention_id_; //!<
  geometry_msgs::Point attention_point_; //!<
  ObserverStateMachine * state_machine_; //!<
  ros::ServiceClient connect_port_srv_;
  ros::ServiceClient head_stop_srv_;
  InitActionClient_t * init_action_client_; //!< initialisation client
  HeadActionClient_t * head_action_client_; //!< interface to head controller client
  geometry_msgs::Point head_position_; //!< 
  geometry_msgs::Point hand_position_; //!< 
  ros::Time stop_moving_start_time_; //!<
  bool timer_on_;
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  RobotObserver(ros::NodeHandle& node)
  {
    node_=node;
    
    // Getting robot's id from ros param
    if(node_.hasParam("my_robot_id"))
    {
      node_.getParam("my_robot_id",my_id_);
    } else {
      my_id_="PR2_ROBOT";
    }
    // Advertise subscribers
    waiting_timer_ = node_.createTimer(ros::Duration(2.0), &RobotObserver::timerCallback, this, true);
    timer_on_=false;
    enable_event_=true;
    fact_list_sub_ = node_.subscribe("/agent_monitor/factList", 1, &RobotObserver::factListCallback, this);
    fact_area_list_sub_ = node_.subscribe("/area_manager/factList", 1, &RobotObserver::factListAreaCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &RobotObserver::humanListCallback, this);
    // Advertise publishers
    attention_vizu_pub_ = node_.advertise<geometry_msgs::PointStamped>("head_manager/attention_vizualisation", 5);

    init_action_client_ = new InitActionClient_t("pr2motion/Init", true);
    // Initialize action client for the action interface to the head controller
    head_action_client_ = new HeadActionClient_t("pr2motion/Head_Move_Target", true);
    // Connection to the pr2motion client
    connect_port_srv_ = node_.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    head_stop_srv_ = node_.serviceClient<pr2motion::connect_port>("pr2motion/Head_Stop");
    ROS_INFO("[robot_observer] Waiting for pr2motion action server to start.");
    init_action_client_->waitForServer(); //will wait for infinite time
    head_action_client_->waitForServer(); //will wait for infinite time
    ROS_INFO("[robot_observer] pr2motion action server started.");

    pr2motion::InitGoal goal_init;
    init_action_client_->sendGoal(goal_init);

    pr2motion::connect_port srv;
    srv.request.local = "joint_state";
    srv.request.remote = "joint_states";
    if (!connect_port_srv_.call(srv)){
      ROS_ERROR("[robot_observer] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect_port_srv_.call(srv)){
      ROS_ERROR("[robot_observer] Failed to call service pr2motion/connect_port");
    }
    pr2motion::Z_Head_SetMinDuration srv_MinDuration;
    srv_MinDuration.request.head_min_duration=0.6;
    if(!ros::service::call("/pr2motion/Z_Head_SetMinDuration",srv_MinDuration))
        ROS_ERROR("[robot_observer] Failed to call service /pr2motion/Z_Head_SetMinDuration");
    state_machine_ = new ObserverStateMachine(boost::cref(this));
    state_machine_->start();
    ROS_INFO("[robot_observer] Starting state machine, node ready !");
    human_is_moving_=false;
    stop_moving_start_time_=ros::Time::now();
  }
  /****************************************************
   * @brief : Default destructor
   ****************************************************/
  ~RobotObserver(){}

private:

  void lookAt(geometry_msgs::PointStamped p)
  {
    pr2motion::Head_Stop stop;
    if (!ros::service::call("pr2motion/Head_Stop", stop))
      ROS_ERROR("[sensitive_reorientation] Failed to call service pr2motion/Head_Stop");
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = p.header.frame_id;
    goal.head_target_x = p.point.x;
    goal.head_target_y = p.point.y;
    goal.head_target_z = p.point.z;

    attention_vizu_pub_.publish(p);

    head_action_client_->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(5.0));

    if (finishedBeforeTimeout)
    {
      actionlib::SimpleClientGoalState state = head_action_client_->getState();
      //ROS_INFO("[robot_observer] Action finished: %s", state.toString().c_str() );
    }
    else
      ROS_WARN("[robot_observer] Action did not finish before the time out.");
  }
  void timerCallback(const ros::TimerEvent& event)
  {
    if(!timer_on_)
    {
        timer_on_=true;
        ROS_INFO("[robot_observer] Timer ready to fire.");
    } else {
        //ROS_INFO("[robot_observer] Timer fired !");
        enable_event_=true;
    }
  }
  /****************************************************
   * @brief : Update the fact list provided by agent_monitor
   * @param : fact list
   ****************************************************/
  void factListCallback(const toaster_msgs::FactList::ConstPtr& msg)
  {
      if (!msg->factList.empty())
      {
            bool human_is_moving=false;
            for (unsigned int i = 0; i < msg->factList.size(); ++i)
            {
              if (msg->factList[i].property=="IsMoving"
                  && msg->factList[i].subjectId=="rightHand")
                  {
                    //ROS_INFO("HUMAN HAND MOVING");
                    human_is_moving=true;
                  }
            }
            
            if(human_is_moving)
            {
                //ROS_INFO("[robot_observer] start time human hand stop");
                stop_moving_start_time_=ros::Time::now();    
            }else{
                if(ros::Time::now()-stop_moving_start_time_>ros::Duration(2.0))
                {
                    //ROS_INFO("[robot_observer] HUMAN HAND STOP");
                    state_machine_->process_event(humanHandStop());
                    human_is_moving_=false;
                } else {
                    //ROS_INFO("[robot_observer] HUMAN HAND MOVING");
                    //state_machine_->process_event(humanHandMove());
                    human_is_moving_=true;
                }
            }
       }
  }
  /****************************************************
   * @brief : Update the fact list provided by area_manager
   * @param : fact list
   ****************************************************/
  void factListAreaCallback(const toaster_msgs::FactList::ConstPtr& msg)
  {
    bool human_near=false;
    bool hand_on_table=false;
    for (unsigned int i = 0; i < msg->factList.size(); ++i)
    {
      if (msg->factList[i].property=="IsInArea" 
          && msg->factList[i].targetId=="interaction" 
          && msg->factList[i].subjectId=="HERAKLES_HUMAN1")
      {
        human_near=true;
      }
      if (msg->factList[i].property=="IsInArea" 
          && msg->factList[i].targetId=="action" 
          && msg->factList[i].subjectId=="rightHand")
      {
        hand_on_table=true;
      }
      
    }
    if(human_near)
    {
        //ROS_INFO("[robot_observer] process event HumanNear");
        state_machine_->process_event(humanNear());
    } else {
        //ROS_INFO("[robot_observer] process event HumanNotNear");
        state_machine_->process_event(humanNotNear());
    }
    if(hand_on_table)
    {
        //ROS_INFO("[robot_observer] process event humanHandOnTable");
        state_machine_->process_event(humanHandOnTable());
    } else {
        //ROS_INFO("[robot_observer] process event humanHandNotOnTable");
        state_machine_->process_event(humanHandNotOnTable());
    }
  }
  /****************************************************
   * @brief : Update the human list
   * @param : human list
   ****************************************************/
  void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg)
  {
    try
    {
      if (!msg->humanList.empty())
      {
        for (unsigned int i = 0; i < msg->humanList.size(); ++i)
        {
            if(msg->humanList[i].meAgent.meEntity.id=="HERAKLES_HUMAN1")
                for( unsigned int j = 0 ; j < msg->humanList[i].meAgent.skeletonJoint.size() ; ++j )
                {
                    if(msg->humanList[i].meAgent.skeletonNames[j]=="head")
                        head_position_=msg->humanList[i].meAgent.skeletonJoint[j].meEntity.pose.position;
                    if(msg->humanList[i].meAgent.skeletonNames[j]=="rightHand")
                        hand_position_=msg->humanList[i].meAgent.skeletonJoint[j].meEntity.pose.position;
                }
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
    }
  }
public:
  void rest()
  {
    //ROS_INFO("[robot_observer] Rest");
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.header.stamp = ros::Time::now();
    point.point.x = 3; 
    point.point.y = 0; 
    point.point.z = 0.6;
    lookAt(point);
  }
  void focusHead()
  {
    //ROS_INFO("[robot_observer] Focus head");
    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.point=head_position_;
    lookAt(point);
  }
  void focusHand()
  {
    //ROS_INFO("[robot_observer] Focus hand");
    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.point=hand_position_;
    lookAt(point);
  }
  
};

void ObserverStateMachine_::rest(humanNotNear const&)
{
  try
  {
    observer_ptr_->rest();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
  }
}

void ObserverStateMachine_::focus_head(humanNear const&)
{
  try
  {
    observer_ptr_->focusHead();

  } catch (HeadManagerException& e ) {
    ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
  }
}

void ObserverStateMachine_::refocus_head(humanHandNotOnTable const&)
{
  try
  {
    observer_ptr_->focusHead();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
  }
}

void ObserverStateMachine_::focus_hand(humanHandOnTable const&)
{
  try
  {
    observer_ptr_->focusHand();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
  }
}

bool ObserverStateMachine_::enable_ack_end(humanHandOnTable const&)
{
  return(observer_ptr_->enable_event_);// && observer_ptr_->human_is_moving_);
}

void ObserverStateMachine_::ack(humanHandStop const&)
{
  try
  {
    observer_ptr_->enable_event_=false;
    observer_ptr_->waiting_timer_.setPeriod(ros::Duration(1.0));
    observer_ptr_->waiting_timer_.start();
    observer_ptr_->focusHead();
  } catch (HeadManagerException& e ) {
    ROS_ERROR("[robot_observer] Exception was caught : %s",e.description().c_str());
  }
}

/****************************************************
 * @brief : Main process function
 * @param : arguments count
 * @param : arguments values
 ****************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_observer");
  ros::NodeHandle n;
  RobotObserver * ro = new RobotObserver(n);
  while(ros::ok())
  {
    ros::spinOnce();    
  }
}
