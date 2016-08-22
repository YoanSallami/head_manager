#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "geometry_msgs/PointStamped.h"

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

#include <dynamic_reconfigure/server.h>
#include <head_manager/StimulusDrivenAttentionConfig.h>
#include "head_manager/AttentionStamped.h"

#include "head_manager/MapStamped.h"

using namespace std;

//for convenience
typedef std::map < std::string, float > SaliencyMap_t;
typedef std::pair < std::string, float > SaliencyPair_t;
typedef std::vector < toaster_msgs::Object > ObjectList_t;
typedef std::vector < toaster_msgs::Robot > RobotList_t;
typedef std::vector < toaster_msgs::Human > HumanList_t;
typedef std::vector < toaster_msgs::Fact > FactList_t;
typedef dynamic_reconfigure::Server<head_manager::StimulusDrivenAttentionConfig> ParamServer_t;

class StimulusDrivenAttention
{
public:
  SaliencyMap_t saliency_map_; //!< saliency map for stimuli selection
  ObjectList_t object_list_; //!< object list from pdg
  HumanList_t human_list_; //!< human list from pdg
  RobotList_t robot_list_; //!< robot list from pdg
  FactList_t fact_list_; //!< fact list from agent_monitor
  FactList_t fact_area_list_;
private:
  double stimuliDiscountFactor_; //!<
  double directionSalienceFactor_; //!<
  double movingSalienceFactor_; //!<
  double lookingSalienceFactor_; //!<
  double inhibitionRadius_; //!<
  double hysteresisMinThreshold_; //!<
  double hysteresisThresholdFactor_; //!<
  SaliencyPair_t salient_stimuli_;
  string my_id_; //!< robot's id
  ros::NodeHandle node_; //!< node handler
  ros::Subscriber fact_area_list_sub_;
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber robot_list_sub_; //!< robot list subscriber
  ros::Publisher salient_stimuli_pub_; //!< sensitive goal publisher
  ros::Publisher salient_stimuli_vizu_pub_; //!< sensitive goal publisher
  ros::Publisher saliency_map_pub_; //!< saliency map publisher
  ros::ServiceServer inhibition_of_return_srv_; //!< inhibition of return service
  ParamServer_t stimulu_driven_dyn_param_srv; //!<
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  StimulusDrivenAttention(ros::NodeHandle& node)
  {
    node_=node;
    // Setting reactive parameters to default
    stimuliDiscountFactor_ = 0.6;
    directionSalienceFactor_ = 0.0;
    movingSalienceFactor_ = 7.0;
    lookingSalienceFactor_ = 0.0;
    inhibitionRadius_ = 0.4;
    hysteresisMinThreshold_=2.5;
    hysteresisThresholdFactor_ = 0.2;
    // Getting robot's id from ros param
    if(node_.hasParam("my_robot_id"))
    {
      node_.getParam("my_robot_id",my_id_);
    } else {
      my_id_="PR2_ROBOT";
    }
    // Advertise subscribers
    object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &StimulusDrivenAttention::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &StimulusDrivenAttention::humanListCallback, this);
    robot_list_sub_ = node_.subscribe("/pdg/robotList", 1, &StimulusDrivenAttention::robotListCallback, this);
    fact_list_sub_ = node_.subscribe("/agent_monitor/factList", 1, &StimulusDrivenAttention::factListCallback, this);
    fact_area_list_sub_ = node_.subscribe("/area_manager/factList", 1, &StimulusDrivenAttention::factListAreaCallback, this);
    // Advertise publishers
    salient_stimuli_vizu_pub_ = node_.advertise<geometry_msgs::PointStamped>("head_manager/salient_stimuli_vizualisation", 5);
    salient_stimuli_pub_ = node_.advertise<head_manager::AttentionStamped>("head_manager/salient_stimuli", 5);
    saliency_map_pub_ = node_.advertise<head_manager::MapStamped>("head_manager/saliency_map",1);
    // Advertise services
    // Dyn param server
    stimulu_driven_dyn_param_srv.setCallback(boost::bind(&StimulusDrivenAttention::dynParamCallback, this, _1, _2));
    // Add a waiting attention zone to saliency map
    saliency_map_.insert(SaliencyPair_t("Waiting",0.0));
    salient_stimuli_=SaliencyPair_t("Waiting",0.0);
  }
  /****************************************************
   * @brief : Default destructor
   ****************************************************/
  ~StimulusDrivenAttention(){}
  /****************************************************
   * @brief : Update the saliency map using
   *          agent_monitor facts
   ****************************************************/
  void updateSaliencyMap()
  {
    toaster_msgs::Human human;
    toaster_msgs::Object object;
    float xPosition=0;
    float yPosition=0;
    float zPosition=0;
    SaliencyMap_t input_map(saliency_map_);
    SaliencyMap_t::iterator subject;
    SaliencyMap_t::iterator subjectOwner;
    SaliencyMap_t::iterator target;
    SaliencyMap_t::iterator joint;
    bool in_area=false;
    
    if (!input_map.empty())
    {
      for(SaliencyMap_t::iterator it_tm = input_map.begin() ; it_tm != input_map.end() ; ++it_tm )
      {
        it_tm->second=0.0;
      }
    }
    SaliencyMap_t directionSaliency_map(input_map);
    SaliencyMap_t movingSaliency_map(input_map);
    SaliencyMap_t lookingSaliency_map(input_map);
    SaliencyMap_t inhibition_map(input_map);
    // Saliency update according to motion facts provided by agent_monitor
    if(!fact_list_.empty())
    {
      for (FactList_t::iterator it_fl = fact_list_.begin() ; it_fl != fact_list_.end() ; ++it_fl )
      {
        if (it_fl->subjectId=="pr2")
        {
          it_fl->subjectId=my_id_;
        }
        if (it_fl->targetId=="pr2")
        {
          it_fl->targetId=my_id_;
        }
        if (it_fl->subjectOwnerId=="pr2")
        {
          it_fl->subjectOwnerId=my_id_;
        }
        if (it_fl->targetOwnerId=="pr2")
        {
          it_fl->targetOwnerId=my_id_;
        }
        if (it_fl->subjectId!="unknown_object" && it_fl->targetId!="unknown_object")
        {
          if (it_fl->subjectId!=my_id_)
          {
            if ( it_fl->property == "IsMovingToward" && it_fl->subProperty=="direction" && it_fl->subjectId=="rightHand")
            {
              //ROS_INFO("[DEBUG] %s is moving toward %s",it_fl->subjectId.c_str(),it_fl->targetId.c_str());
              in_area=false;
              for (FactList_t::iterator it_area = fact_area_list_.begin(); it_area < fact_area_list_.end(); ++it_area)
              {
                  if (it_area->property =="IsInArea")
                  {
                    if (it_area->subjectId==it_fl->subjectId && it_area->targetId=="action")
                    {
                      in_area=true;
                      //ROS_INFO("[DEBUG] %s is in area %s",it_fl->subjectId.c_str(),it_area->targetId.c_str());
                    }
                  }
              }
              if(in_area)
              {
                if (it_fl->targetId!=my_id_)
                {
                  target=directionSaliency_map.find(it_fl->targetId);
                  if ( target != directionSaliency_map.end() )
                  {
                    target->second+=it_fl->doubleValue;
                  } else {
                    //throw HeadManagerException ("Could not find "+it_fl->targetId+" in object saliency map.");
                  }
                }
              }
            }
            if ( it_fl->property == "IsLookingToward" )
            {
              if (it_fl->targetId!=my_id_)
              {
                target=lookingSaliency_map.find(it_fl->targetId);
                if ( target != lookingSaliency_map.end() )
                {
                  target->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->targetId+" in looking saliency map.");
                }
              }else{
                subject=lookingSaliency_map.find(it_fl->subjectId+"::head");
                if ( subject != lookingSaliency_map.end() )
                {
                  subject->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->subjectId+"::head in looking saliency map.");
                }
              }
            }
            if ( it_fl->property == "IsMoving")
            {
              if ( it_fl->subProperty == "joint")
              {
                subject=movingSaliency_map.find(it_fl->subjectOwnerId+"::"+it_fl->subjectId);
                if ( subject != movingSaliency_map.end() )
                {
                  subject->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in moving saliency map.");
                }
              }
              if ( it_fl->subProperty == "agent")
              { 
                subject=movingSaliency_map.find(it_fl->subjectId+"::head");
                if ( subject != movingSaliency_map.end() )
                {
                  subject->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in movingsaliency map.");
                }
              }
            }
          } else if (it_fl->subjectId==my_id_)
          {
            //Inhibition of return
            if (it_fl->property == "IsLookingToward" )
            {
              if(isHuman(it_fl->targetId))
              {
                // humans
                target=inhibition_map.find(it_fl->targetId+"::head");
                if ( target != inhibition_map.end() )
                {
                  target->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->targetId+"::head in inhibition map.");
                }
              } else {
                // ojects
                target=inhibition_map.find(it_fl->targetId);
                if ( target != inhibition_map.end() )
                {
                  target->second+=it_fl->doubleValue;
                } else {
                  //throw HeadManagerException ("Could not find "+it_fl->targetId+" in inhibition map.");
                }
              }//TODO add case for others robots
              
            }
          }
        }
      }

      if(!normalizeMap(inhibition_map))
        throw HeadManagerException ("Could not normalize object saliency map");
      if(!normalizeMap(directionSaliency_map))
        throw HeadManagerException ("Could not normalize object saliency map");
      if(!normalizeMap(movingSaliency_map))
        throw HeadManagerException ("Could not normalize joint saliency map");
      if(!normalizeMap(lookingSaliency_map))
        throw HeadManagerException ("Could not normalize looking saliency map");
      for (SaliencyMap_t::iterator it = input_map.begin(); it != input_map.end() ; ++it)
      {
        it->second=(directionSaliency_map.find(it->first)->second*directionSalienceFactor_)+
                   (movingSaliency_map.find(it->first)->second*movingSalienceFactor_)+
                   (lookingSaliency_map.find(it->first)->second*lookingSalienceFactor_);
      }
      if(!saliency_map_.empty())
      {
        for(SaliencyMap_t::iterator it_sm = saliency_map_.begin() ; it_sm != saliency_map_.end() ; ++it_sm )
        {
          it_sm->second*=stimuliDiscountFactor_; // Temporal filtering to reduce salience over time
          it_sm->second+=(input_map.find(it_sm->first)->second*(1-inhibition_map.find(it_sm->first)->second)); // Add normalized feature saliency
          if (it_sm->second<0.001)
          {
            it_sm->second=0.0;
          }
        }
      } else {
        throw HeadManagerException ( "Could not update an empty saliency map." );
      }
    }
    sendSaliencyMap();
    selectStimuli(salient_stimuli_);
  }
  /****************************************************
   * @brief : Send the saliency map throw his topic
   ****************************************************/
  void sendSaliencyMap()
  {
    head_manager::MapStamped map;
    map.header.stamp = ros::Time::now();
    SaliencyMap_t::iterator it;

    for (it = saliency_map_.begin(); it != saliency_map_.end() ; ++it)
    {
      map.key.push_back(it->first);
      map.value.push_back(it->second);
    }
    saliency_map_pub_.publish(map);
  }
  /****************************************************
   * @brief : Send the salient stimuli throw his topic
   ****************************************************/
  void sendSalientStimuli(SaliencyPair_t salient)
  {
    geometry_msgs::PointStamped salient_attention_point_vizu;
    head_manager::AttentionStamped salient_attention_point;
    
    geometry_msgs::Vector3 tempPoint;
    tf::Vector3 tempPointTF;
    tf::Quaternion q;
    if (!object_list_.empty())
    {
      if (!robot_list_.empty())
      {
        salient_attention_point_vizu.point = getEntity(salient.first).pose.position;
        
        salient_attention_point_vizu.header.frame_id="map";
        salient_attention_point_vizu.header.stamp=ros::Time::now();

        salient_attention_point.header = salient_attention_point_vizu.header;
        salient_attention_point.point = salient_attention_point_vizu.point;
        salient_attention_point.id = salient.first;
          salient_attention_point.object = isObject(salient.first);
      
        salient_stimuli_pub_.publish(salient_attention_point); 
        salient_stimuli_vizu_pub_.publish(salient_attention_point_vizu);  
      } else {
        throw HeadManagerException ("Could not read an empty robot list.");
      }
    } else {
      throw HeadManagerException ("Could not read an empty object list.");
    }
    /*
    if (!human_list_.empty())
    {
      salient_attention_point_vizu.point = getEntity("HERAKLES_HUMAN1::head").pose.position;
        
      salient_attention_point_vizu.header.frame_id="map";
      salient_attention_point_vizu.header.stamp=ros::Time::now();

      salient_attention_point.header = salient_attention_point_vizu.header;
      salient_attention_point.point = salient_attention_point_vizu.point;
      salient_attention_point.id = "HERAKLES_HUMAN1::head";
      salient_attention_point.object = isObject("HERAKLES_HUMAN1::head");
    
      salient_stimuli_pub_.publish(salient_attention_point); 
      salient_stimuli_vizu_pub_.publish(salient_attention_point_vizu);  
    }  else {
      throw HeadManagerException ("Could not read an empty human list.");
    }
    */
  }
private:
  bool normalizeMap(SaliencyMap_t& map)
  {
    SaliencyPair_t max;
    SaliencyMap_t::iterator it;
    if(!map.empty())
    {
      if(selectBestStimuli(max))
      {
        if (max.second > 0.0)
        {
          for(it = map.begin() ; it != map.end() ; ++it )
          {
            if (it->second != 0.0)
            {
              it->second = it->second / max.second;
              if(it->second == max.second)
              {
                it->second=1.0;
              }
            }
          }
        }
        return(true);
      }      
    }
    return(false);
  }
  /****************************************************
   * @brief : Select the best stimuli from saliency map
   * @param : best stimuli
   * @return : true if succeed
   ****************************************************/
  bool selectBestStimuli(SaliencyPair_t & best)
  {
    SaliencyMap_t::iterator it;
    SaliencyPair_t bestTemp = * saliency_map_.rbegin();

    if (!saliency_map_.empty())
    {
      for( it = saliency_map_.begin() ; it != saliency_map_.end() ; ++it )
      {
        if( it->second > bestTemp.second )
        {
          bestTemp=*it;
        }
      }
      best=bestTemp;
      return(true);
    } 
    return(false);
  }
  /****************************************************
   * @brief : Select better stimuli from saliency map by
   *          using an hysteresis
   * @param : best stimuli
   * @return : true if succeed
   ****************************************************/
  bool selectStimuli(SaliencyPair_t& stimuli)
  {
    SaliencyMap_t::iterator it;
    if (!saliency_map_.empty())
    {
      stimuli=*saliency_map_.find(stimuli.first);
      for( it = saliency_map_.begin() ; it != saliency_map_.end() ; ++it )
      {
        if (stimuli.first=="Waiting")
        {
          if( it->second >= stimuli.second)
          {
            stimuli = * it;
          }
        } else {
          if( it->second > (stimuli.second + hysteresisMinThreshold_ + (stimuli.second*hysteresisThresholdFactor_)))
          {
            stimuli = * it;
          }
        }
      }
      return(true);
    }
    return(false);
  }
  /****************************************************
   * @brief : Get entity from object/human/robot list
   * @param : entity's id
   * @return : entity
   ****************************************************/
  toaster_msgs::Entity getEntity(std::string id)
  {
    toaster_msgs::Entity entity;
    if(id!="Waiting")
    {
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
    }else{
      geometry_msgs::Vector3 tempPoint;
      tf::Vector3 tempPointTF;
      geometry_msgs::Vector3 resultVec;
      tf::Vector3 resultVecTF;
      toaster_msgs::Entity temp;
      tf::Quaternion q;
      tempPoint.x = 1.0;
      tempPoint.y = 0.0;
      tempPoint.z = 1.2;
      tf::vector3MsgToTF(tempPoint,tempPointTF);
      tf::quaternionMsgToTF(getRobot(my_id_).pose.orientation,q);
      resultVecTF = tf::quatRotate((const tf::Quaternion)q,(const tf::Vector3)tempPointTF);
      tf::vector3TFToMsg(resultVecTF,resultVec);
      temp.id="Waiting";
      temp.name="Waiting";
      temp.time=0.0;
      temp.pose.position.x=resultVec.x;
      temp.pose.position.y=resultVec.y;
      temp.pose.position.z=resultVec.z;
      return(temp);
    }
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

private:
  /****************************************************
   * @brief : Update the object list & the saliency map
   * @param : object list
   ****************************************************/
  void objectListCallback(const toaster_msgs::ObjectListStamped::ConstPtr& msg)
  {
    try
    {
      if(!msg->objectList.empty())
      {
        object_list_.clear();
        for (unsigned int i = 0; i < msg->objectList.size(); ++i)
        {
          if (msg->objectList[i].meEntity.id.find("CUBE")!=std::string::npos)
          {
            object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));

            if( saliency_map_.find(msg->objectList[i].meEntity.id) == saliency_map_.end() )
            {
              ROS_INFO("[stimulus_driven_attention] Add %s to saliency map.",msg->objectList[i].meEntity.id.c_str());
              saliency_map_.insert(SaliencyPair_t(msg->objectList[i].meEntity.id,0.0));
            }
          }
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[stimulus_driven_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update the robot list
   * @param : robot list
   ****************************************************/
  void robotListCallback(const toaster_msgs::RobotListStamped::ConstPtr& msg)
  {
    try
    {
      if(!msg->robotList.empty())
      {
        robot_list_.clear();
        for (unsigned int i = 0; i < msg->robotList.size(); ++i)
        {
          robot_list_.push_back(*(new toaster_msgs::Robot(msg->robotList[i])));
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[stimulus_driven_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update the human list & the saliency map
   * @param : human list
   ****************************************************/
  void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg)
  {
    std::string ownerId,jointId;
    try
    {
      if (!msg->humanList.empty())
      {
        human_list_.clear();

        for (unsigned int i = 0; i < msg->humanList.size(); ++i)
        {
          human_list_.push_back(*(new toaster_msgs::Human(msg->humanList[i])));
          ownerId = msg->humanList[i].meAgent.meEntity.id;
          if( saliency_map_.find(ownerId) == saliency_map_.end() )
          {
            if ( !msg->humanList[i].meAgent.skeletonJoint.empty() )
            {
              for( unsigned int j = 0 ; j < msg->humanList[i].meAgent.skeletonJoint.size() ; ++j )
              {
                  jointId = msg->humanList[i].meAgent.skeletonJoint[j].meEntity.id;
                  if (jointId != "base")
                    //ROS_INFO("[stimulus_driven_attention] Add %s::%s to saliency map.",ownerId.c_str(),jointId.c_str());
                    saliency_map_.insert(SaliencyPair_t((ownerId+"::"+jointId),0.0));
              }
            }
          }
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[stimulus_driven_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update the fact list provided by agent_monitor
   * @param : fact list
   ****************************************************/
  void factListCallback(const toaster_msgs::FactList::ConstPtr& msg)
  {
    try
    {
      if (!msg->factList.empty())
      {
        fact_list_.clear();
        for (unsigned int i = 0; i < msg->factList.size(); ++i)
        {
          if (msg->factList[i].targetId!="unknown object" && msg->factList[i].subjectId!="unknown object")
          {
            fact_list_.push_back(*(new toaster_msgs::Fact(msg->factList[i])));
          }
        }
      }
      updateSaliencyMap();
      sendSalientStimuli(salient_stimuli_);
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[stimulus_driven_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update the fact list provided by area_manager
   * @param : fact list
   ****************************************************/
  void factListAreaCallback(const toaster_msgs::FactList::ConstPtr& msg)
  {
    try
    {
      if (!msg->factList.empty())
      {
        fact_area_list_.clear();
        for (unsigned int i = 0; i < msg->factList.size(); ++i)
        {
          if (msg->factList[i].targetId!="unknown object" && msg->factList[i].subjectId!="unknown object")
          {
            fact_area_list_.push_back(*(new toaster_msgs::Fact(msg->factList[i])));
          }
        }
      }
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[stimulus_driven_attention] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update reactives parameters
   * @param : fact list
   ****************************************************/
  void dynParamCallback(head_manager::StimulusDrivenAttentionConfig &config, uint32_t level) 
  {
    stimuliDiscountFactor_ = config.stimuli_discount_factor;
    directionSalienceFactor_ = config.direction_salience_factor;
    movingSalienceFactor_ = config.moving_salience_factor;
    lookingSalienceFactor_ = config.looking_salience_factor;
    hysteresisMinThreshold_ = config.hysteresis_min_threshold;
    hysteresisThresholdFactor_ = config.hysteresis_threshold_factor;
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
  ros::init(argc, argv, "salient_stimuli_selection");
  ros::NodeHandle n;
  StimulusDrivenAttention * sss = new StimulusDrivenAttention(n);
  while(ros::ok())
  {
    ros::spinOnce();    
  }
}
