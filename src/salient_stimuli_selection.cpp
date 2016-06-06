#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "geometry_msgs/PointStamped.h"

#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Robot.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/Entity.h"

#include <dynamic_reconfigure/server.h>
#include <head_manager/ReactiveHeadMotionConfig.h>

#include "../include/toaster-lib/MathFunctions.h"

#include "head_manager/StampedMap.h"
#include "head_manager/InhibitionOfReturn.h"

using namespace std;

//for convenience
typedef std::map < std::string, float > SaliencyMap_t;
typedef std::pair < std::string, float > SaliencyPair_t;
typedef std::vector < toaster_msgs::Object > ObjectList_t;
typedef std::vector < toaster_msgs::Robot > RobotList_t;
typedef std::vector < toaster_msgs::Human > HumanList_t;
typedef std::vector < toaster_msgs::Fact > FactList_t;
typedef dynamic_reconfigure::Server<head_manager::ReactiveHeadMotionConfig> ParamServer_t;

class SalientStimuliSelection
{
public:
  SaliencyMap_t saliency_map_; //!< saliency map for stimuli selection
  ObjectList_t object_list_; //!< object list from pdg
  HumanList_t human_list_; //!< human list from pdg
  RobotList_t robot_list_; //!< robot list from pdg
  FactList_t fact_list_; //!< fact list from agent_monitor
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
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber robot_list_sub_; //!< robot list subscriber
  ros::Publisher salient_stimuli_pub_; //!< sensitive goal publisher
  ros::Publisher saliency_map_pub_; //!< saliency map publisher
  ros::ServiceServer inhibition_of_return_srv_; //!< inhibition of return service
  ParamServer_t reactive_dyn_param_srv; //!<
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  SalientStimuliSelection(ros::NodeHandle& node)
  {
    node_=node;
    // Setting reactive parameters to default
    stimuliDiscountFactor_ = 0.8;
    directionSalienceFactor_ = 1;
    movingSalienceFactor_ = 1;
    lookingSalienceFactor_ = 1;
    inhibitionRadius_ = 0.2;
    hysteresisMinThreshold_=1.0;
    hysteresisThresholdFactor_ = 0.2;
    // Getting robot's id from ros param
    if(node_.hasParam("my_robot_id"))
    {
      node_.getParam("my_robot_id",my_id_);
    } else {
      my_id_="pr2";
    }
    // Advertise subscribers
    object_list_sub_ = node_.subscribe("/pdg/objectList", 1, &SalientStimuliSelection::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 1, &SalientStimuliSelection::humanListCallback, this);
    robot_list_sub_ = node_.subscribe("/pdg/robotList", 1, &SalientStimuliSelection::robotListCallback, this);
    fact_list_sub_ = node_.subscribe("/agent_monitor/factList", 1, &SalientStimuliSelection::factListCallback, this);
    // Advertise publishers
    salient_stimuli_pub_ = node_.advertise<geometry_msgs::PointStamped>("head_manager/salient_stimuli", 1);
    saliency_map_pub_ = node_.advertise<head_manager::StampedMap>("head_manager/saliency_map",1);
    // Advertise services
    // Dyn param server
    reactive_dyn_param_srv.setCallback(boost::bind(&SalientStimuliSelection::dynParamCallback, this, _1, _2));
    // Add a waiting attention zone to saliency map
    saliency_map_.insert(SaliencyPair_t("Waiting",0.0));
    salient_stimuli_=SaliencyPair_t("Waiting",0.0);
  }
  /****************************************************
   * @brief : Default destructor
   ****************************************************/
  ~SalientStimuliSelection(){}
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
        if (it_fl->subjectId!=my_id_)
        {
          if ( it_fl->property == "IsMovingToward" && it_fl->subProperty=="direction")
          {
            
            if (it_fl->targetId!=my_id_)
            {
              if(it_fl->subjectId=="rightHand")
              {
                target=directionSaliency_map.find(it_fl->targetId);
                if ( target != directionSaliency_map.end() )
                {
                  target->second+=it_fl->doubleValue;
                } else {
                  throw HeadManagerException ("Could not find "+it_fl->targetId+" in object saliency map.");
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
                throw HeadManagerException ("Could not find "+it_fl->targetId+" in looking saliency map.");
              }
            }else{
              subject=lookingSaliency_map.find(it_fl->subjectId+"::head");
              if ( subject != lookingSaliency_map.end() )
              {
                subject->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+"::"+it_fl->subjectId+" in looking saliency map.");
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
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in moving saliency map.");
              }
            }
            if ( it_fl->subProperty == "agent")
            { 
              subject=movingSaliency_map.find(it_fl->subjectId+"::head");
              if ( subject != movingSaliency_map.end() )
              {
                subject->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in movingsaliency map.");
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
                throw HeadManagerException ("Could not find "+it_fl->targetId+"::head in inhibition map.");
              }
            } else {
              // ojects
              target=inhibition_map.find(it_fl->targetId);
              if ( target != inhibition_map.end() )
              {
                target->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->targetId+" in inhibition map.");
              }
            }//TODO add case for others robots
            
          }
        }
      }

      if(!normalizeMap(inhibition_map))
        throw HeadManagerException ( "Could not normalize object saliency map");
      if(!normalizeMap(directionSaliency_map))
        throw HeadManagerException ( "Could not normalize object saliency map");
      if(!normalizeMap(movingSaliency_map))
        throw HeadManagerException ( "Could not normalize joint saliency map");
      if(!normalizeMap(lookingSaliency_map))
        throw HeadManagerException ( "Could not normalize looking saliency map");
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
    head_manager::StampedMap map;
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
    geometry_msgs::PointStamped point;
    toaster_msgs::Entity entity;
    if(salient.first!="Waiting")
    {
      point.header.frame_id = "map";
      if(!isObject(salient.first))
      {
        if (isJoint(salient.first))
        {
          size_t pos = salient.first.find("::");
          if(isHuman(salient.first.substr(0,pos)))
            entity = getHumanJoint(salient.first.substr(0,pos),salient.first.substr(pos+2,salient.first.size()-1));
          else
            entity = getRobotJoint(salient.first.substr(0,pos),salient.first.substr(pos+2,salient.first.size()-1));
        } else {
          if(isHuman(salient.first))
            entity = getHuman(salient.first);
          else
            entity = getRobot(salient.first);
        }
      } else {
        entity = getObject(salient.first);
      }
      point.point.x = entity.positionX;
      point.point.y = entity.positionY; 
      point.point.z = entity.positionZ;
    } else {
      Vec_t tempPoint(3);
      Vec_t resultPoint(3);
      Mat_t rotZ(3);
      tempPoint[0]= 1.0;
      tempPoint[1]= 0.0;
      tempPoint[2]= 1.2;
      rotZ = MathFunctions::matrixfromAngle(2,(const double)getRobot(my_id_).orientationYaw);
      resultPoint = MathFunctions::multiplyMatVec(rotZ,tempPoint);
      point.point.x = resultPoint[0]+getRobot(my_id_).positionX;
      point.point.y = resultPoint[1]+getRobot(my_id_).positionY;
      point.point.z = resultPoint[2]+getRobot(my_id_).positionZ;
    }
    point.header.frame_id="map";
    point.header.stamp=ros::Time::now();
    salient_stimuli_pub_.publish(point);  
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
  void objectListCallback(const toaster_msgs::ObjectList::ConstPtr& msg)
  {
    if(!msg->objectList.empty())
    {
      object_list_.clear();
      for (unsigned int i = 0; i < msg->objectList.size(); ++i)
      {
        object_list_.push_back(*(new toaster_msgs::Object(msg->objectList[i])));

        if( saliency_map_.find(msg->objectList[i].meEntity.id) == saliency_map_.end() )
        {
          saliency_map_.insert(SaliencyPair_t(msg->objectList[i].meEntity.id,0.0));
        }
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
      }
    }
  }
  /****************************************************
   * @brief : Update the human list & the saliency map
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
        ownerId = msg->humanList[i].meAgent.meEntity.id;
        if( saliency_map_.find(ownerId) == saliency_map_.end() )
        {
          if ( !msg->humanList[i].meAgent.skeletonJoint.empty() )
          {
            for( unsigned int j = 0 ; j < msg->humanList[i].meAgent.skeletonJoint.size() ; ++j )
            {
                jointId = msg->humanList[i].meAgent.skeletonJoint[j].meEntity.id;
                if (jointId != "base")
                  saliency_map_.insert(SaliencyPair_t((ownerId+"::"+jointId),0.0));
            }
          }
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
      updateSaliencyMap();
      sendSalientStimuli(salient_stimuli_);
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[salient_stimuli_selection] Exception was caught : %s",e.description().c_str());
    }
  }
  /****************************************************
   * @brief : Update reactives parameters
   * @param : fact list
   ****************************************************/
  void dynParamCallback(head_manager::ReactiveHeadMotionConfig &config, uint32_t level) 
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
  SalientStimuliSelection * sss = new SalientStimuliSelection(n);
  while(ros::ok())
  {
    ros::spinOnce();    
  }
}
