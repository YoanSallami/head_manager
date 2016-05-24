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

class SalientStimuliSelection
{
public:
  SaliencyMap_t saliency_map_; //!< saliency map for stimuli selection
  ObjectList_t object_list_; //!< object list from pdg
  HumanList_t human_list_; //!< human list from pdg
  RobotList_t robot_list_; //!< robot list from pdg
  FactList_t fact_list_; //!< fact list from agent_monitor
private:
  string my_id_; //!< robot's id
  ros::NodeHandle node_; //!< node handler
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Subscriber robot_list_sub_; //!< robot list subscriber
  ros::Publisher salient_stimuli_pub_; //!< sensitive goal publisher
  ros::Publisher saliency_map_pub_; //!< saliency map publisher
  ros::ServiceServer inhibition_of_return_srv_; //!< inhibition of return service
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  SalientStimuliSelection(ros::NodeHandle& node)
  {
    node_=node;
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
    inhibition_of_return_srv_ = node_.advertiseService("head_manager/inhibition_of_return", &SalientStimuliSelection::inhibitionOfReturn, this);
    // Add a waiting attention zone to saliency map
    saliency_map_.insert(SaliencyPair_t("Waiting",0.0));
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
    double stimuliDiscountFactor;
    double objectSalienceFactor;
    double headSalienceFactor;
    double jointSalienceFactor;
    double lookingSalienceFactor;
    SaliencyMap_t temp_map = saliency_map_;
    SaliencyMap_t objectSaliency_map;
    SaliencyMap_t headSaliency_map;
    SaliencyMap_t jointSaliency_map;
    SaliencyMap_t lookingSaliency_map;
    SaliencyMap_t::iterator subject;
    SaliencyMap_t::iterator subjectOwner;
    SaliencyMap_t::iterator target;
    SaliencyMap_t::iterator joint;
    // Getting ros parameters
    if(node_.hasParam("stimuli_discount_factor"))
    {
      node_.getParam("stimuli_discount_factor", stimuliDiscountFactor);
    } else {
      stimuliDiscountFactor = 0.8;
    }
    if(node_.hasParam("object_salience_factor"))
    {
      node_.getParam("object_salience_factor", objectSalienceFactor);
    } else {
      objectSalienceFactor = 1;
    }
    if(node_.hasParam("head_salience_factor"))
    {
      node_.getParam("head_salience_factor", headSalienceFactor);
    } else {
      headSalienceFactor = 1;
    }
    if(node_.hasParam("joint_salience_factor"))
    {
      node_.getParam("joint_salience_factor", jointSalienceFactor);
    } else {
      jointSalienceFactor = 1;
    }
    if(node_.hasParam("looking_salience_factor"))
    {
      node_.getParam("looking_salience_factor", lookingSalienceFactor);
    } else {
      lookingSalienceFactor= 1;
    }
    if (!temp_map.empty())
    {
      for(SaliencyMap_t::iterator it_tm = temp_map.begin() ; it_tm != temp_map.end() ; ++it_tm )
      {
        it_tm->second=0.0;
      }
    }
    objectSaliency_map = temp_map;
    headSaliency_map = temp_map;
    jointSaliency_map = temp_map;
    lookingSaliency_map = temp_map;
    // Saliency update according to motion facts provided by agent_monitor
    if(!fact_list_.empty())
    {
      for (FactList_t::iterator it_fl = fact_list_.begin() ; it_fl != fact_list_.end() ; ++it_fl )
      {
        if (it_fl->subjectId!=my_id_)
        {
          if ( it_fl->property == "IsMovingToward" && it_fl->subProperty=="angle")
          {
            if (it_fl->targetId!=my_id_)
            {
              target=objectSaliency_map.find(it_fl->targetId);
                if ( target != objectSaliency_map.end() )
              {
                target->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->targetId+" in object saliency map.");
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
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in looking saliency map.");
              }
            }
          }
          if ( it_fl->property == "IsMoving")
          {
            if ( it_fl->subProperty == "joint")
            { 
              subject=jointSaliency_map.find(it_fl->subjectOwnerId+"::"+it_fl->subjectId);
              if ( subject != jointSaliency_map.end() )
              {
                  subject->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in joint saliency map.");
              }
            }
            if ( it_fl->subProperty == "agent")
            { 
              subject=headSaliency_map.find(it_fl->subjectId+"::head");
              if ( subject != headSaliency_map.end() )
              {
                  subject->second+=it_fl->doubleValue;
              } else {
                throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in head saliency map.");
              }
            }
          }
        }
      }
      if(!normalizeMap(objectSaliency_map))
        throw HeadManagerException ( "Could not normalize object saliency map");
      if(!normalizeMap(headSaliency_map))
        throw HeadManagerException ( "Could not normalize head saliency map");
      if(!normalizeMap(jointSaliency_map))
        throw HeadManagerException ( "Could not normalize joint saliency map");
      if(!normalizeMap(lookingSaliency_map))
        throw HeadManagerException ( "Could not normalize looking saliency map");
      for (SaliencyMap_t::iterator it = temp_map.begin(); it != temp_map.end() ; ++it)
      {
        it->second=(objectSaliency_map[it->first].second*objectSalienceFactor)+
                    (headSaliency_map[it->first].second*headSalienceFactor)+
                    (jointSaliency_map[it->first].second*jointSalienceFactor)+
                    (lookingSaliency_map[it->first].second*lookingSalienceFactor);
      }
      if(!normalizeMap(temp_map))
        throw HeadManagerException ( "Could not normalize temp saliency map");
      
      if(!saliency_map_.empty())
      {
        for(SaliencyMap_t::iterator it_sm = saliency_map_.begin() ; it_sm != saliency_map_.end() ; ++it_sm )
        {
          it_sm->second*=stimuliDiscountFactor; // Temporal filtering to reduce salience over time
          it_sm->second+=temp_map[it_sm->first].second; // Add normalized feature saliency
        }
      } else {
        throw HeadManagerException ( "Could not update an empty saliency map." );
      }
    }
    sendSaliencyMap();
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
  void sendSalientStimuli()
  {
    geometry_msgs::PointStamped point;
    toaster_msgs::Entity entity;
    SaliencyPair_t salient;
    if (selectBestStimuli(salient))
    {
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
        tempPoint[0]= 1.0;//getRobot(my_id_).positionX;
        tempPoint[1]= 0.0;//getRobot(my_id_).positionY;
        tempPoint[2]= 1.2;//getRobot(my_id_).positionZ;
        rotZ = MathFunctions::matrixfromAngle(2,(const double)getRobot(my_id_).orientationYaw);
        resultPoint = MathFunctions::multiplyMatVec(rotZ,tempPoint);
        point.point.x = resultPoint[0]+getRobot(my_id_).positionX;
        point.point.y = resultPoint[1]+getRobot(my_id_).positionY;
        point.point.z = resultPoint[2]+getRobot(my_id_).positionZ;
      }
      point.header.frame_id="map";
      point.header.stamp=ros::Time::now();
      salient_stimuli_pub_.publish(point);
    } else {
      throw HeadManagerException ( "Could not select best stimuli in an empty saliency map." );
    }   
  }
private:
  bool normalizeMap(SaliencyMap_t& map)
  {
    double max=0;
    SaliencyMap_t::iterator it;
    if(!map.empty())
    {
      for( it = map.begin() ; it != map.end() ; ++it )
      {
        if( it->second > max )
        {
          max=it->second;
        }
      }
      for( it = map.begin() ; it != map.end() ; ++it )
      {
        it->second /= max;
      }
      return(true);
    } else return(false);
  }
  /****************************************************
   * @brief : Select the best stimuli from saliency map
   * @param : best stimuli
   * @return : true if succeed
   ****************************************************/
  bool selectBestStimuli(SaliencyPair_t& best)
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
      sendSalientStimuli();
    }
    catch (HeadManagerException& e )
    {
      ROS_ERROR("[salient_stimuli_selection] Exception was caught : %s",e.description().c_str());
    }
  }
public:
  /****************************************************
   * @brief : Inhibition of return service
   * @param : stamped point
   * @param : true if succeed
   ****************************************************/
  bool inhibitionOfReturn(head_manager::InhibitionOfReturn::Request& req,
                          head_manager::InhibitionOfReturn::Response& res)
  {
    SaliencyMap_t::iterator it;
    toaster_msgs::Entity entity;
    float xPosition=0;
    float yPosition=0;
    float zPosition=0;
    float xDistance=0;
    float yDistance=0;
    float zDistance=0;
    float radius=0.2;
    res.success=false;
    if (!saliency_map_.empty())
    {
      for( it = saliency_map_.begin() ; it != saliency_map_.end() ; ++it )
      {
        if(it->first!="Waiting")
        {
          if(!isObject(it->first))
          {
            if (isJoint(it->first))
            {
              size_t pos = it->first.find("::");
              if(isHuman(it->first.substr(0,pos)))
                entity = getHumanJoint(it->first.substr(0,pos),it->first.substr(pos+2,it->first.size()-1));
              else
                entity = getRobotJoint(it->first.substr(0,pos),it->first.substr(pos+2,it->first.size()-1));
            } else {
              if(isHuman(it->first))
                entity = getHuman(it->first);
              else
                entity = getRobot(it->first);
            }
          } else {
            entity = getObject(it->first);
          }
          xPosition = entity.positionX;
          yPosition = entity.positionY; 
          zPosition = entity.positionZ;
          xDistance = xPosition - req.point.point.x;
          yDistance = yPosition - req.point.point.y;
          zDistance = zPosition - req.point.point.z;
          if (sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance)) < radius )
          {
            it->second=0.0;
            res.success=true;
          }
        }
      }
    }
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
  ros::init(argc, argv, "salient_stimuli_selection");
  ros::NodeHandle n;
  SalientStimuliSelection * sss = new SalientStimuliSelection(n);
  while(ros::ok())
  {
    ros::spinOnce();    
  }
}
