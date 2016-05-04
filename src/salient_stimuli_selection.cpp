#include <ros/ros.h>
#include <string>
#include <cstdlib>
//#include <math>
#include <map>

#include "../include/head_manager/HeadManagerException.h"
#include "geometry_msgs/PointStamped.h"

#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/Entity.h"

#include "head_manager/StampedMap.h"
#include "head_manager/InhibitionOfReturn.h"

using namespace std;

typedef std::map < std::string, float > SaliencyMap_t;
typedef std::pair < std::string, float > SaliencyPair_t;
typedef std::vector < toaster_msgs::Object > ObjectList_t;
typedef std::vector < toaster_msgs::Human > HumanList_t;
typedef std::vector < toaster_msgs::Fact > FactList_t;

class SalientStimuliSelection
{
public:
  SaliencyMap_t saliency_map_; //!< saliency map for stimuli selection
  ObjectList_t object_list_;
  HumanList_t human_list_;
  FactList_t fact_list_;
private:
  ros::NodeHandle node_;
  ros::Subscriber fact_list_sub_; //!< fact list subscriber
  ros::Subscriber object_list_sub_; //!< object list subscriber
  ros::Subscriber human_list_sub_; //!< human list subscriber
  ros::Publisher salient_stimuli_pub_; //!< sensitive goal publisher
  ros::Publisher saliency_map_pub_;
  ros::ServiceServer inhibition_of_return_srv_;
public:
  /****************************************************
   * @brief : Default constructor
   * @param : ros node handler
   ****************************************************/
  SalientStimuliSelection(ros::NodeHandle& node)
  {
    /**
     * Node setup
     **/
    node_=node;
    // Advertise subscribers
    object_list_sub_ = node_.subscribe("/pdg/objectList", 5, &SalientStimuliSelection::objectListCallback, this);
    human_list_sub_ = node_.subscribe("/pdg/humanList", 5, &SalientStimuliSelection::humanListCallback, this);
    fact_list_sub_ = node_.subscribe("/agent_monitor/factList", 5, &SalientStimuliSelection::factListCallback, this);
    // Advertise publishers
    salient_stimuli_pub_ = node_.advertise <geometry_msgs::PointStamped>("head_manager/salient_stimuli", 5);
    saliency_map_pub_ = node_.advertise <head_manager::StampedMap>("head_manager/saliency_map",5);
    // Advertise services
    inhibition_of_return_srv_ = node_.advertiseService("head_manager/inhibition_of_return", &SalientStimuliSelection::inhibitionOfReturn, this);
  }
  /**
   * Default destructor
   */
  ~SalientStimuliSelection()
  {
    delete(&object_list_);
    delete(&human_list_);
  }

  void updateSaliencyMap()
  {
    toaster_msgs::Human human;
    toaster_msgs::Object object;
    float xPosition=0;
    float yPosition=0;
    float zPosition=0;
    float stimuliDiscountFactor; // stimuli discount factor
    float objectSalienceFactor;
    float headSalienceFactor;
    float jointSalienceFactor;
    float lookingSalienceFactor;
    SaliencyMap_t temp = saliency_map_;
    SaliencyMap_t::iterator subject;
    SaliencyMap_t::iterator subjectOwner;
    SaliencyMap_t::iterator target;
    SaliencyMap_t::iterator joint;

    /**
     * Getting ros parameters
     **/
    if(node_.hasParam("stimuli_discount_factor"))
    {
      node_.getParam("stimuli_discount_factor", stimuliDiscountFactor);
    } else {
      stimuliDiscountFactor = 0.99;
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
      headSalienceFactor = 10;
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
    /**
    * Temporal filtering to reduce salience over time
    **/
    if(!saliency_map_.empty())
    {
      for(SaliencyMap_t::iterator it_sm = saliency_map_.begin() ; it_sm != saliency_map_.end() ; ++it_sm )
      {
        it_sm->second*=stimuliDiscountFactor;
        if (it_sm->second < 0.0001)//to avoid very low float
        {
          it_sm->second=0.0;
        }
      }
    } else {
      throw HeadManagerException ( "Could not update an empty saliency map." );
    }
    /**
    * Saliency update according to motion facts provided by toaster
    **/
    if(!fact_list_.empty())
    {
      for (FactList_t::iterator it_fl = fact_list_.begin() ; it_fl != fact_list_.end() ; ++it_fl )
      {
        if ( it_fl->property == "IsMovingToward" )
        {
          if (it_fl->targetId!="pr2")
          {
            target=saliency_map_.find(it_fl->targetId);
              if ( target != saliency_map_.end() )
            {
              target->second+=it_fl->doubleValue*objectSalienceFactor;
            } else {
              throw HeadManagerException ("Could not find "+it_fl->targetId+" in saliency map.");
            }
          }
        }
        if ( it_fl->property == "IsLookingToward" )
        {
          if (it_fl->targetId!="pr2")
          {
            target=saliency_map_.find(it_fl->targetId);
              if ( target != saliency_map_.end() )
            {
              target->second+=it_fl->doubleValue*lookingSalienceFactor;
            } else {
              throw HeadManagerException ("Could not find "+it_fl->targetId+" in saliency map.");
            }
          }else{
            subject=saliency_map_.find(it_fl->subjectId+"::head");
            if ( subject != saliency_map_.end() )
            {
                subject->second+=it_fl->doubleValue*lookingSalienceFactor;
            } else {
              throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in saliency map.");
            }
          }
        }
        if ( it_fl->property == "IsMoving")
        {
          if ( it_fl->subProperty == "joint")
          { 
            subject=saliency_map_.find(it_fl->subjectOwnerId+"::"+it_fl->subjectId);
            if ( subject != saliency_map_.end() )
            {
                subject->second+=it_fl->doubleValue*jointSalienceFactor;
            } else {
              throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in saliency map.");
            }
          }
          if ( it_fl->subProperty == "agent")
          { 
            subject=saliency_map_.find(it_fl->subjectId+"::head");
            if ( subject != saliency_map_.end() )
            {
                subject->second+=it_fl->doubleValue*headSalienceFactor;
            } else {
              throw HeadManagerException ("Could not find "+it_fl->subjectOwnerId+" "+it_fl->subjectId+" in saliency map.");
            }
          }
        }
      }
    } else {
      throw HeadManagerException ( "Could not read an empty fact list." );
    }

    sendSaliencyMap();

  }

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

  void sendSalientStimuli()
  {
    geometry_msgs::PointStamped point;
    toaster_msgs::Entity entity;
    SaliencyPair_t salient;
    if (selectBestStimuli(salient))
    {
      point.header.frame_id = "map";
      if (!isObject(salient.first))
      {
        if (isJoint(salient.first))
        {
          size_t pos = salient.first.find("::");
          entity = getJoint(salient.first.substr(0,pos),salient.first.substr(pos+2,salient.first.size()-1));
        } else {
          entity = getHuman(salient.first);
        }
      } else {
        entity = getObject(salient.first);
      }
        ROS_INFO("Salient stimuli send :");
        ROS_INFO("positionX = %f",entity.positionX);
        ROS_INFO("positionY = %f",entity.positionY);
        ROS_INFO("positionZ = %f",entity.positionZ);
        point.header.stamp=ros::Time::now();
        point.point.x = entity.positionX;
        point.point.y = entity.positionY; 
        point.point.z = entity.positionZ;
        ROS_INFO("PUBLICATION");
        salient_stimuli_pub_.publish(point);
    } else {
      throw HeadManagerException ( "Could not select best stimuli in an empty saliency map." );
    }   
  }

private:
  /*************************************
   * Node private methods
   *************************************/
  bool selectBestStimuli(SaliencyPair_t& best)
  {
    SaliencyMap_t::iterator it;
    SaliencyPair_t bestTemp = * saliency_map_.begin();

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

  toaster_msgs::Entity getObject(std::string id)
  {
    float x,y,z;
    if(!object_list_.empty())
    {
      for (unsigned int i = 0; i < object_list_.size(); ++i)
      {
        if (object_list_[i].meEntity.id == id)
        {
          std::string offset = "offset_"+object_list_[i].meEntity.id;
          if (node_.hasParam(offset))
          {
            if (node_.getParam(offset+"/x",x))
              object_list_[i].meEntity.positionX+=x;
            if (node_.getParam(offset+"/y",y))
              object_list_[i].meEntity.positionY+=y;
            if (node_.getParam(offset+"/z",z))
              object_list_[i].meEntity.positionZ+=z;
          }
          return (object_list_[i].meEntity);
        }
      }
    } else {
      throw HeadManagerException ( "Could not get entity object in an empty object list." );
    }
  }

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
      throw HeadManagerException ( "Could not get entity human in an empty human list." );
    }
  }

  toaster_msgs::Entity getJoint(std::string ownerId, std::string jointId)
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
            throw HeadManagerException ( "Could not get entity "+jointId+"joint in an empty skeletonJoint list." );
          }
        }
      }
    } else {
      throw HeadManagerException ( "Could not get entity "+ownerId+"human in an empty human list." );
    }
  }

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

  bool isJoint(std::string id)
  {
    std::size_t found=id.find("::");
    if(id.find("::") != std::string::npos)
      return(true);
    return(false);
  }

private:
  /*************************************
   * ROS Topic Callbacks
   *************************************/
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
          saliency_map_.emplace(msg->objectList[i].meEntity.id,0.0);
        }
      }
    }
  }

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
          //saliency_map_.emplace(msg->humanList[i].meAgent.meEntity.id,0.0);
          if ( !msg->humanList[i].meAgent.skeletonJoint.empty() )
          {
            for( unsigned int j = 0 ; j < msg->humanList[i].meAgent.skeletonJoint.size() ; ++j )
            {
                jointId = msg->humanList[i].meAgent.skeletonJoint[j].meEntity.id;
                if (jointId != "base")
                  saliency_map_.emplace((ownerId+"::"+jointId),0.0);
            }
          }
        }
      }
    }
  }

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
      ROS_ERROR("[head_manager] Exception was caught : %s",e.description().c_str());
    }   
    
  }

public:
  /******************************************************************
   * ROS Services 
   ******************************************************************/
  bool inhibitionOfReturn(head_manager::InhibitionOfReturn::Request &req,
                          head_manager::InhibitionOfReturn::Response &res)
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

    for( it = saliency_map_.begin() ; it != saliency_map_.end() ; ++it )
    {
      if(!isObject(it->first))
      {
        if (isJoint(it->first))
        {
          size_t pos = it->first.find("::");
          entity = getJoint(it->first.substr(0,pos),it->first.substr(pos+2,it->first.size()-1));
        } else {
          entity = getHuman(it->first);
        }
      } else {
        entity = getObject(it->first);
      }
      xPosition = entity.positionX;
      yPosition = entity.positionY; 
      zPosition = entity.positionZ;
      xDistance=xPosition-req.point.point.x;
      yDistance=yPosition-req.point.point.y;
      zDistance=zPosition-req.point.point.z;

      if (sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance)) < radius )
      {
        ROS_INFO("Inhibition of return success");
        it->second=0.0;
        res.success=true;
      }
    }
    return(true);
  }

};

  /******************************************************************
   * Main process
   ******************************************************************/
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