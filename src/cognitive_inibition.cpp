#include <ros/ros.h>
#include <string>

using namespace std;

class CognitiveInibition
{
private:
  
public:
  /** 
   * Default constructor
   */
  CognitiveInibition(ros::NodeHandle& node)
  {
    
  }
  /** 
   * Default destructor
   */
  ~CognitiveInibition()
  {
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cognitive_inibition");
  ros::NodeHandle n;
  CognitiveInibition * ci = new CognitiveInibition(n);
  while(ros::ok())
  {
    ros::spinOnce();
  }
}