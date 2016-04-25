#include <ros/ros.h>
#include <string>

using namespace std;

class CognitiveOrientation
{
private:
 
public:
  /**
   * Default constructor
   */
  CognitiveOrientation(ros::NodeHandle& node)
  {
  
  }
  /** 
   * Default destructor
   */
  ~CognitiveOrientation()
  {
    
  }
private:

};

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