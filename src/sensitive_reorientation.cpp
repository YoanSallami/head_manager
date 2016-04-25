#include <ros/ros.h>
#include <string>

using namespace std;

class SensitiveReorientation
{
private:
 
public:
  /**
   * Default constructor
   */
  SensitiveReorientation(ros::NodeHandle& node)
  {
  
  }
  /** 
   * Default destructor
   */
  ~SensitiveReorientation()
  {
    
  }
private:

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cognitive_orientation");
  ros::NodeHandle n;
  SensitiveReorientation * sr = new SensitiveReorientation(n);
  while(ros::ok())
  {
    ros::spinOnce();

  }
}