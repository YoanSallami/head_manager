// HeadManagerException class


#ifndef HeadManagerException_class
#define HeadManagerException_class

#include <string>

class HeadManagerException
{
 public:
  HeadManagerException ( std::string str ) : msg ( str ) {};
  ~HeadManagerException (){};

  std::string description() { return msg; }

 private:

  std::string msg;

};

#endif