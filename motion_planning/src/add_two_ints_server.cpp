#include "ros/ros.h"
#include "motion_planning/AddTwoInts.h"

int fib(int a);

int fib(int a)
{
    if (a <= 1)
    {
      return 1;
    }
    else 
    {
	return fib(a-1)+fib(a-2);
    }
}


bool add(motion_planning::AddTwoInts::Request  &req,
         motion_planning::AddTwoInts::Response &res)
{
  res.sum = fib(req.a);
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

