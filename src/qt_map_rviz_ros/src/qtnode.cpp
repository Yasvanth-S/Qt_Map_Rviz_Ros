#include "../include/qt_map_rviz_ros/qtnode.hpp"

namespace qt_map_rviz_ros
{
  QtNode::QtNode(int argc, char ** argv):init_argc(argc), init_argv(argv)
  {}

  QtNode::~QtNode()
  {
    if(ros::isStarted())
    {
      ros::shutdown();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QtNode::init()
  {
    ros::init(init_argc, init_argv, "qt_map_rviz_ros");
    if(! ros::master::check())
    {
      return false;
    }
    ros::start();
    ros::NodeHandle n;
    goalPub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
    cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    start();
    return true;
  }

  bool QtNode::init(const std::string &master_url, const std::string &host_url)
  {
      std::map<std::string,std::string> remappings;
      remappings["__master"] = master_url;
      remappings["__hostname"] = host_url;
      ros::init(remappings,"qt_map_rviz_ros");
      if ( ! ros::master::check() )
      {
          return false;
      }
      ros::start();
      ros::NodeHandle n;
      goalPub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
      cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
      start();
      return true;
}


  void QtNode::disinit()
  {
    if(ros::isStarted())
    {
      ROS_INFO("ROS will ShutDown");
      ros::shutdown();
      ros::waitForShutdown();
    }
    ROS_INFO("ROS ShutDown");
    this->exit();
  }

  void QtNode::setGoal(std::string frame, double x, double y, double z, double w)
  {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = frame;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.w = w;
    goalPub.publish(goal);
    ros::spinOnce();
  }

  void QtNode::run()
  {
    ros::Rate loopRate(1);
    while(ros::ok())
    {
      ros::spinOnce();
      loopRate.sleep();
    }
  }

  void QtNode::moveBase(char k, float speedLinear, float speedTurn)
  {
    std::map<char, std::vector<float>> moveBindings
    {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},
      {'K', {0, 0, 0, 0}}
    };
    char key = k;
    float x = moveBindings[key][0];
    float y = moveBindings[key][1];
    float z = moveBindings[key][2];
    float yaw = moveBindings[key][3];
    float speed = speedLinear;
    float turn = speedTurn;
    geometry_msgs::Twist twist;
    twist.linear.x = static_cast<double>(x * speed);
    twist.linear.y = static_cast<double>(y * speed);
    twist.linear.z = static_cast<double>(z * speed);

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = static_cast<double>(yaw * turn);

    cmdPub.publish(twist);
    ros::spinOnce();
  }

}
