#ifndef QTNODE_HPP
#define QTNODE_HPP

#include <QLabel>
#include <QImage>
#include <QDebug>
#include <QThread>
#include <QStringListModel>

#include <map>
#include <string>
#include <sstream>
#include <QSettings>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace qt_map_rviz_ros
{
  class QtNode:public QThread
  {
    Q_OBJECT
  public:
    QtNode(int argc, char ** argv);
    virtual ~QtNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void disinit();
    void moveBase(char k, float speedLinear, float speedTurn);
    void setGoal(std::string frame, double x, double y, double z, double w);
    void run();


  Q_SIGNALS:
    void rosShutdown();
    void masterShutdown();
  private:
    int init_argc;
    char ** init_argv;
    ros::Publisher goalPub;
    ros::Publisher cmdPub;

  };

}

#endif // QTNODE_HPP
