#ifndef NAO_MOCAP_NODE_GUARD_H
#define NAO_MOCAP_NODE_GUARD_H 1

#include <mutex>
#include <thread>

#include <ros/ros.h>

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <naoqi_msgs/Bumper.h>
#include <std_srvs/Empty.h>

// Aldebaran
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alvalue/alvalue.h>
#include <alcommon/altoolsmain.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/alproxy.h>
#include <alproxies/alproxies.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

class NaoMocapNode
{
public:
  NaoMocapNode();
  ~NaoMocapNode();

  void run();

private:
  ros::NodeHandle m_nh, m_privateNh;

  std::string m_naoIP;
  int m_naoPort;

  double m_goalUpdatingFrequency;

  int m_taskId;

  double m_xTolerance;
  double m_yTolerance;
  double m_yawTolerance; 

  ros::Timer m_updateGoalTimer;

  ros::Subscriber m_moveBaseSimpleGoalSub;
  ros::Subscriber m_trackedPoseSub;
  ros::Subscriber m_bumperSub;

  ros::ServiceServer m_moveBaseGoalPreemptServer;

  boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
  boost::shared_ptr<AL::ALRobotPostureProxy> m_postureProxy;

  tf::Transform m_currentRobotPositon;
  tf::Transform m_currentGoalPosition;

  bool m_isGoalRunning;
  std::mutex goal_mtx;

  void updateGoalTimerCallback(const ros::TimerEvent&);
  void startGoalExecution(geometry_msgs::Pose2D goal);
  bool abortCurrentGoalSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void abortCurrentGoal();

  geometry_msgs::Pose2D TFPoseToPose2D(tf::Transform tf_pose);

  bool isGoalReached(const geometry_msgs::Pose2D& goal);

  void moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void bumperCallback(const naoqi_msgs::Bumper::ConstPtr& msg);
};

#endif