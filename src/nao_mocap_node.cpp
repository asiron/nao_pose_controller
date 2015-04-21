#include "nao_mocap/nao_mocap_node.h"

NaoMocapNode::NaoMocapNode() :
  m_nh(),
  m_privateNh("~"),
  m_naoIP(""),
  m_naoPort(0),
  m_goalUpdatingFrequency(100.0),
  m_currentRobotPositon(),
  m_currentGoalPosition(),
  m_isGoalRunning(false),
  goal_mtx(),
  m_xTolerance(),
  m_yTolerance(),
  m_yawTolerance(),
  m_taskId(0)

{
  m_privateNh.param("updating_frequency", m_goalUpdatingFrequency, m_goalUpdatingFrequency);
  m_privateNh.param("nao_ip", m_naoIP, m_naoIP);
  m_privateNh.param("nao_port", m_naoPort, m_naoPort);

  m_privateNh.param("x_tolerance", m_xTolerance, m_xTolerance);
  m_privateNh.param("y_tolerance", m_yTolerance, m_yTolerance);
  m_privateNh.param("yaw_tolerance", m_yawTolerance, m_yawTolerance);

  ROS_INFO("Nao Motion Capture Control Node initialized...");
  ROS_INFO("\t Tolerance values: X: %lf Y: %lf Theta: %lf", m_xTolerance, m_yTolerance, m_yawTolerance);
}

NaoMocapNode::~NaoMocapNode()
{

}

void NaoMocapNode::moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{

  ROS_DEBUG("Received Move Base goal:");
  ROS_DEBUG("\t Position: %.2f %.2f %.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ROS_DEBUG("\t Quaternion: %.2f %.2f %.2f %.2f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  tf::Stamped<tf::Pose> moveBaseGoalTFPose;
  tf::poseStampedMsgToTF(*msg,moveBaseGoalTFPose);
  m_currentGoalPosition = moveBaseGoalTFPose;

  tf::Transform robotToGoalTransform = m_currentRobotPositon.inverse() * m_currentGoalPosition ;
 
  geometry_msgs::Pose2D newGoal = TFPoseToPose2D(robotToGoalTransform);

  ROS_INFO("\t New goal with respect to Robot's frame:");
  ROS_INFO("\t X: %lf \t Y: %lf \t Theta: %lf", newGoal.x, newGoal.y, newGoal.theta);

  if ( isGoalReached(newGoal) )
  {
    ROS_INFO("Starting position is within the tolerance of the goal, aborting...");
    return;
  }

  if (m_isGoalRunning == true)
  {
    ROS_INFO("Goal is already running, updating ONLY the goal position");
    return;
  }

  ROS_INFO("New goal accepted, starting execution");
  startGoalExecution(newGoal);
}

void NaoMocapNode::trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Stamped<tf::Pose> trackedPose;
  tf::poseStampedMsgToTF(*msg,trackedPose);

  tfScalar roll, pitch, yaw;
  tf::Matrix3x3(trackedPose.getRotation()).getEulerYPR(yaw, pitch, roll);

  tf::Quaternion trackedPose2DQuaternion = tf::createQuaternionFromYaw(yaw);

  m_currentRobotPositon = tf::Transform(trackedPose2DQuaternion, 
    tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0L));

  tfScalar r, p, y;
  tf::Matrix3x3(m_currentRobotPositon.getRotation()).getEulerYPR(y, p, r);

  ROS_DEBUG("%.2f %.2f %.2f \t %.2f %.2f %.2f ", 
    m_currentRobotPositon.getOrigin().x(),
    m_currentRobotPositon.getOrigin().y(),
    m_currentRobotPositon.getOrigin().z(),
    y,p,r);
}

geometry_msgs::Pose2D NaoMocapNode::TFPoseToPose2D(tf::Transform tf_pose)
{
  geometry_msgs::Pose2D pose2D;

  tfScalar r, p, y;
  tf::Matrix3x3(tf_pose.getRotation()).getEulerYPR(y, p, r);

  pose2D.x = tf_pose.getOrigin().x();
  pose2D.y  = tf_pose.getOrigin().y();
  pose2D.theta = y;

  return pose2D;
}

void NaoMocapNode::bumperCallback(const naoqi_msgs::Bumper::ConstPtr& msg)
{
  if (msg->state == 1) {
    ROS_WARN("Robot hit surface with bumpers, aborting goal.");
    abortCurrentGoal();
  }
}

void NaoMocapNode::updateGoalTimerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("Timer callback!");

  /* if(m_taskId != 0) {
    m_motionProxy->stop(m_taskId);
  }
  */

  if (m_isGoalRunning == false) {
    ROS_WARN("Goal was aborted, should stop any second now");
    return;
  }

  tf::Transform robotToGoalTransform = m_currentRobotPositon.inverse() * m_currentGoalPosition;
  geometry_msgs::Pose2D newGoal = TFPoseToPose2D(robotToGoalTransform);

  if ( isGoalReached(newGoal) )
  {
    ROS_INFO("Goal reached!!");
    abortCurrentGoal();
    return;
  }

  m_taskId =  m_motionProxy->post.moveTo(newGoal.x, newGoal.y, newGoal.theta);
}


bool NaoMocapNode::isGoalReached(const geometry_msgs::Pose2D& goal)
{
  if ( (fabs(goal.x) < m_xTolerance) && 
       (fabs(goal.y) < m_yTolerance) && 
       (fabs(goal.theta) < m_yawTolerance) )
  {
    return true;
  }
  return false;
}

void NaoMocapNode::startGoalExecution(geometry_msgs::Pose2D goal)
{
  m_isGoalRunning = true;
  m_postureProxy->goToPosture("StandInit", 0.5);
  m_updateGoalTimer = m_nh.createTimer(ros::Duration( 1 / m_goalUpdatingFrequency), &NaoMocapNode::updateGoalTimerCallback, this);
}

void NaoMocapNode::abortCurrentGoal()
{
  if(m_isGoalRunning == false)
    return;

  m_isGoalRunning = false;
  ROS_INFO("Aborting goal");
  //m_motionProxy->stop(m_taskId);
  m_motionProxy->stopMove();
  m_updateGoalTimer.stop();
}

bool NaoMocapNode::abortCurrentGoalSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  abortCurrentGoal();
}

void NaoMocapNode::run()
{
  m_moveBaseSimpleGoalSub = m_nh.subscribe("/move_base_simple/goal", 1000 , &NaoMocapNode::moveBaseSimpleGoalCallback, this);
  m_trackedPoseSub = m_nh.subscribe("/mocap/nao_tracker/pose", 1000, &NaoMocapNode::trackedPoseStampedCallback, this);
  m_bumperSub = m_nh.subscribe("nao/bumper", 1000, &NaoMocapNode::bumperCallback, this);
  m_moveBaseGoalPreemptServer = m_nh.advertiseService("abort_goal", &NaoMocapNode::abortCurrentGoalSrvCallback, this);

  try {
    m_motionProxy  = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(m_naoIP, m_naoPort));
  } catch (const AL::ALError& e) {
    ROS_ERROR("Couldn't connect to ALMotionProxy : %s", e.what());
    exit(1);
  }

  try {
    m_postureProxy = boost::shared_ptr<AL::ALRobotPostureProxy>(new AL::ALRobotPostureProxy(m_naoIP, m_naoPort));
  } catch (const AL::ALError& e) {
    ROS_ERROR("Couldn't connect to ALRobotPostureProxy : %s", e.what());
    exit(1);
  }

  ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_mocap");
    NaoMocapNode nm;
    nm.run();
    return 0;
}