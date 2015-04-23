#include "nao_pose_controller/nao_pose_controller.h"

namespace SNT {
  namespace ARG {
    namespace NaoControllers {
   
      PathFollower::PathFollower() :
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
        m_nextGoalXTolerance(),
        m_nextGoalYTolerance(),
        m_nextGoalYawTolerance(), 
        m_taskId(0),
        m_moveBaseActionServer(m_nh, "move_base", boost::bind(&PathFollower::moveBaseGoalCallback, this, _1), false),
        m_followPathActionServer(m_nh, "follow_path", boost::bind(&PathFollower::followPathGoalCallback, this, _1), false)

      {
        m_privateNh.param("updating_frequency", m_goalUpdatingFrequency, m_goalUpdatingFrequency);
        m_privateNh.param("nao_ip", m_naoIP, m_naoIP);
        m_privateNh.param("nao_port", m_naoPort, m_naoPort);

        m_privateNh.param("x_tolerance", m_xTolerance, m_xTolerance);
        m_privateNh.param("y_tolerance", m_yTolerance, m_yTolerance);
        m_privateNh.param("yaw_tolerance", m_yawTolerance, m_yawTolerance);

        m_privateNh.param("next_goal_x_accept_treshold", m_nextGoalXTolerance, m_nextGoalXTolerance);
        m_privateNh.param("next_goal_y_accept_treshold", m_nextGoalYTolerance, m_nextGoalYTolerance);
        m_privateNh.param("next_goal_yaw_accept_treshold", m_nextGoalYawTolerance, m_nextGoalYawTolerance);

        ROS_INFO("Nao Motion Capture Control Node initialized...");
        ROS_INFO("\t Tolerance values: X: %lf Y: %lf Theta: %lf", m_xTolerance, m_yTolerance, m_yawTolerance);
      }

      PathFollower::~PathFollower()
      {

      }

      void PathFollower::moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
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

        if ( isGoalWithinTresholdOf(Treshold::LastPose, newGoal) )
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

      ros::Duration PathFollower::runPoseController(const std::vector<geometry_msgs::PoseStamped>& path)
      {
        ros::Time begin = ros::Time::now();

        if (path.size() == 0)
        {
          ROS_WARN("Received goal's path is empty");
          
          return ros::Time::now() - begin;
        }

        bool last_pose = false;

        for (auto it = path.begin(); it != path.end(); ++it)
        {

          m_currentGoalPosition = *it;

          if (it + 1 == path.end()) {
            last_pose = true;
            ROS_INFO("Processing the last pose in path");
          }

          ros::Rate r(m_goalUpdatingFrequency);

          while(ros::ok())
          {

            if (m_isGoalRunning == false) {
              ROS_WARN("Goal was aborted, should stop any second now");
              return ros::Time::now() - begin;
            }

            tf::Transform robotToGoalTransform = m_currentRobotPositon.inverse() * m_currentGoalPosition;
            geometry_msgs::Pose2D newGoal = TFPoseToPose2D(robotToGoalTransform);

            if ( isGoalWithinTresholdOf(Treshold::BeforeNextPose, newGoal) )
            {
              ROS_INFO("Switching to next goal");
              break;
            }
            else if ( last_pose && isGoalWithinTresholdOf(Treshold::LastPose, newGoal) )
            {
              abortCurrentGoal();
              return ros::Time::now() - begin;
            }

            float angleDiff = std::atan2(std::sin(newGoal.theta), std::cos(newGoal.theta));

            ROS_INFO("Updating - new goal is ( %.3f %.3f ) theta: %.3f", newGoal.x, newGoal.y, angleDiff);

            m_taskId =  m_motionProxy->post.moveTo(newGoal.x, newGoal.y, newGoal.theta);
            //m_cmdPosePub.publish(newGoal);


            r.sleep();
            if (r.cycleTime() > ros::Duration(1.0 / m_goalUpdatingFrequency)) {
              ROS_WARN("Controller loop of %.3f took %.3f instead of %.3f", 
                m_goalUpdatingFrequency, r.cycleTime().toSec(), 1.0/ m_goalUpdatingFrequency);
            }
          }

        }

      }


      void PathFollower::trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

      geometry_msgs::Pose2D PathFollower::TFPoseToPose2D(tf::Transform tf_pose)
      {
        geometry_msgs::Pose2D pose2D;

        tfScalar r, p, y;
        tf::Matrix3x3(tf_pose.getRotation()).getEulerYPR(y, p, r);

        pose2D.x = tf_pose.getOrigin().x();
        pose2D.y  = tf_pose.getOrigin().y();
        pose2D.theta = y;

        return pose2D;
      }

      void PathFollower::bumperCallback(const naoqi_msgs::Bumper::ConstPtr& msg)
      {
        if (msg->state == 1) {
          ROS_WARN("Robot hit surface with bumpers, aborting goal.");
          abortCurrentGoal();
        }
      }

      float PathFollower::angleDifference(const float& first, const float& second)
      {
        return (first - second );
      }

      void PathFollower::updateGoalTimerCallback(const ros::TimerEvent& event)
      {

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



        if ( isGoalWithinTresholdOf(Treshold::LastPose, newGoal) )
        {
          abortCurrentGoal();
          return;
        }

        float angleDiff = std::atan2(std::sin(newGoal.theta), std::cos(newGoal.theta));

        ROS_INFO("Updating - new goal is ( %.3f %.3f ) theta: %.3f", newGoal.x, newGoal.y, angleDiff);

        m_taskId =  m_motionProxy->post.moveTo(newGoal.x, newGoal.y, newGoal.theta);
        //m_cmdPosePub.publish(newGoal);
      }

      bool PathFollower::isGoalWithinTresholdOf(const Treshold& treshold, const geometry_msgs::Pose2D& goal)
      {

        float xTreshold, yThreshold, yawTreshold;

        if (treshold == Treshold::LastPose)
        {
          xTreshold   = m_xTolerance;
          yThreshold  = m_yTolerance;
          yawTreshold = m_yawTolerance;
        } else if (treshold == Treshold::BeforeNextPose) {
          xTreshold   = m_nextGoalXTolerance;
          yThreshold  = m_nextGoalYTolerance;
          yawTreshold = m_nextGoalYawTolerance;
        } else {
          ROS_FATAL("Received unknown treshold type");
        }

        float angleDiff = std::atan2(std::sin(goal.theta), std::cos(goal.theta));

        if ( (fabs(goal.x) < xTreshold) && 
             (fabs(goal.y) < yThreshold) && 
             (fabs(angleDiff) < yawTreshold) )
        {

          ROS_INFO("Goal within reach of treshold! ( %.3f, %.3f ) theta: %.3f", goal.x, goal.y, angleDiff);
          return true;
        }
        return false;
      }

      void PathFollower::moveBaseGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
      {
        m_moveBaseActionServer.setSucceeded();
      }

      void PathFollower::followPathGoalCallback(const naoqi_msgs::FollowPathGoalConstPtr& goal)
      {
        m_followPathActionServer.setSucceeded();
      }


      void PathFollower::startGoalExecution(geometry_msgs::Pose2D goal)
      {
        m_isGoalRunning = true;
        m_postureProxy->goToPosture("StandInit", 0.5);
        m_updateGoalTimer = m_nh.createTimer(ros::Duration( 1 / m_goalUpdatingFrequency), &PathFollower::updateGoalTimerCallback, this);
      }

      void PathFollower::abortCurrentGoal()
      {
        if(m_isGoalRunning == false)
          return;

        m_isGoalRunning = false;
        ROS_INFO("Aborting goal");
        m_motionProxy->stopMove();
        m_motionProxy->stop(m_taskId);

        /*
        geometry_msgs::Pose2D stopGoal;
        stopGoal.x = 0.0;
        stopGoal.y = 0.0;
        stopGoal.theta = 0.0;
        m_cmdPosePub.publish(stopGoal);
        */
        m_updateGoalTimer.stop();
      }

      bool PathFollower::abortCurrentGoalSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
      {
        abortCurrentGoal();
      }

      void PathFollower::run()
      {
        m_moveBaseSimpleGoalSub = m_nh.subscribe("/move_base_simple/goal", 1000 , &PathFollower::moveBaseSimpleGoalCallback, this);
        m_trackedPoseSub = m_nh.subscribe("/mocap/nao_tracker/pose", 1000, &PathFollower::trackedPoseStampedCallback, this);
        m_bumperSub = m_nh.subscribe("nao/bumper", 1000, &PathFollower::bumperCallback, this);
        m_moveBaseGoalPreemptServer = m_nh.advertiseService("/move_base_simple/abort", &PathFollower::abortCurrentGoalSrvCallback, this);

        m_cmdPosePub = m_nh.advertise<geometry_msgs::Pose2D>("cmd_pose", 10);

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

        m_moveBaseActionServer.start();

        ros::spin();
      }
    }
  }
}
