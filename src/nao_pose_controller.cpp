#include "nao_pose_controller/nao_pose_controller.h"

namespace SNT {
  namespace ARG {
    namespace NaoControllers {
    
      // TODO : Add default values to constructor in order to not have to use hasParam
      PathFollower::PathFollower() :
        m_nh(),
        m_privateNh("~"),
        m_bumperState(false),
        m_footContact(true),
        m_positionFromBaseFootPrint(true),
        m_goalUpdatingFrequency(100.0),
        m_currentRobotPositon(),
        m_currentGoalPosition(),
        m_tfPollingFreq(120.0),
        m_xTolerance(0.2),
        m_yTolerance(0.2),
        m_yawTolerance(0.2),
        m_nextGoalXTolerance(0.2),
        m_nextGoalYTolerance(0.2),
        m_nextGoalYawTolerance(0.2),
        m_robotPoseTimeTh(),
        m_bodyPoseActionClient("body_pose_naoqi", true),
        m_globalFrameId("map"),
        m_basefootprintFrameId("base_footprint"), 
        m_taskId(0),
        m_tfListener(),
        m_moveBaseActionServer(m_nh, "move_base", boost::bind(&PathFollower::moveBaseGoalCallback, this, _1), false),
        m_followPathActionServer(m_nh, "follow_path", boost::bind(&PathFollower::followPathGoalCallback, this, _1), false)

      {
        m_privateNh.param("updating_frequency", m_goalUpdatingFrequency, m_goalUpdatingFrequency);

        m_privateNh.param("x_tolerance", m_xTolerance, m_xTolerance);
        m_privateNh.param("y_tolerance", m_yTolerance, m_yTolerance);
        m_privateNh.param("yaw_tolerance", m_yawTolerance, m_yawTolerance);

        // TODO: refactor these parameters
        m_privateNh.param("next_goal_x_accept_treshold", m_nextGoalXTolerance, m_nextGoalXTolerance);
        m_privateNh.param("next_goal_y_accept_treshold", m_nextGoalYTolerance, m_nextGoalYTolerance);
        m_privateNh.param("next_goal_yaw_accept_treshold", m_nextGoalYawTolerance, m_nextGoalYawTolerance);

        m_privateNh.param("global_frame_id", m_globalFrameId, m_globalFrameId);
        m_privateNh.param("basefootprint_frame_id", m_basefootprintFrameId, m_basefootprintFrameId);

        m_privateNh.param("tf_polling_freq", m_tfPollingFreq, m_tfPollingFreq);
        m_privateNh.param("robot_pose_time_th", m_robotPoseTimeTh, m_robotPoseTimeTh);

        m_privateNh.param("position_from_basefootprint", m_positionFromBaseFootPrint, m_positionFromBaseFootPrint);

        m_basefootprintFrameId = m_tfListener.resolve(m_basefootprintFrameId);

        ROS_INFO("Nao Motion Capture Control Node initialized...");
        ROS_INFO("\t Final pose tolerance values: X: %lf Y: %lf Theta: %lf", m_xTolerance, m_yTolerance, m_yawTolerance);
        ROS_INFO("\t Next goal tolerance values: X: %lf Y: %lf Theta: %lf", m_nextGoalXTolerance, m_nextGoalYTolerance, m_nextGoalYawTolerance);

      }

      PathFollower::~PathFollower()
      {

      }

      void PathFollower::moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
      {

        ROS_DEBUG("Received Move Base Simple message:");
        ROS_DEBUG("\t Position: %.2f %.2f %.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        ROS_DEBUG("\t Quaternion: %.2f %.2f %.2f %.2f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      
        ROS_DEBUG("Received goal on move_base_simple topic, wrapping it up and sending as goal to actionlib server");
        
        move_base_msgs::MoveBaseActionGoal wrapped_goal;
        wrapped_goal.header.stamp = ros::Time::now();
        wrapped_goal.goal.target_pose = *msg;
        m_moveBaseGoalPub.publish(wrapped_goal);
      }

      bool PathFollower::runPoseController(const std::vector<geometry_msgs::PoseStamped>& path)
      {
        ros::Time begin = ros::Time::now();

        if (path.size() == 0)
        {
          ROS_WARN("Received goal's path is empty, finishing goal's execution");
          return true;
        }

        bool last_pose = false;

        int i = 0;
        auto it = path.begin();

        for (; it != path.end(); ++it, ++i)
        {
          ROS_INFO("Proccessing [ %d ] / [ %lu ] goal", i+1, path.size());

          publishPoseToRviz(*it);

          tf::Stamped<tf::Pose> next_pose_tf;
          tf::poseStampedMsgToTF(*it,next_pose_tf);
          m_currentGoalPosition = next_pose_tf;

          if (it + 1 == path.end()) {
            last_pose = true;
            ROS_INFO("Processing the last pose in path");
          }

          ros::Rate r(m_goalUpdatingFrequency);

          while(ros::ok())
          {
            if (m_followPathActionServer.isPreemptRequested() == true)
            {
              ROS_WARN("Received goal preemption request, preempting....");
              return false;
            }

            if (m_bumperState == true )
            {
              stopWalking();
              ROS_WARN("Robot hit bumper!, aborting...");
              return false;
            }
            else if (m_footContact == false)
            {             
              stopWalking();
              ROS_WARN("Robot lost foot contact!, aborting...");
              return false;
            }

            double lastRobotPostionTime = (ros::Time::now() - m_currentRobotPositon.stamp_).toSec();
            if (false && lastRobotPostionTime > m_robotPoseTimeTh)
            {
              ROS_WARN("Last robot position was %.3f ago, treshold is %.3f, aborting...", lastRobotPostionTime, m_robotPoseTimeTh);
              return false;
            }

            tf::Transform robotToGoalTransform = m_currentRobotPositon.inverse() * m_currentGoalPosition;
            geometry_msgs::Pose2D newGoal = TFPoseToPose2D(robotToGoalTransform);

            if ( !last_pose && isGoalWithinTresholdOf(Treshold::BeforeNextPose, newGoal) )
            {
              ROS_INFO("\tSwitching to next goal");
              break;
            }
            else if ( last_pose && isGoalWithinTresholdOf(Treshold::LastPose, newGoal) )
            {
              stopWalking();
              ROS_INFO("Last goal reached ! Sending success result to client");
              return true;
            }

            float angleDiff = std::atan2(std::sin(newGoal.theta), std::cos(newGoal.theta));

            ROS_DEBUG("Updating - new goal is ( %.3f %.3f ) theta: %.3f", newGoal.x, newGoal.y, angleDiff);
            m_cmdPosePub.publish(newGoal);


            r.sleep();
            if (r.cycleTime() > ros::Duration(1.0 / m_goalUpdatingFrequency))
            {
              ROS_WARN("Controller loop of %.3f took %.3f instead of %.3f", 
                m_goalUpdatingFrequency, r.cycleTime().toSec(), 1.0/ m_goalUpdatingFrequency);
            }
          }

        }
        ROS_ERROR("Shouldn't reach this point");
        return false;
      }


      void PathFollower::trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
      {
        tf::Stamped<tf::Pose> trackedPose;
        tf::poseStampedMsgToTF(*msg,trackedPose);

        tfScalar roll, pitch, yaw;
        tf::Matrix3x3(trackedPose.getRotation()).getEulerYPR(yaw, pitch, roll);

        tf::Quaternion trackedPose2DQuaternion = tf::createQuaternionFromYaw(yaw);

        tf::Transform currentRobotPosition = tf::Transform(trackedPose2DQuaternion, 
          tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0L));

        m_currentRobotPositon = tf::StampedTransform(currentRobotPosition, trackedPose.stamp_, trackedPose.frame_id_, m_basefootprintFrameId);

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
        m_bumperState = msg->state;
      }

      void PathFollower::footContactCallback(const std_msgs::Bool::ConstPtr& msg)
      {
        m_footContact = msg->data;
      }

      bool PathFollower::isGoalWithinTresholdOf(const Treshold& treshold, const geometry_msgs::Pose2D& goal)
      {

        float xTreshold, yThreshold, yawTreshold;

        if (treshold == Treshold::LastPose)
        {
          xTreshold   = m_xTolerance;
          yThreshold  = m_yTolerance;
          yawTreshold = m_yawTolerance;
        }
        else if (treshold == Treshold::BeforeNextPose)
        {
          xTreshold   = m_nextGoalXTolerance;
          yThreshold  = m_nextGoalYTolerance;
          yawTreshold = m_nextGoalYawTolerance;
        }
        else
        {
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
        if (startWalking(10.0) == false)
          m_moveBaseActionServer.setAborted(move_base_msgs::MoveBaseResult(), "Failed");


        ROS_DEBUG("INSIDE MOVE BASE GOAL CALLBACK");

        std::vector<geometry_msgs::PoseStamped> path;
        path.push_back(goal->target_pose);
        bool result = runPoseController(path);

        if (result)
          m_moveBaseActionServer.setSucceeded(move_base_msgs::MoveBaseResult(), "Succeeded");
        else
          m_moveBaseActionServer.setAborted(move_base_msgs::MoveBaseResult(), "Failed");
      }

      void PathFollower::followPathGoalCallback(const naoqi_msgs::FollowPathGoalConstPtr& goal)
      {
        if (startWalking(10.0) == false)
          m_followPathActionServer.setAborted(naoqi_msgs::FollowPathResult(), "Failed");
        
        ROS_DEBUG("INSIDE FOLLOW PATH GOAL CALLBACK");

        m_visPathPub.publish(goal->path);

        bool result = runPoseController(goal->path.poses);

        if (result)
          m_followPathActionServer.setSucceeded(naoqi_msgs::FollowPathResult(), "Succeeded");
        else
          m_followPathActionServer.setAborted(naoqi_msgs::FollowPathResult(), "Failed");

      }

      void PathFollower::stopWalking()
      {
        ROS_INFO("Aborting goal");

        std_srvs::Empty e;
        m_stopWalkClient.call(e);

      }

      bool PathFollower::startWalking(float max_prep_time)
      {
        ROS_INFO("Going Stand Init position");
        naoqi_msgs::BodyPoseWithSpeedGoal goal;
        goal.posture_name = "StandInit";
        goal.speed = 0.7;
        m_bodyPoseActionClient.sendGoal(goal);

        bool finishedBeforeTimeout = m_bodyPoseActionClient.waitForResult(ros::Duration(max_prep_time));

        if (finishedBeforeTimeout)
        {
          ROS_INFO("\t Robot in StandInit posture");
          return true;
        }
        else
        {
          ROS_INFO("\t Robot couldn't get to desired posture in required %.3f s", max_prep_time);
          return false;
        }

      }

      void PathFollower::updateRobotPositionCallback(const ros::TimerEvent&)
      {
        if (m_tfListener.canTransform(m_globalFrameId, m_basefootprintFrameId, ros::Time(0)) == false)
        {
          ROS_DEBUG("Cannot transform from torso frame: %s to sensor frame: %s", m_globalFrameId.c_str(), m_basefootprintFrameId.c_str());
          return;
        }

        try
        {
          m_tfListener.lookupTransform(m_globalFrameId, m_basefootprintFrameId, ros::Time(0), m_currentRobotPositon);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }

      void PathFollower::publishPoseToRviz(const geometry_msgs::PoseStamped& pose)
      {
        ROS_INFO("Publishing pose for RVIZ");
        m_visTargetPosePub.publish(pose);
      }

      void PathFollower::run()
      {
        if (m_positionFromBaseFootPrint) {
          m_robotPositionPollTimer = m_nh.createTimer(ros::Duration(1.0 / m_tfPollingFreq), &PathFollower::updateRobotPositionCallback, this);      
        } else {
          m_trackedPoseSub = m_nh.subscribe("/marker/pose", 1000, &PathFollower::trackedPoseStampedCallback, this);
        }      

        m_moveBaseSimpleGoalSub = m_nh.subscribe("/move_base_simple/goal", 1000 , &PathFollower::moveBaseSimpleGoalCallback, this);
        m_bumperSub = m_nh.subscribe("bumper", 1, &PathFollower::bumperCallback, this);
        m_footContactSub = m_nh.subscribe("foot_contact", 1, &PathFollower::footContactCallback, this);

        m_cmdPosePub       = m_nh.advertise<geometry_msgs::Pose2D>("cmd_pose", 10);
        m_moveBaseGoalPub  = m_nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
        m_visTargetPosePub = m_nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
        m_visPathPub       = m_nh.advertise<nav_msgs::Path>("path", 1);

        m_stopWalkClient = m_nh.serviceClient<std_srvs::Empty>("stop_walk_srv");
        
        m_bodyPoseActionClient.waitForServer();

        m_moveBaseActionServer.start();
        m_followPathActionServer.start();

        ros::spin();
      }
    }
  }
}
