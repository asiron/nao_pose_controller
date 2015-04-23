#ifndef NAO_POSE_CONTROLLER_GUARD_H
#define NAO_POSE_CONTROLLER_GUARD_H 1

#include <mutex>
#include <thread>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_datatypes.h>

// Messages
#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <naoqi_msgs/Bumper.h>
#include <naoqi_msgs/FollowPathAction.h>

#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>

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

namespace SNT {
  namespace ARG {
    namespace NaoControllers {
      
      struct Treshold
      {
          enum Type
          {
              LastPose, BeforeNextPose
          };
          Type t_;
          Treshold(Type t) : t_(t) {}
          operator Type () const {return t_;}

      private:
         //prevent automatic conversion
         template<typename T> operator T () const;
      };

      class PathFollower
      {

      public:

        typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseServer;
        typedef actionlib::SimpleActionServer<naoqi_msgs::FollowPathAction> FollowPathServer;

        PathFollower();
        ~PathFollower();

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

        double m_nextGoalXTolerance;
        double m_nextGoalYTolerance;
        double m_nextGoalYawTolerance; 

        ros::Timer m_updateGoalTimer;

        ros::Subscriber m_moveBaseSimpleGoalSub;
        ros::Subscriber m_trackedPoseSub;
        ros::Subscriber m_bumperSub;

        ros::Publisher  m_cmdPosePub;

        ros::ServiceServer m_moveBaseGoalPreemptServer;

        MoveBaseServer m_moveBaseActionServer;
        FollowPathServer m_followPathActionServer;

        boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
        boost::shared_ptr<AL::ALRobotPostureProxy> m_postureProxy;

        tf::Transform m_currentRobotPositon;
        tf::Transform m_currentGoalPosition;

        bool m_isGoalRunning;
        std::mutex goal_mtx;

        void startGoalExecution(geometry_msgs::Pose2D goal);
        void abortCurrentGoal();

        geometry_msgs::Pose2D TFPoseToPose2D(tf::Transform tf_pose);

        bool isGoalWithinTresholdOf(const Treshold& treshold, const geometry_msgs::Pose2D& goal);

        // Timer callbacks
        void updateGoalTimerCallback(const ros::TimerEvent&);

        // Service callbacks
        bool abortCurrentGoalSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        // Subscriber callbacks
        void moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void bumperCallback(const naoqi_msgs::Bumper::ConstPtr& msg);

        // Actionlib Server callbacks
        void moveBaseGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);

        void followPathGoalCallback(const naoqi_msgs::FollowPathGoalConstPtr& goal);


        ros::Duration runPoseController(const std::vector<geometry_msgs::PoseStamped>& path);

        float angleDifference(const float& first, const float& second);
      };
    }
  }
}

#endif