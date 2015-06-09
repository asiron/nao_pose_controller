#ifndef NAO_POSE_CONTROLLER_GUARD_H
#define NAO_POSE_CONTROLLER_GUARD_H 1

#include <mutex>
#include <thread>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Messages
#include <std_srvs/Empty.h>

#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <naoqi_msgs/Bumper.h>
#include <naoqi_msgs/FollowPathAction.h>
#include <naoqi_msgs/BodyPoseWithSpeedAction.h>

#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>

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
        typedef actionlib::SimpleActionClient<naoqi_msgs::BodyPoseWithSpeedAction> BodyPoseClient;

        PathFollower();
        ~PathFollower();

        void run();

      private:

        ros::NodeHandle m_nh, m_privateNh;

        double m_goalUpdatingFrequency;

        int m_taskId;

        bool m_bumperState;
        bool m_footContact;

        bool m_positionFromBaseFootPrint;

        double m_xTolerance;
        double m_yTolerance;
        double m_yawTolerance; 

        double m_nextGoalXTolerance;
        double m_nextGoalYTolerance;
        double m_nextGoalYawTolerance; 

        double m_tfPollingFreq;
        double m_robotPoseTimeTh;


        std::string m_globalFrameId;
        std::string m_basefootprintFrameId;

        ros::Timer m_robotPositionPollTimer;

        ros::Subscriber m_moveBaseSimpleGoalSub;
        ros::Subscriber m_trackedPoseSub;
        ros::Subscriber m_bumperSub;
        ros::Subscriber m_footContactSub;

        ros::Publisher  m_cmdPosePub;
        ros::Publisher  m_moveBaseGoalPub;
        ros::Publisher  m_visTargetPosePub;
        ros::Publisher  m_visPathPub;

        ros::ServiceClient m_stopWalkClient;

        MoveBaseServer m_moveBaseActionServer;
        FollowPathServer m_followPathActionServer;
        BodyPoseClient m_bodyPoseActionClient;

        tf::TransformListener m_tfListener;

        tf::StampedTransform m_currentRobotPositon;
        tf::Transform m_currentGoalPosition;

        void stopWalking();
        bool startWalking(float max_prep_time);
        bool finishWalking(float max_prep_time);
        bool goToPosture(std::string posture_name, float max_prep_time);

        void publishPoseToRviz(const geometry_msgs::PoseStamped& pose);

        // Helpers
        geometry_msgs::Pose2D TFPoseToPose2D(tf::Transform tf_pose);
        bool isGoalWithinTresholdOf(const Treshold& treshold, const geometry_msgs::Pose2D& goal);

        // Timer callbacks
        void updateRobotPositionCallback(const ros::TimerEvent&);

        // Subscriber callbacks
        void moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void trackedPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void bumperCallback(const naoqi_msgs::Bumper::ConstPtr& msg);
        void footContactCallback(const std_msgs::Bool::ConstPtr& msg);

        // Actionlib Server callbacks
        void moveBaseGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
        void followPathGoalCallback(const naoqi_msgs::FollowPathGoalConstPtr& goal);

        bool runPoseController(const std::vector<geometry_msgs::PoseStamped>& path);

      };
    }
  }
}

#endif