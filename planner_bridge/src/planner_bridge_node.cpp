#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace {

class PlannerBridge {
 public:
  PlannerBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    LoadParams();

    local_cmd_sub_ = nh_.subscribe(local_cmd_topic_, 20, &PlannerBridge::LocalCmdCallback, this);
    replan_sub_ = nh_.subscribe(replan_in_topic_, 20, &PlannerBridge::ReplanCallback, this);
    status_sub_ = nh_.subscribe(local_status_topic_, 20, &PlannerBridge::StatusCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_out_topic_, 20);
    need_replan_pub_ = nh_.advertise<std_msgs::Bool>(need_replan_topic_, 10, true);
    status_pub_ = nh_.advertise<std_msgs::String>(status_out_topic_, 10, true);

    PublishZero("startup");
    ROS_INFO("planner_bridge ready: %s -> %s, %s -> %s", local_cmd_topic_.c_str(),
             cmd_vel_out_topic_.c_str(), replan_in_topic_.c_str(), need_replan_topic_.c_str());
  }

 private:
  void LoadParams() {
    pnh_.param<std::string>("local_cmd_topic", local_cmd_topic_, "/isweep_planner/local_cmd");
    pnh_.param<std::string>("cmd_vel_out_topic", cmd_vel_out_topic_, "/isweep_cmd_vel");
    pnh_.param<std::string>("replan_in_topic", replan_in_topic_, "/isweep_planner/replan_request");
    pnh_.param<std::string>("need_replan_topic", need_replan_topic_, "/need_replan");
    pnh_.param<std::string>("local_status_topic", local_status_topic_,
                            "/isweep_planner/local_planner_status");
    pnh_.param<std::string>("status_out_topic", status_out_topic_, "/isweep_status");
    pnh_.param<bool>("enable_cmd_output", enable_cmd_output_, false);
    pnh_.param<bool>("publish_zero_when_disabled", publish_zero_when_disabled_, true);
  }

  void LocalCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_ = *msg;
    if (enable_cmd_output_) {
      cmd_vel_pub_.publish(last_cmd_);
    } else if (publish_zero_when_disabled_) {
      PublishZero("disabled");
    }
  }

  void ReplanCallback(const std_msgs::Bool::ConstPtr& msg) {
    need_replan_pub_.publish(*msg);
  }

  void StatusCallback(const std_msgs::String::ConstPtr& msg) {
    status_pub_.publish(*msg);
  }

  void PublishZero(const char* reason) {
    geometry_msgs::Twist zero;
    cmd_vel_pub_.publish(zero);
    ROS_INFO_THROTTLE(5.0, "planner_bridge holding zero cmd_vel (%s)", reason);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber local_cmd_sub_;
  ros::Subscriber replan_sub_;
  ros::Subscriber status_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher need_replan_pub_;
  ros::Publisher status_pub_;

  geometry_msgs::Twist last_cmd_;
  std::string local_cmd_topic_;
  std::string cmd_vel_out_topic_;
  std::string replan_in_topic_;
  std::string need_replan_topic_;
  std::string local_status_topic_;
  std::string status_out_topic_;
  bool enable_cmd_output_ = false;
  bool publish_zero_when_disabled_ = true;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  PlannerBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
