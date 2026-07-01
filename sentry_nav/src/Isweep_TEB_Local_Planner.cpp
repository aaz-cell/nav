#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

double Distance2d(const geometry_msgs::PoseStamped& a,
                  const geometry_msgs::PoseStamped& b) {
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::Quaternion YawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

class IsweepTebLocalPlannerNode {
 public:
  IsweepTebLocalPlannerNode()
      : pnh_("~"),
        tf_buffer_(ros::Duration(10.0)),
        tf_listener_(tf_buffer_),
        costmap_ros_("local_costmap", tf_buffer_) {
    LoadParams();

    plan_sub_ = nh_.subscribe(global_plan_topic_, 1,
                              &IsweepTebLocalPlannerNode::PlanCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 10, true);

    costmap_ros_.start();
    teb_planner_.initialize("TebLocalPlannerROS", &tf_buffer_, &costmap_ros_);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
                             &IsweepTebLocalPlannerNode::TimerCallback, this);

    ROS_INFO("isweep_teb_local_planner ready: plan=%s cmd=%s status=%s",
             global_plan_topic_.c_str(), cmd_vel_topic_.c_str(),
             status_topic_.c_str());
  }

  ~IsweepTebLocalPlannerNode() {
    if (publish_zero_on_shutdown_) {
      PublishZero("shutdown");
    }
  }

 private:
  void LoadParams() {
    pnh_.param<std::string>("global_plan_topic", global_plan_topic_,
                            "/isweep_planner/trajectory");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                            "/isweep_teb_cmd_vel");
    pnh_.param<std::string>("status_topic", status_topic_,
                            "/isweep_teb_local_planner/status");
    pnh_.param<double>("control_rate", control_rate_hz_, 10.0);
    pnh_.param<double>("plan_timeout", plan_timeout_sec_, 0.0);
    pnh_.param<int>("min_plan_poses", min_plan_poses_, 2);
    pnh_.param<bool>("publish_zero_when_idle", publish_zero_when_idle_, true);
    pnh_.param<bool>("publish_zero_on_shutdown", publish_zero_on_shutdown_, true);

    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    plan_timeout_sec_ = std::max(0.0, plan_timeout_sec_);
    min_plan_poses_ = std::max(2, min_plan_poses_);
  }

  void PlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<geometry_msgs::PoseStamped> plan = NormalizePlan(*msg);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_plan_ = std::move(plan);
    latest_plan_stamp_ = ros::Time::now();
    have_plan_ = static_cast<int>(latest_plan_.size()) >= min_plan_poses_;
    plan_dirty_ = have_plan_;
  }

  std::vector<geometry_msgs::PoseStamped> NormalizePlan(
      const nav_msgs::Path& msg) const {
    std::vector<geometry_msgs::PoseStamped> plan;
    plan.reserve(msg.poses.size());
    for (size_t i = 0; i < msg.poses.size(); ++i) {
      geometry_msgs::PoseStamped pose = msg.poses[i];
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = msg.header.frame_id.empty() ? "map" : msg.header.frame_id;
      }
      if (pose.header.stamp.isZero()) {
        pose.header.stamp = msg.header.stamp.isZero() ? ros::Time::now()
                                                      : msg.header.stamp;
      }
      if (!plan.empty() && Distance2d(plan.back(), pose) < 1e-3) {
        continue;
      }
      plan.push_back(pose);
    }
    // iSweep stores full SE(2) body yaw in its Path. For TEB this global plan
    // yaw must describe path progression; otherwise TEB may track the same
    // spatial path by commanding reverse motion.
    for (size_t i = 0; i < plan.size(); ++i) {
      const size_t prev = i == 0 ? i : i - 1;
      const size_t next = (i + 1 < plan.size()) ? i + 1 : i;
      const double dx = plan[next].pose.position.x - plan[prev].pose.position.x;
      const double dy = plan[next].pose.position.y - plan[prev].pose.position.y;
      if (std::hypot(dx, dy) > 1e-6) {
        plan[i].pose.orientation = YawToQuaternion(std::atan2(dy, dx));
      }
    }
    return plan;
  }

  void TimerCallback(const ros::TimerEvent&) {
    std::vector<geometry_msgs::PoseStamped> plan;
    bool set_plan = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_plan_ ||
          (plan_timeout_sec_ > 0.0 &&
           (ros::Time::now() - latest_plan_stamp_).toSec() > plan_timeout_sec_)) {
        PublishIdleStatus("waiting for fresh iSweep global path");
        if (publish_zero_when_idle_) {
          PublishZero("no fresh plan");
        }
        return;
      }
      plan = latest_plan_;
      set_plan = plan_dirty_;
      plan_dirty_ = false;
    }

    if (set_plan && !teb_planner_.setPlan(plan)) {
      PublishStatus("ERROR", "TEB rejected iSweep global path");
      if (publish_zero_when_idle_) {
        PublishZero("setPlan failed");
      }
      return;
    }

    geometry_msgs::Twist cmd;
    if (teb_planner_.isGoalReached()) {
      PublishStatus("GOAL_REACHED", "TEB reports goal reached");
      PublishZero("goal reached");
      return;
    }

    if (!teb_planner_.computeVelocityCommands(cmd)) {
      PublishStatus("BLOCKED", "TEB failed to compute velocity command");
      if (publish_zero_when_idle_) {
        PublishZero("computeVelocityCommands failed");
      }
      return;
    }

    cmd_pub_.publish(cmd);
    PublishStatus("OK", "TEB tracking iSweep global path");
  }

  void PublishZero(const char* reason) {
    geometry_msgs::Twist zero;
    cmd_pub_.publish(zero);
    ROS_WARN_THROTTLE(2.0, "isweep_teb_local_planner publishing zero (%s)",
                      reason);
  }

  void PublishIdleStatus(const std::string& detail) {
    PublishStatus("IDLE", detail);
  }

  void PublishStatus(const std::string& state, const std::string& detail) {
    std_msgs::String msg;
    msg.data = state + "|" + detail;
    status_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  costmap_2d::Costmap2DROS costmap_ros_;
  teb_local_planner::TebLocalPlannerROS teb_planner_;

  ros::Subscriber plan_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  std::mutex mutex_;
  std::vector<geometry_msgs::PoseStamped> latest_plan_;
  ros::Time latest_plan_stamp_;
  bool have_plan_ = false;
  bool plan_dirty_ = false;

  std::string global_plan_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  double control_rate_hz_ = 10.0;
  double plan_timeout_sec_ = 0.0;
  int min_plan_poses_ = 2;
  bool publish_zero_when_idle_ = true;
  bool publish_zero_on_shutdown_ = true;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "isweep_teb_local_planner");
  IsweepTebLocalPlannerNode node;
  ros::spin();
  return 0;
}
