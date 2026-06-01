#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
// tf chage https://zhuanlan.zhihu.com/p/340016739


int main(int argc, char** argv){
  ros::init(argc, argv, "Trans_TF_2d");

  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform_broadcaster;
  ros::Duration(1.0).sleep();

  ros::Rate rate(1000);
  while (node.ok()){
    tf::StampedTransform transform_listener;
    
    try{
      listener.lookupTransform("map", "body",  
                               ros::Time(0), transform_listener);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    float robot_pose_x=transform_listener.getOrigin().x();
    float robot_pose_y=transform_listener.getOrigin().y();
    float robot_pose_z=0;
    double robot_yaw = tf::getYaw(transform_listener.getRotation());
    tf::Quaternion yaw_only_quaternion;
    yaw_only_quaternion.setRPY(0.0, 0.0, robot_yaw);

    transform_broadcaster.setOrigin( tf::Vector3(robot_pose_x, robot_pose_y, 0.0) );
    transform_broadcaster.setRotation(yaw_only_quaternion);
    broadcaster.sendTransform(tf::StampedTransform(transform_broadcaster, ros::Time::now(), "map", "body_2d"));
    

    rate.sleep();
  }
  return 0;
};
