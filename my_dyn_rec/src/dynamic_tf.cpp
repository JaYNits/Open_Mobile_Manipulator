#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <my_dyn_rec/lidartfConfig.h>

void callback(my_dyn_rec::lidartfConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f", config.lidar_px,
           config.lidar_py, config.lidar_pz, config.lidar_or, config.lidar_op,
           config.lidar_oy);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(config.lidar_px, config.lidar_py, config.lidar_pz) );
  tf::Quaternion q;
  q.setRPY(config.lidar_or, config.lidar_op, config.lidar_oy);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "chassis", "laser_link"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_dyn_tf");

  dynamic_reconfigure::Server<my_dyn_rec::lidartfConfig> server;
  dynamic_reconfigure::Server<my_dyn_rec::lidartfConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}