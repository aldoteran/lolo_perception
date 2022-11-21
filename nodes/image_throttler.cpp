/*
 * @file image_publisher.cpp
 * @brief ROS node that rectifies the a compressed image topic.
 * @data Jun 12, 2022
 * @author aldo teran (aldot@kth.se)
 */

#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

// Sorry for this. Global vars for publishers.
ros::Publisher throttled_image_pub;
ros::Publisher throttled_camera_info_pub;
sensor_msgs::CameraInfo camera_info_msg;
bool camera_info_init = false;
int counter = 0;

void image_callback(const sensor_msgs::CompressedImage& msg) {
  // Make sure we have the CameraInfo first.
  if (!camera_info_init){ return; }
  // Cameras running at 60FPS, we're gonna throttle to 20FPS.
  if (!counter%4 == 0) { return; }

  // Fake-sync CameraInfo to Image.
  camera_info_msg.header.stamp = msg.header.stamp;

  // Publish all.
  throttled_camera_info_pub.publish(camera_info_msg);
  throttled_image_pub.publish(msg);
}

void camera_info_callback(const sensor_msgs::CameraInfo& msg) {
  camera_info_msg = msg;
  camera_info_init = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_throttler", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  const std::string compressed_topic = nh.resolveName("image_in");
  const std::string camera_info_topic = nh.resolveName("cam_info_in");
  const std::string out_image_topic = nh.resolveName("image_out");
  const std::string out_camera_info_topic = nh.resolveName("cam_info_out");

  ros::Subscriber cam_info_sub = nh.subscribe(camera_info_topic, 1, camera_info_callback);
  ros::Subscriber image_sub = nh.subscribe(compressed_topic, 1, image_callback);

  throttled_image_pub = nh.advertise<sensor_msgs::CompressedImage>(out_image_topic, 1);
  throttled_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(out_camera_info_topic, 1);

  // Hardcoding 20 FPS.
  ros::Rate loop_rate(20);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
