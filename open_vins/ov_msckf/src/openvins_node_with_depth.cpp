#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS1Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_msckf;

// Declare shared pointers for the VIO system and visualizer
std::shared_ptr<VioManager> sys;
std::shared_ptr<ROS1Visualizer> viz;

// Callback for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // Forward the IMU data to the visual-inertial odometry system
  viz->callback_inertial(msg);
}

// Callback for camera RGB image data
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  // Pass RGB image to the VIO visualizer
  viz->callback_monocular(msg, 0); // Assuming monocular camera; update if stereo
}

// Callback for depth image data
void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
  // Optional: Implement depth callback logic if required by the VIO system
  // sys->addDepth(...); // Use if your VIO system supports depth integration
}

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "openvins_node_with_depth");
  ros::NodeHandle nh("~");

  // Load the configuration file path, either from command line or parameter server
  std::string config_path = "unset_path_to_config.yaml";
  nh.param<std::string>("config_path", config_path, config_path);

  // Load VIO configuration
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(std::make_shared<ros::NodeHandle>(nh));

  // Set verbosity level
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create VIO system and visualizer
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<ROS1Visualizer>(std::make_shared<ros::NodeHandle>(nh), sys);

  // Check if configuration parsing was successful
  if (!parser->successful()) {
    PRINT_ERROR(RED "[NODE]: Unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Subscribe to IMU topic
  std::string topic_imu = "/imu0";
  nh.param<std::string>("topic_imu", topic_imu, topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  ros::Subscriber imu_sub = nh.subscribe(topic_imu, 1000, imuCallback);

  // Subscribe to camera topic
  std::string topic_camera_rgb = "/camera/color/image_raw";
  nh.param<std::string>("topic_camera_rgb", topic_camera_rgb, topic_camera_rgb);
  ros::Subscriber img_sub = nh.subscribe(topic_camera_rgb, 1000, imageCallback);

  // Subscribe to depth topic (assuming depth data is needed)
  std::string topic_camera_depth = "/camera/depth/image_raw";
  nh.param<std::string>("topic_camera_depth", topic_camera_depth, topic_camera_depth);
  ros::Subscriber depth_sub = nh.subscribe(topic_camera_depth, 1000, depthCallback);

  // Log subscribed topics
  ROS_INFO("Subscribed to IMU topic: %s", topic_imu.c_str());
  ROS_INFO("Subscribed to RGB camera topic: %s", topic_camera_rgb.c_str());
  ROS_INFO("Subscribed to depth camera topic: %s", topic_camera_depth.c_str());

  // Spin to keep the node running and processing incoming data
  ros::spin();

  return EXIT_SUCCESS;
}
