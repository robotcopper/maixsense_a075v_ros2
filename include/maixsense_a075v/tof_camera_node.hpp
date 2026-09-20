#ifndef MAIXSENSE_A075V__TOF_CAMERA_NODE_HPP_
#define MAIXSENSE_A075V__TOF_CAMERA_NODE_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <atomic>
#include <memory>
#include <thread>

#include "maixsense_a075v/device.hpp"
#include "maixsense_a075v/maixsense_a075v_parameters.hpp"

namespace maixsense_a075v
{

class TofCameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TofCameraNode(const rclcpp::NodeOptions & options);
  ~TofCameraNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  AcquisitionConfig acquisition_config(uint8_t trigger_mode) const;
  PipelineConfig pipeline_config() const;
  void initialize_calibration_messages();
  void broadcast_static_transforms();
  void acquisition_loop();
  void publish_frame(const DecodedFrame & frame);
  bool update_runtime_config();
  void stop_acquisition();
  void release_resources();

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::unique_ptr<Device> device_;
  sensor_msgs::msg::CameraInfo tof_camera_info_;
  sensor_msgs::msg::CameraInfo rgb_camera_info_;

  std::atomic<bool> acquiring_{false};
  std::thread acquisition_thread_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
    color_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
    depth_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
    intensity_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
    status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    tof_camera_info_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    rgb_camera_info_publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
    static_transform_broadcaster_;
};

}  // namespace maixsense_a075v

#endif  // MAIXSENSE_A075V__TOF_CAMERA_NODE_HPP_
