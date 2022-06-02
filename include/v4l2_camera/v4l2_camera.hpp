// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include <memory>
#include <string>
#include <map>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "v4l2_camera/visibility_control.h"

namespace v4l2_camera
{

template<typename Topic>
class V4L2Camera : public rclcpp::Node
{
public:
  explicit V4L2Camera(rclcpp::NodeOptions const & options, bool const & is_shm);

  virtual ~V4L2Camera();

private:
  std::shared_ptr<V4l2CameraDevice<Topic>> camera_;

  // Publisher used for intra process comm
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  // Publisher used for shm_msg transport comm
  typename rclcpp::Publisher<Topic>::SharedPtr image_shm_pub_;
  std::string m_shm_topic_name = "image_shm_invalid";

  // Publisher used for inter process comm
  image_transport::CameraPublisher camera_transport_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  std::thread capture_thread_;
  std::atomic<bool> canceled_;

  bool is_shm_{false};

  std::string camera_frame_id_;
  std::string output_encoding_;

  std::map<std::string, int32_t> control_name_to_id_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  void createParameters();
  bool handleParameter(rclcpp::Parameter const & param);

  bool requestPixelFormat(std::string const & fourcc);
  bool requestImageSize(std::vector<int64_t> const & size);

  sensor_msgs::msg::Image::UniquePtr convert(sensor_msgs::msg::Image const & img) const;
  void convert_shm(Topic & img) const;

  bool checkCameraInfo(
    sensor_msgs::msg::Image const & img,
    sensor_msgs::msg::CameraInfo const & ci);
  bool checkCameraInfo_shm(
    Topic const & img,
    sensor_msgs::msg::CameraInfo const & ci);
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_HPP_
