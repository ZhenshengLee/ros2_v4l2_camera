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

#include "v4l2_camera/v4l2_camera.hpp"

#include <memory>

class ImageSubscriber : public rclcpp::Node
{
public:
  explicit ImageSubscriber(rclcpp::NodeOptions const & options)
  : rclcpp::Node{"image_subscriber", options}
  {
    img_sub_ =
      create_subscription<sensor_msgs::msg::Image>(
      "/image_raw",
      10,
      [this](sensor_msgs::msg::Image::UniquePtr img) {
        std::stringstream ss;
        ss << "Image message address [RECEIVE]:\t" << img.get();
        auto time_offset_ns = (now() - img->header.stamp).nanoseconds();
        auto timestamp_offset_ns = (rclcpp::Time(img->header.stamp) - m_last_image_ts).nanoseconds();
        auto time_offset_ms = time_offset_ns / 1000000.0F;
        auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
        RCLCPP_INFO(get_logger(), "get-image-transport-time: %.3f", time_offset_ms);
        if(m_last_image_ts.nanoseconds() > 0.0)
        {
          RCLCPP_INFO(get_logger(), "get-image-timestamp_offset-time: %.3f", timestamp_offset_ms);
        }
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Time m_last_image_ts{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec{};
  rclcpp::NodeOptions options;

  auto image_subscriber = std::make_shared<ImageSubscriber>(options);

  exec.add_node(image_subscriber);

  exec.spin();

  rclcpp::shutdown();
  image_subscriber = nullptr;

  return 0;
}
