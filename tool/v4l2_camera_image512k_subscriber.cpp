// Copyright 2021 Apex.AI, Inc.
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

#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using namespace shm_msgs;
class Listener : public rclcpp::Node {
private:
  using Topic = shm_msgs::msg::Image512k;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_image512k_subscriber", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {

      RCLCPP_INFO(this->get_logger(), "Received...");
      m_last_cvimage = shm_msgs::toCvShare(msg);
      // m_last_cvimage = shm_msgs::toCvCopy(msg);
      // RCLCPP_INFO(this->get_logger(), "image_encoding(%s), frameid(%s)", (shm_msgs::get_str(msg->encoding)).c_str(), (shm_msgs::get_str(msg->header.frame_id)).c_str());

      auto time_offset_ns = (now() - m_last_cvimage->header.stamp).nanoseconds();
      auto timestamp_offset_ns = (rclcpp::Time(msg->header.stamp) - m_last_image_ts).nanoseconds();
      auto time_offset_ms = time_offset_ns / 1000000.0F;
      auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
      RCLCPP_INFO(get_logger(), "get-image512k-transport-time: %.3f", time_offset_ms);
      if(m_last_image_ts.nanoseconds() > 0.0)
      {
        RCLCPP_INFO(get_logger(), "get-image512k-timestamp_offset-time: %.3f", timestamp_offset_ms);
      }
      m_last_image_ts = msg->header.stamp;

      // cv::imshow("im show", m_last_cvimage->image);
      // cv::waitKey(0);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("shm_image_512k", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  shm_msgs::CvImageConstPtr m_last_cvimage;
  rclcpp::Time m_last_image_ts{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}