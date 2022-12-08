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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include "v4l2_camera/fourcc.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Explicit Specialization
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image8k>;
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image512k>;
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image1m>;
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image2m>;
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image4m>;
template class v4l2_camera::V4L2Camera<shm_msgs::msg::Image8m>;

namespace v4l2_camera
{

template<typename Topic>
V4L2Camera<Topic>::V4L2Camera(rclcpp::NodeOptions const & options, bool const & is_shm)
: rclcpp::Node{"v4l2_camera_node", options},
  canceled_{false},
  is_shm_{is_shm}
{
  if(typeid(Topic) == typeid(shm_msgs::msg::Image8k)) {
    m_shm_topic_name = "shm_image_8k";
  }
  if(typeid(Topic) == typeid(shm_msgs::msg::Image512k)) {
    m_shm_topic_name = "shm_image_512k";
  }
  if(typeid(Topic) == typeid(shm_msgs::msg::Image1m)) {
    m_shm_topic_name = "shm_image_1m";
  }
  if(typeid(Topic) == typeid(shm_msgs::msg::Image2m)) {
    m_shm_topic_name = "shm_image_2m";
  }
  if(typeid(Topic) == typeid(shm_msgs::msg::Image4m)) {
    m_shm_topic_name = "shm_image_4m";
  }
  if(typeid(Topic) == typeid(shm_msgs::msg::Image8m)) {
    m_shm_topic_name = "shm_image_8m";
  }
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters

  if (is_shm_) {
    image_shm_pub_ = create_publisher<Topic>(m_shm_topic_name, 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  } else if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  } else {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, "image_raw");
  }

  // Prepare camera
  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Path to video device";
  device_descriptor.read_only = true;
  auto device = declare_parameter<std::string>("video_device", "/dev/video0", device_descriptor);
  if(is_shm_)
  {
    camera_ = std::make_shared<V4l2CameraDevice<Topic>>(device, true);
  }
  else
  {
    camera_ = std::make_shared<V4l2CameraDevice<Topic>>(device, false);
  }



  if (!camera_->open()) {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  // Read parameters and set up callback
  createParameters();

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      while (rclcpp::ok() && !canceled_.load()) {
        if(is_shm_)
        {
          RCLCPP_DEBUG(get_logger(), "Capture_shm...");
          auto loaned_msg = image_shm_pub_->borrow_loaned_message();

          // populateLoanedMessage-start
          auto &img = loaned_msg.get();
          if (!camera_->capture_shm(img)) {
            // Failed capturing image, assume it is temporarily and continue a bit later
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }

          auto stamp = now();
          if ((shm_msgs::get_str(img.encoding)) != output_encoding_) {
            RCLCPP_WARN_ONCE(
              get_logger(),
              "Image encoding not the same as requested output, performing possibly slow conversion: "
              "%s => %s",
              (shm_msgs::get_str(img.encoding)).c_str(), output_encoding_.c_str());
            convert_shm(img);
          }
          img.header.stamp = stamp;
          shm_msgs::set_str(img.header.frame_id, camera_frame_id_);
          RCLCPP_DEBUG(get_logger(), "image_encoding(%s), frameid(%s)", (shm_msgs::get_str(img.encoding)).c_str(), (shm_msgs::get_str(img.header.frame_id)).c_str());
          // populateLoanedMessage-end

          // debug: check the shm_msg
          // shm_msgs::CvImageConstPtr shm_cvimage = shm_msgs::toCvCopy(std::make_shared<Topic>(img));
          // cv::imshow("im show", shm_cvimage->image);
          // cv::waitKey(0);

          auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
          if (!checkCameraInfo_shm(img, *ci)) {
            *ci = sensor_msgs::msg::CameraInfo{};
            ci->height = img.height;
            ci->width = img.width;
          }

          ci->header.stamp = stamp;

          image_shm_pub_->publish(std::move(loaned_msg));
          info_pub_->publish(std::move(ci));
        }
        else
        {
          RCLCPP_DEBUG(get_logger(), "Capture...");
          auto img = camera_->capture();
          if (img == nullptr) {
            // Failed capturing image, assume it is temporarily and continue a bit later
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }

          auto stamp = now();
          if (img->encoding != output_encoding_) {
            RCLCPP_WARN_ONCE(
              get_logger(),
              "Image encoding not the same as requested output, performing possibly slow conversion: "
              "%s => %s",
              img->encoding.c_str(), output_encoding_.c_str());
            img = convert(*img);
          }
          img->header.stamp = stamp;
          img->header.frame_id = camera_frame_id_;

          auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
          if (!checkCameraInfo(*img, *ci)) {
            *ci = sensor_msgs::msg::CameraInfo{};
            ci->height = img->height;
            ci->width = img->width;
          }

          ci->header.stamp = stamp;

          if (get_node_options().use_intra_process_comms()) {
            RCLCPP_DEBUG_STREAM(get_logger(), "Image message address [PUBLISH]:\t" << img.get());
            image_pub_->publish(std::move(img));
            info_pub_->publish(std::move(ci));
          } else {
            camera_transport_pub_.publish(*img, *ci);
          }
        }
      }
    }
  };
}

template<typename Topic>
V4L2Camera<Topic>::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

template<typename Topic>
void V4L2Camera<Topic>::createParameters()
{
  // Node parameters
  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image."
    "Can be any supported by cv_bridge given the input pixel format";
  output_encoding_ = declare_parameter(
    "output_encoding", std::string{"rgb8"},
    output_encoding_description);

  // Camera info parameters
  auto camera_info_url_description = rcl_interfaces::msg::ParameterDescriptor();
  camera_info_url_description.description = "The location for getting camera calibration data";
  camera_info_url_description.read_only = true;
  auto camera_info_url = declare_parameter<std::string>(
    "camera_info_url", "",
    camera_info_url_description);
  if (camera_info_url != "") {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  auto camera_frame_id_description = rcl_interfaces::msg::ParameterDescriptor{};
  camera_frame_id_description.description = "Frame id inserted in published image";
  camera_frame_id_description.read_only = true;
  camera_frame_id_ = declare_parameter<std::string>(
    "camera_frame_id", "camera",
    camera_frame_id_description);

  // Format parameters
  // Pixel format
  auto const & image_formats = camera_->getImageFormats();
  auto pixel_format_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  pixel_format_descriptor.name = "pixel_format";
  pixel_format_descriptor.description = "Pixel format (FourCC)";
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  pixel_format_descriptor.additional_constraints = str;
  auto pixel_format =
    declare_parameter<std::string>("pixel_format", "YUYV", pixel_format_descriptor);
  requestPixelFormat(pixel_format);

  // Image size
  using ImageSize = std::vector<int64_t>;
  auto image_size = ImageSize{};
  auto image_size_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  image_size_descriptor.name = "image_size";
  image_size_descriptor.description = "Image width & height";

  // List available image sizes per format
  auto const & image_sizes = camera_->getImageSizes();
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  for (auto const & format : image_formats) {
    image_sizes_constraints << "\n" << FourCC::toString(format.pixelFormat) << " (" <<
      format.description << ")";

    auto iter = image_sizes.find(format.pixelFormat);
    if (iter == image_sizes.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "No sizes available to create parameter description for format: " << format.description);
      continue;
    }

    auto size_type = iter->second.first;
    auto & sizes = iter->second.second;
    switch (size_type) {
      case V4l2CameraDevice<Topic>::ImageSizeType::DISCRETE:
        for (auto const & image_size : sizes) {
          image_sizes_constraints << "\n\t" << image_size.first << "x" << image_size.second;
        }
        break;
      case V4l2CameraDevice<Topic>::ImageSizeType::STEPWISE:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        image_sizes_constraints << "\n\tstep:\t" << sizes[2].first << "x" << sizes[2].second;
        break;
      case V4l2CameraDevice<Topic>::ImageSizeType::CONTINUOUS:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        break;
    }
  }

  image_size_descriptor.additional_constraints = image_sizes_constraints.str();
  image_size = declare_parameter<ImageSize>("image_size", {640, 480}, image_size_descriptor);
  requestImageSize(image_size);

  // Control parameters
  auto toParamName =
    [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

  for (auto const & c : camera_->getControls()) {
    auto name = toParamName(c.name);
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.name = name;
    descriptor.description = c.name;
    switch (c.type) {
      case ControlType::INT:
        {
          auto current_value = camera_->getControlValue(c.id);
          auto range = rcl_interfaces::msg::IntegerRange{};
          range.from_value = c.minimum;
          range.to_value = c.maximum;
          descriptor.integer_range.push_back(range);
          auto value = declare_parameter<int64_t>(name, current_value, descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::BOOL:
        {
          auto value =
            declare_parameter<bool>(name, camera_->getControlValue(c.id) != 0, descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::MENU:
        {
          auto sstr = std::ostringstream{};
          for (auto const & o : c.menuItems) {
            sstr << o.first << " - " << o.second << ", ";
          }
          auto str = sstr.str();
          descriptor.additional_constraints = str.substr(0, str.size() - 2);
          auto value = declare_parameter<int64_t>(name, camera_->getControlValue(c.id), descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control type not currently supported: %s, for control: %s",
          std::to_string(unsigned(c.type)).c_str(),
          c.name.c_str());
        continue;
    }
    control_name_to_id_[name] = c.id;
  }

  // Register callback for parameter value setting
  on_set_parameters_callback_ = add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        result.successful &= handleParameter(p);
      }
      return result;
    });
}

template<typename Topic>
bool V4L2Camera<Topic>::handleParameter(rclcpp::Parameter const & param)
{
  auto name = std::string{param.get_name()};
  if (control_name_to_id_.find(name) != control_name_to_id_.end()) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return camera_->setControlValue(control_name_to_id_[name], param.as_bool());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return camera_->setControlValue(control_name_to_id_[name], param.as_int());
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %s, for parameter: %s",
          std::to_string(unsigned(param.get_type())).c_str(), param.get_name().c_str());
    }
  } else if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "pixel_format") {
    camera_->stop();
    auto success = requestPixelFormat(param.as_string());
    camera_->start();
    return success;
  } else if (param.get_name() == "image_size") {
    camera_->stop();
    auto success = requestImageSize(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "camera_info_url") {
    auto camera_info_url = param.as_string();
    if (cinfo_->validateURL(camera_info_url)) {
      return cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      return false;
    }
  }

  return false;
}

template<typename Topic>
bool V4L2Camera<Topic>::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Invalid pixel format size: must be a 4 character code (FOURCC).");
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

template<typename Topic>
bool V4L2Camera<Topic>::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid image size; expected dimensions: 2, actual: %lu",
      size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}

template<typename Topic>
sensor_msgs::msg::Image::UniquePtr V4L2Camera<Topic>::convert(sensor_msgs::msg::Image const & img) const
{
  auto tracked_object = std::shared_ptr<const void>{};
  auto cvImg = cv_bridge::toCvShare(img, tracked_object);
  auto outImg = std::make_unique<sensor_msgs::msg::Image>();
  auto cvConvertedImg = cv_bridge::cvtColor(cvImg, output_encoding_);
  cvConvertedImg->toImageMsg(*outImg);
  return outImg;
}

template<typename Topic>
void V4L2Camera<Topic>::convert_shm(Topic & img) const
{
  auto tracked_object = std::shared_ptr<const void>{};
  auto cvImg = shm_msgs::toCvShare(img, tracked_object);
  auto cvConvertedImg = shm_msgs::cvtColor(cvImg, output_encoding_);
  cvConvertedImg->toImageMsg(img);
}

template<typename Topic>
bool V4L2Camera<Topic>::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

template<typename Topic>
bool V4L2Camera<Topic>::checkCameraInfo_shm(
  Topic const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

}  // namespace v4l2_camera

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
