// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  m_camera(new usb_cam::UsbCam()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_compressed_img_msg(nullptr),
  m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
      rclcpp::QoS {100}.get_rmw_qos_profile()))),
  m_compressed_image_publisher(nullptr),
  m_compressed_cam_info_publisher(nullptr),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
  m_service_capture(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3))),
  m_enable_undistortion(false),
  m_undistorted_image_publisher(nullptr),
  m_undistorted_image_msg(nullptr),
  m_undistorted_camera_info_msg(nullptr),
  m_undistort_maps_initialized(false)
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("skip_device_check", false);  // allow bypassing V4L2 device list check
  this->declare_parameter("enable_undistortion", false);  // enable fisheye undistortion
  this->declare_parameter(
      "flip_180", false); // rotate published image 180 deg (upside-down mount)

  get_params();
  init();
  m_parameters_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &UsbCamNode::parameters_callback, this,
      std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_publish_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    m_camera->start_capturing();
    response->message = "Start Capturing";
  } else {
    m_camera->stop_capturing();
    response->message = "Stop Capturing";
  }
}

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);

    // if the target path is relative, resolve it
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }

    return target_path.string();
  }
  return path;
}

void UsbCamNode::init()
{
  while (m_parameters.frame_id == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Derive base and optical frame ids from camera_name, e.g. "fisheye_camera"
  // -> "fisheye_camera_base_frame" / "back_camera_color_optical_frame"
  m_base_frame_id = m_parameters.camera_name + "_base_frame";
  m_optical_frame_id = m_parameters.camera_name + "_color_optical_frame";

  // load the camera info
  m_camera_info.reset(
    new camera_info_manager::CameraInfoManager(
      this, m_parameters.camera_name, m_parameters.camera_info_url));
  // check for default camera info
  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_optical_frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  // Check if given device name is an available v4l2 device (unless skipped)
  if (!m_parameters.skip_device_check) {
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Device specified is not available or is not a vaild V4L2 device: `" <<
          m_parameters.device_name << "`"
      );
      RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
      for (const auto & device : available_devices) {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
        RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
      }
      rclcpp::shutdown();
      return;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Skipping V4L2 device availability check by request.");
  }

  // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
  // and publisher
  if (m_parameters.pixel_format_name == "mjpeg") {
    m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
    m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
    m_compressed_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
      std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
    m_compressed_cam_info_publisher =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(100));
  }

  // Initialize undistortion publisher if enabled
  if (m_enable_undistortion) {
    m_undistorted_image_msg.reset(new sensor_msgs::msg::Image());
    m_undistorted_image_msg->header.frame_id = m_optical_frame_id;
    m_undistorted_camera_info_msg.reset(new sensor_msgs::msg::CameraInfo());
    std::string preview_topic = m_parameters.camera_name + "/preview/image_raw";
    m_undistorted_image_publisher = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, preview_topic,
      rclcpp::QoS {100}.get_rmw_qos_profile()));
    RCLCPP_INFO(this->get_logger(), "Fisheye undistortion enabled - will publish on '%s/preview/image_raw' and '%s/preview/camera_info' topics", 
      m_parameters.camera_name.c_str(), m_parameters.camera_name.c_str());
  }

  // Publish static transform: base_frame → optical_frame
  // base_frame: x-forward, y-left, z-up
  // optical_frame: x-right, y-down, z-forward
  // Rotation quaternion (x, y, z, w) = (-0.5, 0.5, -0.5, 0.5)

  m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = m_base_frame_id;
    tf_msg.child_frame_id = m_optical_frame_id;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    // Rotation from body (x-fwd, y-left, z-up) to optical (x-right, y-down, z-fwd)
    tf_msg.transform.rotation.x = -0.5;
    tf_msg.transform.rotation.y =  0.5;
    tf_msg.transform.rotation.z = -0.5;
    tf_msg.transform.rotation.w =  0.5;
    m_static_tf_broadcaster->sendTransform(tf_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing static TF: '%s' -> '%s'",
                m_base_frame_id.c_str(), m_optical_frame_id.c_str());
  }

  m_image_msg->header.frame_id = m_optical_frame_id;
  RCLCPP_INFO(
    this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
    m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
    m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
  // set the IO method
  io_method_t io_method =
    usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  // configure the camera
  m_camera->configure(m_parameters, io_method);

  set_v4l2_params();

  // start the camera
  m_camera->start();

  auto frame_rate = m_camera->get_frame_rate();
  if (static_cast<size_t>(m_parameters.framerate) > frame_rate) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Desired framerate " << m_parameters.framerate << " is higher than the camera's capability " <<
        frame_rate << " fps");
    m_parameters.framerate = frame_rate;
  }

  // TODO(lucasw) should this check a little faster than expected frame rate?
  // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / frame_rate;
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
  const int publish_period_ms = 1000.0 / m_parameters.framerate;
  m_publish_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(publish_period_ms)),
    std::bind(&UsbCamNode::publish, this));
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
      {"camera_name",        "camera_info_url",     "frame_id",
       "framerate",          "image_height",        "image_width",
       "io_method",          "pixel_format",        "av_device_format",
       "video_device",       "brightness",          "contrast",
       "saturation",         "sharpness",           "gain",
       "auto_white_balance", "white_balance",       "autoexposure",
       "exposure",           "autofocus",           "focus",
       "skip_device_check",  "enable_undistortion", "flip_180"});

  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      m_parameters.pixel_format_name = parameter.value_to_string();
    } else if (parameter.get_name() == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    } else if (parameter.get_name() == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (parameter.get_name() == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (parameter.get_name() == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (parameter.get_name() == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (parameter.get_name() == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (parameter.get_name() == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (parameter.get_name() == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (parameter.get_name() == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (parameter.get_name() == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (parameter.get_name() == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (parameter.get_name() == "focus") {
      m_parameters.focus = parameter.as_int();
    } else if (parameter.get_name() == "skip_device_check") {
      m_parameters.skip_device_check = parameter.as_bool();
    } else if (parameter.get_name() == "enable_undistortion") {
      m_enable_undistortion = parameter.as_bool();
    } else if (parameter.get_name() == "flip_180") {
      m_parameters.flip_180 = parameter.as_bool();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void UsbCamNode::set_v4l2_params()
{
  // set camera parameters
  if (m_parameters.brightness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
  }

  if (m_parameters.contrast >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
  }

  if (m_parameters.saturation >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
  }

  if (m_parameters.sharpness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
  }

  if (m_parameters.gain >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
  }

  // check auto white balance
  if (m_parameters.auto_white_balance) {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("exposure_auto", 3);
  }

  // check auto focus
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_auto", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_auto", 0);
    if (m_parameters.focus >= 0) {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

bool UsbCamNode::take_and_send_image()
{
  // Only resize if required
  if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      // Some formats don't have a linesize specified by v4l2
      // Fall back to manually calculating it step = size / height
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));

  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;

  if (m_parameters.flip_180) {
    flip_image_180(*m_image_msg);
    flip_camera_info_180(*m_camera_info_msg);
  }
  return true;
}

/// @brief Rotate a raw image message 180 degrees in place, for an upside-down
/// camera mount. 4:2:2 packed formats (yuyv/uyvy) are rotated at
/// 2-pixel-macropixel granularity so the Y/U/Y/V byte order within each
/// macropixel is preserved -- only row order and macropixel order within a row
/// are reversed, which is exactly what a 180-degree rotation requires.
void UsbCamNode::flip_image_180(sensor_msgs::msg::Image &img) {
  int type;
  int cols = img.width;
  if (img.encoding == "mono8") {
    type = CV_8UC1;
  } else if (img.encoding == "rgb8" || img.encoding == "bgr8") {
    type = CV_8UC3;
  } else if (img.encoding == "yuyv" || img.encoding == "yuv422" ||
             img.encoding == "uyvy") {
    type = CV_8UC4;
    cols = img.width / 2;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "flip_180 requested but encoding '%s' is not "
                         "supported for in-driver rotation",
                         img.encoding.c_str());
    return;
  }

  cv::Mat src(img.height, cols, type, img.data.data(), img.step);
  std::vector<uint8_t> flipped(img.data.size());
  cv::Mat dst(img.height, cols, type, flipped.data(), img.step);
  cv::flip(src, dst, -1); // flip across both axes == 180 degree rotation
  img.data.swap(flipped);
}

/// @brief Adjust the principal point to match a 180-degree-rotated image so
/// downstream consumers (rectification, the undistortion path below) stay
/// geometrically correct. Focal lengths and distortion coefficients are
/// unaffected by a 180-degree rotation.
void UsbCamNode::flip_camera_info_180(sensor_msgs::msg::CameraInfo &info) {
  if (info.k[0] == 0.0) {
    return; // uncalibrated, nothing meaningful to adjust
  }
  info.k[2] = info.width - info.k[2];
  info.k[5] = info.height - info.k[5];
  if (info.p.size() == 12) {
    info.p[2] = info.width - info.p[2];
    info.p[6] = info.height - info.p[6];
  }
}

bool UsbCamNode::take_and_send_image_mjpeg()
{
  // Only resize if required
  if (sizeof(m_compressed_img_msg->data) != m_camera->get_image_size_in_bytes()) {
    m_compressed_img_msg->format = "jpeg";
    m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  m_camera->get_image(reinterpret_cast<char *>(&m_compressed_img_msg->data[0]));

  auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;

  if (m_parameters.flip_180) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "flip_180 is not supported for raw mjpeg passthrough "
                         "(pixel_format 'mjpeg'); "
                         "use 'mjpeg2rgb' (or another raw pixel format) to get "
                         "an in-driver corrected feed.");
  }

  return true;
}

rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
  m_timer->reset();
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::update()
{
  if (m_camera->is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    bool isSuccessful = (m_parameters.pixel_format_name == "mjpeg") ?
      take_and_send_image_mjpeg() :
      take_and_send_image();
    if (!isSuccessful) {
      RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
    }
  }
}

void UsbCamNode::publish()
{
  if (m_parameters.pixel_format_name == "mjpeg") {
    m_compressed_image_publisher->publish(*m_compressed_img_msg);
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  } else {
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
    
    // Publish undistorted image if enabled
    if (m_enable_undistortion) {
      undistort_image();
    }
  }
}

void UsbCamNode::init_undistortion_maps()
{
  if (m_undistort_maps_initialized) {
    return;
  }

  // Get camera info
  auto cam_info = m_camera_info->getCameraInfo();
  if (m_parameters.flip_180) {
    // undistort_image() operates on m_image_msg, which is already rotated by
    // this point, so the maps must be built against the rotated principal
    // point.
    flip_camera_info_180(cam_info);
  }

  // Check if we have valid calibration
  if (cam_info.k[0] == 0.0 || cam_info.d.size() < 4) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No valid camera calibration found. Undistortion disabled. "
      "Please provide camera_info_url parameter with calibration file.");
    return;
  }

  // Extract camera matrix
  m_camera_matrix = (cv::Mat_<double>(3, 3) <<
    cam_info.k[0], cam_info.k[1], cam_info.k[2],
    cam_info.k[3], cam_info.k[4], cam_info.k[5],
    cam_info.k[6], cam_info.k[7], cam_info.k[8]);

  // Extract distortion coefficients
  m_distortion_coeffs = cv::Mat(cam_info.d.size(), 1, CV_64F);
  for (size_t i = 0; i < cam_info.d.size(); ++i) {
    m_distortion_coeffs.at<double>(i, 0) = cam_info.d[i];
  }

  cv::Size image_size(m_parameters.image_width, m_parameters.image_height);

  // Check distortion model
  if (cam_info.distortion_model == "equidistant" || cam_info.distortion_model == "fisheye") {
    // Fisheye model
    RCLCPP_INFO(this->get_logger(), "Initializing fisheye undistortion maps");
    cv::Mat new_camera_matrix = m_camera_matrix.clone();
    
    try {
      cv::fisheye::initUndistortRectifyMap(
        m_camera_matrix,
        m_distortion_coeffs,
        cv::Mat(),
        new_camera_matrix,
        image_size,
        CV_32FC1,
        m_undistort_map1,
        m_undistort_map2);
      
      m_undistort_maps_initialized = true;
      RCLCPP_INFO(this->get_logger(), "Fisheye undistortion maps initialized successfully");
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize fisheye undistortion maps: %s", e.what());
      return;
    }
  } else {
    // Standard pinhole model
    RCLCPP_INFO(this->get_logger(), "Initializing standard undistortion maps");
    cv::Mat new_camera_matrix = m_camera_matrix.clone();
    
    try {
      cv::initUndistortRectifyMap(
        m_camera_matrix,
        m_distortion_coeffs,
        cv::Mat(),
        new_camera_matrix,
        image_size,
        CV_32FC1,
        m_undistort_map1,
        m_undistort_map2);
      
      m_undistort_maps_initialized = true;
      RCLCPP_INFO(this->get_logger(), "Standard undistortion maps initialized successfully");
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize undistortion maps: %s", e.what());
      return;
    }
  }

  // Setup undistorted camera info (identity rectification, no distortion)
  *m_undistorted_camera_info_msg = cam_info;
  m_undistorted_camera_info_msg->distortion_model = "plumb_bob";
  m_undistorted_camera_info_msg->d = {0.0, 0.0, 0.0, 0.0, 0.0};
}

void UsbCamNode::undistort_image()
{
  // Initialize maps on first call
  if (!m_undistort_maps_initialized) {
    init_undistortion_maps();
    if (!m_undistort_maps_initialized) {
      return;  // Initialization failed
    }
  }

  try {
    // Convert ROS image to OpenCV Mat
    cv::Mat src_image;
    std::string out_encoding;
    if (m_image_msg->encoding == "bgr8") {
      src_image = cv::Mat(m_image_msg->height, m_image_msg->width, CV_8UC3,
                          const_cast<uint8_t*>(m_image_msg->data.data()), m_image_msg->step);
      out_encoding = "bgr8";
    } else if (m_image_msg->encoding == "rgb8") {
      // remap is channel-order agnostic, so no color conversion needed
      src_image = cv::Mat(m_image_msg->height, m_image_msg->width, CV_8UC3,
                          const_cast<uint8_t*>(m_image_msg->data.data()), m_image_msg->step);
      out_encoding = "rgb8";
    } else if (m_image_msg->encoding == "mono8") {
      src_image = cv::Mat(m_image_msg->height, m_image_msg->width, CV_8UC1,
                          const_cast<uint8_t*>(m_image_msg->data.data()), m_image_msg->step);
      out_encoding = "mono8";
    } else if (m_image_msg->encoding == "yuv422" || m_image_msg->encoding == "yuyv") {
      // Convert YUYV to BGR
      cv::Mat yuyv_image(m_image_msg->height, m_image_msg->width, CV_8UC2,
                         const_cast<uint8_t*>(m_image_msg->data.data()), m_image_msg->step);
      cv::cvtColor(yuyv_image, src_image, cv::COLOR_YUV2BGR_YUYV);
      out_encoding = "bgr8";
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unsupported image encoding for undistortion: %s", m_image_msg->encoding.c_str());
      return;
    }

    // Apply undistortion
    cv::Mat dst_image;
    cv::remap(src_image, dst_image, m_undistort_map1, m_undistort_map2, cv::INTER_LINEAR);

    // Convert back to ROS message
    m_undistorted_image_msg->header = m_image_msg->header;
    m_undistorted_image_msg->height = dst_image.rows;
    m_undistorted_image_msg->width = dst_image.cols;
    m_undistorted_image_msg->encoding = out_encoding;
    m_undistorted_image_msg->step = dst_image.step;
    m_undistorted_image_msg->is_bigendian = false;
    
    size_t size = dst_image.step * dst_image.rows;
    m_undistorted_image_msg->data.resize(size);
    memcpy(&m_undistorted_image_msg->data[0], dst_image.data, size);

    // Update and publish undistorted camera info
    m_undistorted_camera_info_msg->header = m_image_msg->header;
    
    // Publish undistorted image with camera info
    m_undistorted_image_publisher->publish(*m_undistorted_image_msg, *m_undistorted_camera_info_msg);
    
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Error during image undistortion: %s", e.what());
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
