// Copyright 2021 Evan Flynn
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
//    * Neither the name of the Evan Flynn nor the names of its
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


#ifndef USB_CAM__USB_CAM_NODE_HPP_
#define USB_CAM__USB_CAM_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "usb_cam/usb_cam.hpp"


std::ostream & operator<<(std::ostream & ostr, const rclcpp::Time & tm)
{
  ostr << tm.nanoseconds();
  return ostr;
}


namespace usb_cam
{

class UsbCamNode : public rclcpp::Node
{
public:
  explicit UsbCamNode(const rclcpp::NodeOptions & node_options);
  ~UsbCamNode();

  void init();
  void get_params();
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  void set_v4l2_params();
  void update();
  void publish();
  bool take_and_send_image();
  bool take_and_send_image_mjpeg();
  void flip_image_180(sensor_msgs::msg::Image &img);
  void flip_camera_info_180(sensor_msgs::msg::CameraInfo &info);

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  UsbCam * m_camera;

  sensor_msgs::msg::Image::UniquePtr m_image_msg;
  sensor_msgs::msg::CompressedImage::UniquePtr m_compressed_img_msg;
  std::shared_ptr<image_transport::CameraPublisher> m_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_compressed_cam_info_publisher;

  parameters_t m_parameters;

  sensor_msgs::msg::CameraInfo::SharedPtr m_camera_info_msg;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::TimerBase::SharedPtr m_publish_timer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_service_capture;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;

  // Static TF broadcaster for optical frame
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;
  std::string m_optical_frame_id;

  // Fisheye undistortion support
  bool m_enable_undistortion;
  std::shared_ptr<image_transport::CameraPublisher> m_undistorted_image_publisher;
  sensor_msgs::msg::Image::UniquePtr m_undistorted_image_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr m_undistorted_camera_info_msg;
  cv::Mat m_undistort_map1;
  cv::Mat m_undistort_map2;
  cv::Mat m_camera_matrix;
  cv::Mat m_distortion_coeffs;
  bool m_undistort_maps_initialized;
  void init_undistortion_maps();
  void undistort_image();
};
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
