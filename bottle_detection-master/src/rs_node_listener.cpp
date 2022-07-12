#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/rsutil.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tf/transform_listener.h>


// this Node provide service convert 2d pixel to point in 3d space
class Deprojector
{
protected:
  ros::NodeHandle m_nh;
  sensor_msgs::CameraInfo* m_intrinsics_ptr;
  sensor_msgs::CameraInfo m_camera_info;
  sensor_msgs::Image* m_depth_ptr;
  sensor_msgs::Image m_depth;
  darknet_ros_msgs::BoundingBoxes* m_bbx_ptr;
  darknet_ros_msgs::BoundingBoxes m_bbx;
  geometry_msgs::PointStamped m_point;
  bool depth_received = false;
  struct rs2_intrinsics m_rs_intrinsic;
  float depth_at_left_pixel;
  float depth_at_right_pixel;
  float point_left[3];
  float point_right[3];
  float point_aver[3];
  float pixel_left[2];
  float pixel_right[2];
  bool bottle_detected= false;
   


public:
  ros::Subscriber m_depth_sub, m_bbx_sub, m_intrinsic;
  ros::Publisher m_points_pub;
  ros::ServiceClient m_detect_trigger;
  tf::TransformListener* m_tf_listener = NULL;

Deprojector(void)
{
  //Initialize tf_listener for pointcloud transformation
  m_tf_listener = new(tf::TransformListener);
  m_tf_listener->waitForTransform("/base_link", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(5.0));
  ROS_INFO("Transform to camera found.");

  m_bbx_sub = m_nh.subscribe("/darknet_ros/bounding_boxes", 1, &Deprojector::getBBX, this);
  m_intrinsic = m_nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &Deprojector::getIntrinsics, this);
  m_depth_sub = m_nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Deprojector::getDepth, this);
  m_points_pub = m_nh.advertise<geometry_msgs::PointStamped>("point_in_3d", 1000);
  m_detect_trigger = m_nh.serviceClient<std_srvs::Trigger>("/bottle_detection/detectedTrigger");
}
~Deprojector(void)
{ }

// get depth and do the deprojection
void getDepth(const sensor_msgs::Image::ConstPtr& msg)
{
  if(bottle_detected==true){
  cv_bridge::CvImagePtr cv_ptr;
  m_depth = *msg;
  cv_ptr = cv_bridge::toCvCopy(msg);
  ROS_INFO("pixel position: %d, %d", (int)pixel_left[0], (int)pixel_left[1]);
  depth_at_left_pixel = cv_ptr->image.at<uint16_t>(cv::Point((int)pixel_left[0], (int)pixel_left[1]));
  depth_at_right_pixel = cv_ptr->image.at<uint16_t>(cv::Point((int)pixel_right[0], (int)pixel_right[1]));
  ROS_INFO("depth is %f", depth_at_left_pixel);
  rs2_deproject_pixel_to_point(point_left, &m_rs_intrinsic, pixel_left, depth_at_left_pixel);
  rs2_deproject_pixel_to_point(point_right, &m_rs_intrinsic, pixel_right, depth_at_right_pixel); 
  point_aver[0] = (point_left[0]+point_right[0])/2.0;
  point_aver[1] = (point_left[1]+point_right[1])/2.0;
  point_aver[2] = (point_left[2]+point_right[2])/2.0;
  ROS_INFO("point in 3d space: %f, %f, %f", point_aver[0], point_aver[1], point_aver[2]);
  m_point.header.stamp = ros::Time::now();
  m_point.header.frame_id = "/camera_depth_optical_frame";
  m_point.point.x = point_aver[0]/1000.0;
  m_point.point.y = point_aver[1]/1000.0;
  m_point.point.z = point_aver[2]/1000.0;

  m_tf_listener->waitForTransform("/base_link", "/camera_depth_optical_frame",m_point.header.stamp, ros::Duration(3.0));
  m_tf_listener->transformPoint("/base_link", m_point, m_point);
  m_points_pub.publish(m_point);
  bottle_detected = false;
  }
}

// get bounding box of the first bottle appear in scene
void getBBX(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  m_bbx = *msg;
  for (int i = 0; i < m_bbx.bounding_boxes.size(); i++)
  {
    if(m_bbx.bounding_boxes[i].Class=="bottle"){
      pixel_right[0] = (float)m_bbx.bounding_boxes[i].xmax;
      pixel_right[1] = (float)m_bbx.bounding_boxes[i].ymax;
      pixel_left[0] = (float)m_bbx.bounding_boxes[i].xmin;
      pixel_left[1] = (float)m_bbx.bounding_boxes[i].ymax;
      bottle_detected = true;
      // prevent the situation that only half bottle is detected
      if((pixel_left[0]+pixel_right[0])/2<=500 && (pixel_left[1]+pixel_right[1])/2<=400){
      std_srvs::Trigger srv;
      m_detect_trigger.call(srv);
      }
      break;
    }
  }
}

// get camera intrinsics fot the deprojection
void getIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  m_camera_info = *msg;
  m_rs_intrinsic.width = m_camera_info.width;
  m_rs_intrinsic.height = m_camera_info.height;
  m_rs_intrinsic.ppx = m_camera_info.K[2];
  m_rs_intrinsic.ppy = m_camera_info.K[5];
  m_rs_intrinsic.fx = m_camera_info.K[0];
  m_rs_intrinsic.fy = m_camera_info.K[4];
  m_rs_intrinsic.model = RS2_DISTORTION_NONE;
}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_node_listener");
  ros::NodeHandle nh("~");
  Deprojector m_deprojector;

  ros::spin();
  return 0;
}