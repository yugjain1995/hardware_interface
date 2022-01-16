#include <iostream>
#include <cstdlib>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/log/trivial.hpp>
#include <chrono>
/// [headers]

int main(int argc, char **argv)
{
/// [initialize ROS node]
  ros::init(argc, argv, "kinect_image_publisher");
  ros::NodeHandle n;
///[initialize ROS node]

/// [initialize image publisher]
  image_transport::ImageTransport it(n);
  image_transport::Publisher image_pub = it.advertise("camera/image", 1);
/// [initialize image publisher]

/// [initialize freenect2 library variables]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
/// [initialize freenect2 library variables]

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    BOOST_LOG_TRIVIAL(error) << "no device connected!" << std::endl;
    return -1;
  }	
  std::string serial = freenect2.getDefaultDeviceSerialNumber();
/// [discovery]

/// [open]
  dev = freenect2.openDevice(serial);
/// [open]

if(dev == 0)
  {
    BOOST_LOG_TRIVIAL(error) << "failure opening device!" << std::endl;
    return -1;
  }

/// [listeners]
  int types = 0;
  types |= libfreenect2::Frame::Color;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
/// [listeners]

/// [start]
  if (!dev->startStreams(true, false))
    {
      BOOST_LOG_TRIVIAL(error) << "faliure to start stream to listner";
      return -1;
    }
  BOOST_LOG_TRIVIAL(info) << "device - Kinect v2\n";
  BOOST_LOG_TRIVIAL(info) << "device serial: " << dev->getSerialNumber() << std::endl;
  BOOST_LOG_TRIVIAL(info) << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]

  ros::Rate loop_rate(10);
  while(n.ok())
  {
    auto start = std::chrono::high_resolution_clock::now();
    if (!listener.waitForNewFrame(frames, 0.06*1000)) // 0.06 seconds
    {
      BOOST_LOG_TRIVIAL(warning) << "Timeout!" << std::endl;
      ros::spinOnce();
      continue;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    
  /// [Convert RGB from freenect to OpenCV cv::Mat image]
    cv::Mat rgb_frame = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
  /// [Convert RGB from freenect to OpenCV cv::Mat image]

  /// [Convert RGB in cv::Mat to grey scale]
    cv::Mat grey_frame;
    cv::cvtColor(rgb_frame, grey_frame, cv::COLOR_BGRA2GRAY);
  /// [Convert RGB in cv::Mat to grey scale]

  /// [Convert cv::Mat to ROS image message and publish the image message]
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grey_frame).toImageMsg();
    image_pub.publish(msg);
    cv::waitKey(1); // Why do we use this delay/wait?
    BOOST_LOG_TRIVIAL(info) << "Published image frame.\n";
  /// [Convert cv::Mat to ROS image message]

    listener.release(frames);
    BOOST_LOG_TRIVIAL(info) << "Released image frame.\n";

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    auto frame_rate = 1000/(float)duration.count();
    BOOST_LOG_TRIVIAL(info) << "Frame rate = " << frame_rate << "\n";

    ros::spinOnce();
    loop_rate.sleep();
  }

  dev->stop();
  dev->close();
  return 0;
}
