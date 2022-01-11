#include <iostream>
#include <cstdlib>
#include <chrono>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include "viewer.h"
/// [headers]

int get_rgb_frames()
{
/// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
/// [context]

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  std::string serial = freenect2.getDefaultDeviceSerialNumber();
/// [discovery]

/// [open]
  dev = freenect2.openDevice(serial);
/// [open]

if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
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
      std::cout << "faliure to start stream to listner";
      return -1;
    }
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]

/// Initialize visualizer for image stream
Viewer viewer;
viewer.initialize();

int framecount = 0;
int frameMax = 100;
auto start = std::chrono::high_resolution_clock::now();
/// [loop start]
  while(framecount < frameMax)
  {
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
/// [loop start]

    framecount++;

    /// Update viewer frame with latest image
    viewer.addFrame("RGB", rgb);
    viewer.render();

    /// [loop end]
    listener.release(frames);
  }
/// [loop end]
auto stop = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
float frame_rate = (float)frameMax/(float)duration.count();
std::cout << "Frame rate = " << frame_rate << "\n";
  dev->stop();
  dev->close();
  return 0;
}
