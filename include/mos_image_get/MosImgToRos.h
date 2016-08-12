#ifndef MOS_IMG_TO_ROS_H
#define MOS_IMG_TO_ROS_H

#include <string>
#include <sstream>

#include <sensor_msgs/Image.h>

namespace usb_cam {

class UsbCam {
 public:

  UsbCam();
  ~UsbCam();

  // grabs a new image from the camera
  void grab_image(sensor_msgs::Image* image);

 private:
  typedef struct
  {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char *image;
    int is_new;
  } camera_image_t;

  struct buffer
  {
    void * start;
    size_t length;
  };


  void process_image(const void * src, int len, camera_image_t *dest);
  void grab_image();
  bool is_capturing_;


  buffer * buffers_;
  unsigned int n_buffers_;
  camera_image_t *image_;

};

}

#endif

