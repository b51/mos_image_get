#include <ros/ros.h>
#include <mos_image_get/MosImgToRos.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <std_srvs/Empty.h>

namespace usb_cam 
{
    class UsbCamNode
    {
        public:
            ros::NodeHandle node_;
            
            // shared image message
            sensor_msgs::Image img_;
            image_transport::Publisher image_pub_;
//            ros::Publisher publish_fn;
//            ros::Publisher image_pub_;
            
            // parameters
            std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
            //std::string start_service_name_, start_service_name_;
            int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
                white_balance_, gain_;
            bool autofocus_, autoexposure_, auto_white_balance_;
            
            UsbCam cam_;
            
            ros::ServiceServer service_start_, service_stop_;

            UsbCamNode();
//            bool imageCompress(const sensor_msgs::Image& message);
            bool take_and_send_image();
            bool spin();
    };
}
