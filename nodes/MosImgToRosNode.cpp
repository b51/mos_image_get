#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <opencv/highgui.h>

#include <vector>
#include <mos_image_get/MosImgToRos.h>
#include <mos_image_get/MosImgToRosNode.h>

namespace usb_cam 
{
    UsbCamNode::UsbCamNode() :
    node_("~")
    {
        image_transport::ImageTransport it(node_);
//        image_transport::Publisher image_pub_ = it.advertise("image_raw", 1);
        image_pub_ = it.advertise("/image_raw", 1);
//        publish_fn = node_.advertise<sensor_msgs::CompressedImage>("compressed_image", 1);
        
    }
    
    //bool UsbCamNode::imageCompress(const sensor_msgs::Image& message)
    //{
    //    sensor_msgs::CompressedImage compressed;
    //    compressed.header = message.header;
    //    compressed.format = message.encoding;

    //    std::vector<int> params;
    //    params.resize(3, 0);

    //    int bitDepth = enc::bitDepth(message.encoding);
    //    int numChannels = enc::numChannels(message.encoding);

    //    params[0] = CV_IMWRITE_JPEG_QUALITY;
    //    params[1] = 95;

    //    // Update ros message format header
    //    compressed.format += "; jpeg compressed";

    //    // Check input format
    //    if ((bitDepth == 8) && // JPEG only works on 8bit images
    //        ((numChannels == 1) || (numChannels == 3)))
    //    {

    //      // Target image format
    //      stringstream targetFormat;
    //      if (enc::isColor(message.encoding))
    //      {
    //        // convert color images to RGB domain
    //        targetFormat << "rgb" << bitDepth;
    //      }

    //      // OpenCV-ros bridge
    //      cv_bridge::CvImagePtr cv_ptr;
    //      try
    //      {
    //        cv_ptr = cv_bridge::toCvCopy(message, targetFormat.str());

    //        // Compress image
    //        if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
    //        {

    //          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
    //              / (float)compressed.data.size();
    //          ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
    //        }
    //        else
    //        {
    //          ROS_ERROR("cv::imencode (jpeg) failed on input image");
    //        }
    //      }
    //      catch (cv_bridge::Exception& e)
    //      {
    //        ROS_ERROR("%s", e.what());
    //      }
    //      catch (cv::Exception& e)
    //      {
    //        ROS_ERROR("%s", e.what());
    //      }

    //      // Publish message
    //      publish_fn.publish(compressed);
    //      return true;
    //    }
    //}
    
    bool UsbCamNode::take_and_send_image()
    {
        cam_.grab_image(&img_);
   //     imageCompress(img_);
    
        image_pub_.publish(img_);
        printf("start\n");
    
        return true;
    }

    bool UsbCamNode::spin()
    {
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok())
        {
            printf("start\n");
            if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
            ros::spinOnce();
            sleep(1);
    
        }
        return true;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mos_image_get");
    usb_cam::UsbCamNode a;
    a.spin();
    return EXIT_SUCCESS;
}
