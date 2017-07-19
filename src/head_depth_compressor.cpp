#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

boost::shared_ptr<ros::Publisher> pub_ptr;

void callback(const sensor_msgs::Image::ConstPtr &input) {
    // Compressed image message
    sensor_msgs::CompressedImage compressed;
    compressed.header = input->header;
    compressed.format = input->encoding;

    // Compression settings
    std::vector<int> params;
    params.resize(3, 0);

    // Bit depth of image encoding
    int bitDepth = enc::bitDepth(input->encoding);
    int numChannels = enc::numChannels(input->encoding);

    // Update ros message format header
    compressed.format += "; jpeg compressed";


    // Check input format
    if ((bitDepth == 8) && // JPEG only works on 8bit images
        ((numChannels == 1) || (numChannels == 3))) {

        // Target image format
        std::stringstream targetFormat;
        if (enc::isColor(input->encoding)) {
            // convert color images to RGB domain
            targetFormat << "rgb" << bitDepth;
        }

        // OpenCV-ros bridge
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*input, targetFormat.str());

            // Compress image
            if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params)) {

                float cRatio = (float) (cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                               / (float) compressed.data.size();
                ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression: 1:%.2f (%lu bytes)", cRatio,
                          compressed.data.size());
            } else {
                ROS_ERROR("cv::imencode (jpeg) failed on input image");
            }
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("%s", e.what());
        }
        catch (cv::Exception &e) {
            ROS_ERROR("%s", e.what());
        }

        // Publish message
        pub_ptr.get()->publish(compressed);
    } else
        ROS_ERROR(
                "Compressed Image Transport - JPEG compression requires 8-bit, 1/3-channel images (input format is: %s)",
                input->encoding.c_str());
}


int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "force_guiding");

    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/pepper_robot/camera/depth/camera/image_raw", 10, &callback);

    pub_ptr.reset(new ros::Publisher(nh.advertise<sensor_msgs::CompressedImage>("/pepper_robot/camera/depth/camera/image_raw/compressed", 10)));

    return 0;
}


