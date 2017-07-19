#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

boost::shared_ptr<ros::Publisher> pub_ptr;

void callback_rgb(const sensor_msgs::Image::ConstPtr &input) {
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

void callback_depth(const sensor_msgs::Image::ConstPtr &input) {
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

    // Compressed image data
    std::vector <uint8_t> compressedImage;

    // Update ros message format header
    compressed.format += "; depth compressed";


    // Raw depth map compression
    if ((bitDepth == 16) && (numChannels == 1)) {
        params[0] = 16; // this is CV_IMWRITE_PNG_COMPRESSION, but I cant find the namespace
        params[1] = 9;


        // OpenCV-ROS bridge
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*input);
        }
        catch (cv::Exception &e) {
            ROS_ERROR("%s", e.msg.c_str());
        }

        const cv::Mat &depthImg = cv_ptr->image;
        size_t rows = depthImg.rows;
        size_t cols = depthImg.cols;

        if ((rows > 0) && (cols > 0)) {
            unsigned short depthMaxUShort = static_cast<unsigned short>(10 * 1000.0f);

            // Matrix iterators
            cv::MatIterator_ < unsigned short > itDepthImg = cv_ptr->image.begin<unsigned short>(),
                    itDepthImg_end = cv_ptr->image.end< unsigned short>();

            // Max depth filter
            for (; itDepthImg != itDepthImg_end; ++itDepthImg) {
                if (*itDepthImg > depthMaxUShort)
                    *itDepthImg = 0;
            }

            // Compress raw depth image
            if (cv::imencode(".png", cv_ptr->image, compressedImage, params)) {
                float cRatio = (float) (cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                               / (float) compressedImage.size();
                ROS_DEBUG("Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)", cRatio,
                          compressedImage.size());
            } else {
                ROS_ERROR("cv::imencode (png) failed on input image");
            }
        }
    } else {
        ROS_ERROR(
                "Compressed Depth Image Transport - Compression requires 16bit raw depth images (input format is: %s).",
                input->encoding.c_str());

    }

    if (compressedImage.size() > 0)
    {
        ROS_DEBUG("Copying data");

        for(int i = 0; i < compressedImage.size(); i++){
            compressed.data.push_back(i);
        }

    } else {
        ROS_ERROR("Got empty Image!");
    }

    pub_ptr.get()->publish(compressed);
}

int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "head_depth_compressor");

    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/pepper_robot/camera/depth/camera/image_raw", 10, &callback_depth);

    pub_ptr.reset(new ros::Publisher(nh.advertise<sensor_msgs::CompressedImage>("/pepper_robot/camera/depth/camera/image_raw/compressed", 10)));

    while(ros::ok()){
        ros::Duration(0.06).sleep();
        ros::spinOnce();
    }

    return 0;
}


