/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "compressed_depth_image_transport/compressed_depth_subscriber.h"

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <limits>
#include <vector>

using namespace cv;
namespace pepper_image_sink {

    class PepperImageSink : public nodelet::Nodelet {
    public:
        PepperImageSink() {}

    private:

        ros::Publisher c_pub;
        ros::Subscriber c_sub;
        ros::Publisher d_pub;
        ros::Subscriber d_sub;
        ros::NodeHandle private_nh;
        cv_bridge::CvImagePtr c_cv_ptr;
        sensor_msgs::ImagePtr c_output;
        cv_bridge::CvImagePtr d_cv_ptr;
        sensor_msgs::ImagePtr d_output;

        virtual void onInit() {
            private_nh = getPrivateNodeHandle();
            c_pub = private_nh.advertise<sensor_msgs::Image>("out/color", 10);
            c_sub = private_nh.subscribe("in/color", 10, &PepperImageSink::color_cb, this);
            d_pub = private_nh.advertise<sensor_msgs::Image>("out/depth", 10);
            d_sub = private_nh.subscribe("in/depth", 10, &PepperImageSink::depth_cb, this);
        }

        void color_cb(const sensor_msgs::CompressedImage::ConstPtr &input) {
            try {
                c_cv_ptr = cv_bridge::toCvCopy(input);
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }


            try {
                c_output = c_cv_ptr->toImageMsg();
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge !exception: %s", e.what());
                return;
            }
            c_pub.publish(c_output);
        }

        void depth_cb(const sensor_msgs::CompressedImage::ConstPtr &message) {

            // Copy message header
            d_cv_ptr.reset(new cv_bridge::CvImage());
            d_cv_ptr->header = message->header;

            // Assign image encoding
            std::string image_encoding = message->format.substr(0, message->format.find(';'));
            d_cv_ptr->encoding = image_encoding;
            std::vector <uint8_t> imageData;

            ROS_DEBUG("Copying data");

            for (int i = 0; i < message->data.size(); i++) {
                imageData.push_back(message->data[i]);
            }
            try {
                d_cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
            }
            catch (cv::Exception &e) {
                ROS_ERROR("%s", e.what());
            }

            size_t rows = d_cv_ptr->image.rows;
            size_t cols = d_cv_ptr->image.cols;

            if ((rows > 0) && (cols > 0)) {
                d_pub.publish(d_cv_ptr->toImageMsg());
            }
        }

    };

    PLUGINLIB_DECLARE_CLASS(pepper_image_sink, PepperImageSink, pepper_image_sink::PepperImageSink, nodelet::Nodelet
    );
}
