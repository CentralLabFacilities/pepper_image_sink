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

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <limits>
#include <vector>

#include "pepper_clf_msgs/SetImageStreaming.h"

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
        ros::ServiceServer color_stream_service;
        ros::ServiceServer depth_stream_service;
        ros::NodeHandle private_nh;
        cv_bridge::CvImagePtr c_cv_ptr;
        sensor_msgs::ImagePtr c_output;
        cv_bridge::CvImagePtr d_cv_ptr;
        sensor_msgs::ImagePtr d_output;

        virtual void onInit() {
            private_nh = getPrivateNodeHandle();
            c_pub = private_nh.advertise<sensor_msgs::Image>("out/color", 10);
            //c_sub = private_nh.subscribe("in/color", 10, &PepperImageSink::color_cb, this);
            d_pub = private_nh.advertise<sensor_msgs::Image>("out/depth", 10);
            //d_sub = private_nh.subscribe("in/depth", 10, &PepperImageSink::depth_cb, this);
            color_stream_service = private_nh.advertiseService("enable_color_stream", &PepperImageSink::enable_color_stream, this);
            depth_stream_service = private_nh.advertiseService("enable_depth_stream", &PepperImageSink::enable_depth_stream, this);
            ROS_INFO("Image Sink Nodelet 'onInit()' done.");
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

            size_t rows = d_cv_ptr->image.rows;
            size_t cols = d_cv_ptr->image.cols;

            // Assign image encoding
            std::string image_encoding = message->format.substr(0, message->format.find(';'));

            d_cv_ptr->encoding = image_encoding;

            std::vector <uint8_t> imageData;

            ROS_INFO("Copying data");

            for (int i = 0; i < message->data.size(); i++) {
                imageData.push_back(message->data[i]);
            }
            try {
                d_cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
            }
            catch (cv::Exception &e) {
                ROS_ERROR("%s", e.what());
            }

            if ((rows > 0) && (cols > 0)) {
                d_pub.publish(d_cv_ptr->toImageMsg());
            } else {
                ROS_ERROR("rows or cols not greater than 0");
            }
        }

        bool enable_color_stream(pepper_clf_msgs::SetImageStreaming::Request  &req,
                 pepper_clf_msgs::SetImageStreaming::Response &res)
        {
            ROS_INFO("Color Streaming Service called");
            if(req.enable) {
                ROS_WARN("req enabled");
                c_sub = private_nh.subscribe("in/color", 10, &PepperImageSink::color_cb, this);
            } else {
                ROS_WARN("req disabled");
                if(c_sub != NULL) {
                    ROS_WARN("Shutdown");
                    c_sub.shutdown();
                }
            }
            res.success = true;
            return true;
        }

        bool enable_depth_stream(pepper_clf_msgs::SetImageStreaming::Request  &req,
                 pepper_clf_msgs::SetImageStreaming::Response &res)
        {
            ROS_INFO("Depth Streaming Service called");
            if(req.enable) {
                ROS_WARN("req enabled");
                d_sub = private_nh.subscribe("in/depth", 10, &PepperImageSink::depth_cb, this);
            } else {
                ROS_WARN("req disabled");
                if(d_sub != NULL) {
                    ROS_WARN("Shutdown");
                    d_sub.shutdown();
                }
            }
            res.success = true;
            return true;
        }
    };

    PLUGINLIB_DECLARE_CLASS(pepper_image_sink, PepperImageSink, pepper_image_sink::PepperImageSink, nodelet::Nodelet);
}
