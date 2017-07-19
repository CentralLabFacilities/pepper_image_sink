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
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <object_tracking_msgs/Recognize.h>

namespace pepper_image_sink {

    class PepperImageSink : public nodelet::Nodelet {
    public:
        PepperImageSink(){}

    private:
        virtual void onInit() {
            private_nh = getPrivateNodeHandle();
            stream = true;
            pub = private_nh.advertise<sensor_msgs::Image>("out", 10);
            sub = private_nh.subscribe("in", 10, &PepperImageSink::callback, this);
            service = private_nh.advertiseService("image_stream_toggle", &PepperImageSink::toggle_cb, this);
        }

        bool toggle_cb(object_tracking_msgs::Recognize::Request &req, object_tracking_msgs::Recognize::Response &res) {
            stream = !stream;
            if (!stream) {
                sub.shutdown();
            } else {
                sub = private_nh.subscribe("in", 10, &PepperImageSink::callback, this);
            }
        }

        void callback(const sensor_msgs::CompressedImage::ConstPtr &input) {
            if (stream) {
                try {
                    this->cv_ptr = cv_bridge::toCvCopy(input);
                }
                catch (cv_bridge::Exception &e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }


                try {
                    this->output = this->cv_ptr->toImageMsg();
                }
                catch (cv_bridge::Exception &e) {
                    ROS_ERROR("cv_bridge !exception: %s", e.what());
                    return;
                }
                pub.publish(this->output);
            }
        }

        ros::Publisher pub;
        ros::Subscriber sub;
        ros::ServiceServer service;
        ros::NodeHandle private_nh;
        bool stream;
        cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::ImagePtr output;
    };

    PLUGINLIB_DECLARE_CLASS(pepper_image_sink, PepperImageSink, pepper_image_sink::PepperImageSink, nodelet::Nodelet);
}
