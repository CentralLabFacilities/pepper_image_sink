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

namespace pepper_image_sink {

    class PepperImageSink : public nodelet::Nodelet {
    public:
        PepperImageSink()
                : value_(0) {}

    private:
        virtual void onInit() {
            ros::NodeHandle &private_nh = getPrivateNodeHandle();
            private_nh.getParam("value", value_);
            pub = private_nh.advertise<sensor_msgs::Image>("out", 10);
            sub = private_nh.subscribe("in", 10, &PepperImageSink::callback, this);
        }

        void callback(const sensor_msgs::CompressedImage::ConstPtr &input) {
            try {
                this->cv_ptr = cv_bridge::toCvCopy(input);
                //ROS_INFO("callback1");
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }


            try {
                this->output = this->cv_ptr->toImageMsg();
                //ROS_INFO("callback2");

            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge !exception: %s", e.what());
                return;
            }
            pub.publish(this->output);
        }

        ros::Publisher pub;
        ros::Subscriber sub;
        double value_;
        cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::ImagePtr output;
    };

    PLUGINLIB_DECLARE_CLASS(pepper_image_sink, PepperImageSink, pepper_image_sink::PepperImageSink, nodelet::Nodelet);
}
