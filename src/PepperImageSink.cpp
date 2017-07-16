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
            cv_bridge::CvImagePtr cv_ptr;

            try {
                cv_ptr = cv_bridge::toCvCopy(input);
                //ROS_INFO("callback1");
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            sensor_msgs::ImagePtr output(new sensor_msgs::Image());
            try {
                output = cv_ptr->toImageMsg();
                //ROS_INFO("callback2");

            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge !exception: %s", e.what());
                return;
            }
            pub.publish(output);
        }

        ros::Publisher pub;
        ros::Subscriber sub;
        double value_;
    };

    PLUGINLIB_DECLARE_CLASS(pepper_image_sink, PepperImageSink, pepper_image_sink::PepperImageSink, nodelet::Nodelet);
}
