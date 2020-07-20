
/*****************************************************************************
**
* Dummy ros publisher that publishes some common ros time stamped messages 
*   Usefull if writing/debuugin code that depends on message_filters from hardware or
*   Robots that are not available.
*   -> Note. No support for value change in the actual message. This Node only publishes
*   empty messages (default for each message) but with an increasing timestamp (that is why
*   this node only supports TimeStamped messages right now.)
* 
*   Launch arguments are topic_name, publish_rate and msg_type (<- This is limited to timestamped Messages)
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <boost/variant.hpp>
// This function starts and creates a Node.
// The types of the messages are defined in this function

using namespace std;

class base_msgClass
{
public:
    virtual void create_pub(ros::NodeHandle *node_handle, std::string name) = 0;
    virtual void pub_data() = 0;
};

template <typename T>
class template_publisher : public base_msgClass
{
private:
    ros::Publisher pub_;

public:
    void create_pub(ros::NodeHandle *node_handle, std::string name)
    {
        pub_ = node_handle->advertise<T>(name, 1);
        return;
    }
    void pub_data()
    {
        T msg_type;
        msg_type.header.stamp = ros::Time::now();
        msg_type.header.frame_id = "world";

        pub_.publish(msg_type);
        return;
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummy_publisher_");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(1);

    // attributes
    std::string topic_pub_name_;
    std::string message_type_;
    int pub_rate_;
    std::string namespace_;

    // common stuff
    std::string node_name_ = ros::this_node::getName();
    nh_.getParam(node_name_ + "/topic_name", topic_pub_name_);
    nh_.getParam(node_name_ + "/msg_type", message_type_);
    nh_.getParam(node_name_ + "/publish_rate", pub_rate_);
    
    nh_.getParam(node_name_ + "/namespace", namespace_);

    std::stringstream append_;

    // create common msg_class
    base_msgClass *publisher_;
    std::string a_slash = "/";

    if (topic_pub_name_ == "/dummy_pub")
    {
        
        append_ <<  topic_pub_name_ << a_slash << message_type_;
        ROS_INFO_STREAM("Renaming default topic to Message Type Topic: " << append_.str());
    }
    else
    {
        append_ << namespace_<< a_slash << topic_pub_name_;
    }

    ROS_INFO_STREAM("Starting node:\n TopicName: " << append_.str() << " \n MessageType: " << message_type_ << " \n Publishrate: " << pub_rate_ << " \n --------------------");

    ros::Rate loop_rate(pub_rate_);

    // check for message and create new pointer instance with correct template type.

    if (message_type_ == "PointStamped")
    {
        publisher_ = new template_publisher<geometry_msgs::PointStamped>();
    }
    else if (message_type_ == "PoseStamped")
    {
        publisher_ = new template_publisher<geometry_msgs::PoseStamped>();
    }
    else if (message_type_ == "QuaternionStamped")
    {
        publisher_ = new template_publisher<geometry_msgs::QuaternionStamped>();
    }
    else if (message_type_ == "TwistStamped")
    {
        publisher_ = new template_publisher<geometry_msgs::TwistStamped>();
    }
    else if (message_type_ == "Odometry")
    {
        publisher_ = new template_publisher<nav_msgs::Odometry>();
    }
    else
    {
        ROS_INFO_STREAM("I do not know the message: " << message_type_ << ". I know: \n PoseStamped, PointStamped, QuaternionStamped, TwistStamped and Odometry.");
        return 0;
    }

    publisher_->create_pub(&nh_, append_.str());

    spinner.start();
    while (ros::ok())
    {
        publisher_->pub_data();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("shutdown wait ..");
    return 0;

    ROS_INFO_STREAM("see ya ..");
}
