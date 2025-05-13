#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ir2425_group_16/GetApriltagIDs.h>  // Replace with your actual service

ros::Publisher target_pub;

void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node A] Feedback from Node B: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Setup publisher and subscriber
    target_pub = nh.advertise<std_msgs::String>("/target_ids", 10);
    ros::Subscriber feedback_sub = nh.subscribe("/node_b_feedback", 10, feedbackCallback);

    // Wait for service to be available
    ROS_INFO("[Node A] Waiting for /apriltag_ids_srv service...");
    ros::service::waitForService("/apriltag_ids_srv");
    ROS_INFO("[Node A] Service available, calling...");

    // Call the service
    ros::ServiceClient client = nh.serviceClient<ir2425_group_16::GetApriltagIDs>("/apriltag_ids_srv");
    ir2425_group_16::GetApriltagIDs srv;
    
    
    srv.request.ready = true;
    if (client.call(srv)) {
        std_msgs::String msg;
        std::stringstream ss;
        for (size_t i = 0; i < srv.response.ids.size(); ++i) {
            ss << srv.response.ids[i];
            if (i != srv.response.ids.size() -1) ss << ",";
        }
        msg.data = ss.str();
        ROS_INFO("[Node A] Received and publishing IDs: %s", msg.data.c_str());
        target_pub.publish(msg);
    } else {
        ROS_ERROR("[Node A] Failed to call /apriltag_ids_srv");
    }

    // Keep node alive to listen for feedback
    ROS_INFO("[Node A] Waiting for feedback from Node B...");
    ros::spin();

    return 0;
}

