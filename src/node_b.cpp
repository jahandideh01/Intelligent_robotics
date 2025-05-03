#include <ros/ros.h>
#include <ir2425_group_16/ApriltagIDs.h>

void idsCallback(const ir2425_group_16::ApriltagIDs::ConstPtr& msg) {
    ROS_INFO("Node B received IDs:");
    for (auto id : msg->ids) {
        ROS_INFO(" - ID: %d", id);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;
    ros::Subscriber id_sub = nh.subscribe("tag_ids", 10, idsCallback);
    ros::spin();
    return 0;
}

