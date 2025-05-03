#include <ros/ros.h>
#include <ir2425_group_16/ApriltagIDs.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;
    ros::Publisher id_pub = nh.advertise<ir2425_group_16::ApriltagIDs>("tag_ids", 10);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ir2425_group_16::ApriltagIDs msg;
        msg.ids = {1, 2, 3};  // fake IDs (later: call service to get real ones)
        id_pub.publish(msg);
        ROS_INFO("Node A published IDs.");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

