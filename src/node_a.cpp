#include <ros/ros.h>
#include <ir2425_group_16/ApriltagIDs.h>
#include <ir2425_group_16/Coeffs.h>  // Include the service header

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    ros::Publisher id_pub = nh.advertise<ir2425_group_16::ApriltagIDs>("tag_ids", 10);
    ros::ServiceClient client = nh.serviceClient<ir2425_group_16::Coeffs>("/apriltags_ids_srv");

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ir2425_group_16::ApriltagIDs msg;
        ir2425_group_16::Coeffs srv;
        srv.request.ready = true;

        if (client.call(srv)) {
            for (float coeff : srv.response.coeffs) {
                msg.ids.push_back(static_cast<int>(coeff));
            }
            id_pub.publish(msg);
            ROS_INFO("Node A published IDs from service.");
        } else {
            ROS_WARN("Failed to call service /apriltags_ids_srv");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

