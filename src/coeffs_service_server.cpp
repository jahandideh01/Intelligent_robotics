#include <ros/ros.h>
#include <ir2425_group_16/Coeffs.h>
#include <cstdlib>
#include <vector>

bool provideCoeffs(ir2425_group_16::Coeffs::Request &req,
                   ir2425_group_16::Coeffs::Response &res) {
    if (req.ready) {
        res.coeffs = {1.0, 2.0, 3.0, 4.0}; // Example values
        ROS_INFO("Service called: sending coefficients.");
    } else {
        ROS_WARN("Service called with ready = false.");
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "coeffs_service_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/apriltags_ids_srv", provideCoeffs);
    ROS_INFO("Dummy Coeffs Service Server Ready.");
    ros::spin();

    return 0;
}

