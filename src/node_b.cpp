#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <visualization_msgs/MarkerArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <vector>
#include <algorithm>


std::string received_ids;

std::vector<int> target_ids;  // Received from Node A
ros::Publisher feedback_pub;
tf::TransformListener* tf_listener;

void targetIdsCallback(const std_msgs::String::ConstPtr& msg) {
    target_ids.clear();
    std::stringstream ss(msg->data);
    std::string id_str;

    while (std::getline(ss, id_str, ',')) {
        try {
            int id = std::stoi(id_str);
            target_ids.push_back(id);
        } catch (...) {
            ROS_WARN("Invalid ID received: %s", id_str.c_str());
        }
    }

    ROS_INFO("[Node B] Received target IDs:");
    for (int id : target_ids) {
        ROS_INFO("- %d", id);
    }
}


void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    for (const auto& detection : msg->detections) {
        int id = detection.id[0];
        
	ROS_INFO("[Node B] tagDetectionsCallback triggered with %lu detections", msg->detections.size());

        if (std::find(target_ids.begin(), target_ids.end(), id) != target_ids.end()) {
            ROS_INFO("[Node B] Found target ID: %d", id);

            geometry_msgs::PoseStamped camera_pose;
            camera_pose.header = detection.pose.header;  // use the same timestamp/frame
            camera_pose.pose = detection.pose.pose.pose; // get the actual pose (without covariance)

            // Transform to map frame
            geometry_msgs::PoseStamped map_pose;

            try {
                tf_listener->transformPose("map", camera_pose, map_pose);
                ROS_INFO("[Node B] ID %d in map frame: [%.2f, %.2f, %.2f]",
                         id,
                         map_pose.pose.position.x,
                         map_pose.pose.position.y,
                         map_pose.pose.position.z);
            } catch (tf::TransformException &ex) {
                ROS_WARN("Transform error: %s", ex.what());
            }
        }
    }

    std_msgs::String feedback;
    feedback.data = "Scanning tags...";
    feedback_pub.publish(feedback);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    tf_listener = new tf::TransformListener;

    ros::Subscriber ids_sub = nh.subscribe("/target_ids", 10, targetIdsCallback);
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);
    feedback_pub = nh.advertise<std_msgs::String>("/node_b_feedback", 10);

    ros::spin();
    
    return 0;
}

