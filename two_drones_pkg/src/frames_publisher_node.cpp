#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

class FramesPublisherNode {
    ros::NodeHandle nh;
    ros::Time startup_time;
    ros::Timer heartbeat;
    // Add transform broadcaster as class member
    tf::TransformBroadcaster br;

public:
    FramesPublisherNode() {
        // NOTE: This method is run once, when the node is launched.
        startup_time = ros::Time::now();
        heartbeat = nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
        heartbeat.start();
    }

    void onPublish(const ros::TimerEvent&) {
        // 1. Compute elapsed time since startup
        double time = (ros::Time::now() - startup_time).toSec();

        // Create transforms
        tf::Transform AV1World(tf::Transform::getIdentity());
        tf::Transform AV2World(tf::Transform::getIdentity());

        // 2. Set transforms for both AVs
        // For AV1:
        // - Position follows circular path on x-y plane
        tf::Vector3 av1_position(cos(time), sin(time), 0.0);
        AV1World.setOrigin(av1_position);

        // - Orientation: y axis tangent to trajectory, z parallel to world z
        // The tangent vector to a circle at (cos(t), sin(t)) is (-sin(t), cos(t))
        // We want this to be our y axis, which means yaw = time
        tf::Quaternion av1_orientation;
        av1_orientation.setRPY(0, 0, time);  // Roll=0, Pitch=0, Yaw=time
        AV1World.setRotation(av1_orientation);

        // For AV2:
        // - Position follows parabolic arc on x-z plane
        tf::Vector3 av2_position(sin(time), 0.0, cos(2*time));
        AV2World.setOrigin(av2_position);
        
        // - Orientation parallel to world frame
        tf::Quaternion av2_orientation;
        av2_orientation.setRPY(0, 0, 0);  // Keep aligned with world frame
        AV2World.setRotation(av2_orientation);

        // 3. Publish the transforms
        br.sendTransform(tf::StampedTransform(AV1World, ros::Time::now(), "world", "av1"));
        br.sendTransform(tf::StampedTransform(AV2World, ros::Time::now(), "world", "av2"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frames_publisher_node");
    FramesPublisherNode node;
    ros::spin();
    return 0;
}