#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <list>

class PlotsPublisherNode {
    ros::Time startup_time;
    ros::Timer heartbeat;
    ros::NodeHandle nh;
    ros::Publisher markers_pub;
    tf::TransformListener tf_listener;
    int num_trails;

    class TrajTrail {
        PlotsPublisherNode* parent;
        static int id;
        std::list<geometry_msgs::Point> poses;
        std::string ref_frame, dest_frame;
        size_t buffer_size;

        std::string ns;
        float r, g, b, a;

        visualization_msgs::Marker marker_out;

        void update() {
            tf::StampedTransform transform;
            try {
                // Wait for transform to become available
                if (parent->tf_listener.waitForTransform(ref_frame, dest_frame, 
                                                       ros::Time(0), ros::Duration(0.1))) {
                    // Look up transform
                    parent->tf_listener.lookupTransform(ref_frame, dest_frame,
                                                      ros::Time(0), transform);

                    while(poses.size() >= buffer_size)
                        poses.pop_front();

                    tf::Vector3 pos(transform.getOrigin());
                    geometry_msgs::Point tmp;
                    tmp.x = pos.x();
                    tmp.y = pos.y();
                    tmp.z = pos.z();
                    poses.push_back(tmp);
                }
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
            }
        }

    public:
        TrajTrail():parent(NULL) {};

        TrajTrail(PlotsPublisherNode* parent_, const std::string& ref_frame_, 
                 const std::string& dest_frame_, int buffer = 160):
            parent(parent_), ref_frame(ref_frame_), dest_frame(dest_frame_), 
            buffer_size(buffer) {
            marker_out.header.frame_id = ref_frame;
            marker_out.ns = "trails";
            marker_out.id = parent->num_trails++;
            marker_out.type = visualization_msgs::Marker::LINE_STRIP;
            marker_out.action = visualization_msgs::Marker::ADD;
            marker_out.color.a = 0.8;
            marker_out.scale.x = 0.02;
            marker_out.lifetime = ros::Duration(1.0);
        }

        void setColor(float r_, float g_, float b_) {
            marker_out.color.r = r_;
            marker_out.color.g = g_;
            marker_out.color.b = b_;
        }

        void setNamespace(const std::string& ns_) {
            marker_out.ns = ns_;
        }

        void setDashed() {
            marker_out.type = visualization_msgs::Marker::LINE_LIST;
        }

        visualization_msgs::Marker getMarker() {
            update();
            marker_out.header.stamp = ros::Time::now();
            marker_out.points.clear();
            for(auto& p:poses)
                marker_out.points.push_back(p);

            if(marker_out.type == visualization_msgs::Marker::LINE_LIST
               && poses.size()%2)
                marker_out.points.resize(poses.size()-1);

            return marker_out;
        }
    };

    TrajTrail av1trail, av2trail, av2trail_rel;

public:
    PlotsPublisherNode():num_trails(0) {
        startup_time = ros::Time::now();
        markers_pub = nh.advertise<visualization_msgs::MarkerArray>("visuals", 0);
        heartbeat = nh.createTimer(ros::Duration(0.02), &PlotsPublisherNode::onPublish, this);
        heartbeat.start();

        // Set up AV1 trail
        av1trail = TrajTrail(this, "world", "av1", 300);
        av1trail.setColor(.25, .52, 1);
        av1trail.setNamespace("Trail av1-world");

        // Set up AV2 trail
        av2trail = TrajTrail(this, "world", "av2", 300);
        av2trail.setColor(.8, .4, .26);
        av2trail.setNamespace("Trail av2-world");

        // Set up AV2 relative trail
        av2trail_rel = TrajTrail(this, "av1", "av2", 160);
        av2trail_rel.setDashed();
        av2trail_rel.setColor(.8, .4, .26);
        av2trail_rel.setNamespace("Trail av2-av1");
    }

    void onPublish(const ros::TimerEvent&) {
        visualization_msgs::MarkerArray visuals;
        visuals.markers.resize(2);

        // Set up AV1 marker
        visualization_msgs::Marker& av1(visuals.markers[0]);
        av1.header.frame_id = "av1";
        av1.ns = "AVs";
        av1.id = 0;
        av1.header.stamp = ros::Time();
        av1.type = visualization_msgs::Marker::MESH_RESOURCE;
        av1.mesh_resource = "package://two_drones_pkg/mesh/quadrotor.dae";
        av1.action = visualization_msgs::Marker::ADD;
        av1.pose.orientation.w = 1.0;
        av1.scale.x = av1.scale.y = av1.scale.z = av1.color.a = 1.0;
        av1.color.r = .25;
        av1.color.g = .52;
        av1.color.b = 1;
        av1.lifetime = ros::Duration(1.0);

        // Set up AV2 marker
        visualization_msgs::Marker& av2(visuals.markers[1]);
        av2 = av1;
        av2.header.frame_id = "av2";
        av2.ns = "AVs";
        av2.id = 1;
        av2.color.r = .8;
        av2.color.g = .4;
        av2.color.b = .26;

        // Add trail markers
        visuals.markers.push_back(av1trail.getMarker());
        visuals.markers.push_back(av2trail.getMarker());
        visuals.markers.push_back(av2trail_rel.getMarker());

        // Publish all markers
        markers_pub.publish(visuals);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "plots_publisher_node");
    PlotsPublisherNode node;
    ros::spin();
    return 0;
}