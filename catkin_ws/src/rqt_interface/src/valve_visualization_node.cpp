#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "valve_visualization_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("valve_marker", 1);

    ros::Rate rate(30.0);

    while (ros::ok())
    {

        visualization_msgs::Marker lever_marker;
        lever_marker.header.frame_id = "lever_pivot";

        lever_marker.ns = "valve_visualization";
        lever_marker.id = 0;

        lever_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        lever_marker.action = visualization_msgs::Marker::ADD;

        lever_marker.mesh_resource = "package://rqt_interface/resources/lever.dae";

        lever_marker.mesh_use_embedded_materials = true;

        // Adjust the scale if necessary
        lever_marker.scale.x = 1;
        lever_marker.scale.y = 1;
        lever_marker.scale.z = 1;

        // Rotation (if needed)
        lever_marker.pose.orientation.x = 0.5;
        lever_marker.pose.orientation.y = 0.5;
        lever_marker.pose.orientation.z = 0.5;
        lever_marker.pose.orientation.w = 0.5;

        lever_marker.lifetime = ros::Duration(0);

        marker_pub.publish(lever_marker);

        visualization_msgs::Marker body_marker;
        body_marker.header.frame_id = "body_pivot";

        body_marker.ns = "valve_visualization";
        body_marker.id = 1;

        body_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        body_marker.action = visualization_msgs::Marker::ADD;
        body_marker.mesh_resource = "package://rqt_interface/resources/body.dae";

        body_marker.mesh_use_embedded_materials = true;

        // Adjust the scale if necessary
        body_marker.scale.x = 1;
        body_marker.scale.y = 1;
        body_marker.scale.z = 1;

        // Rotation (if needed)
        body_marker.pose.orientation.x = 0.5;
        body_marker.pose.orientation.y = 0.5;
        body_marker.pose.orientation.z = 0.5;
        body_marker.pose.orientation.w = 0.5;

        body_marker.lifetime = ros::Duration(0);

        marker_pub.publish(body_marker);

        rate.sleep();
    }

    return 0;
}
