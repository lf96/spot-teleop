#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseStamped.h>

using namespace visualization_msgs;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher pose_pub;
ros::Time last_pub_time;


void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
    if ((ros::Time::now() - last_pub_time).toSec() < 0.5)
        return;
    
    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "base"; 
        goal.header.stamp = ros::Time::now();
        goal.pose = feedback->pose;

        pose_pub.publish(goal);
        last_pub_time = ros::Time::now();
    }
}

void make6DofMarker()
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base";
    int_marker.name = "ee_goal_marker";
    int_marker.description = "EE Goal";
    int_marker.scale = 0.3;

    int_marker.pose.position.x = 0.4;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.4;

    InteractiveMarkerControl control;
    

    // X axis
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Y axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Z axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Free movement (true 3D drag)
    InteractiveMarkerControl move3d_control;
    move3d_control.name = "move_3d";
    move3d_control.interaction_mode =
        InteractiveMarkerControl::MOVE_ROTATE_3D;

    move3d_control.orientation.w = 1;
    move3d_control.orientation.x = 0;
    move3d_control.orientation.y = 0;
    move3d_control.orientation.z = 0;

    move3d_control.always_visible = true;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.r = 0.2;
    marker.color.g = 0.8;
    marker.color.b = 0.1;
    marker.color.a = 1.0;

    move3d_control.markers.push_back(marker);
    int_marker.controls.push_back(move3d_control);

    server->insert(int_marker, &processFeedback);
    server->applyChanges();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ee_marker_node");
    ros::NodeHandle nh;

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_goal_pose", 1);

    server.reset(new interactive_markers::InteractiveMarkerServer("ee_marker_server"));

    make6DofMarker();

    ros::spin();
    return 0;
}
