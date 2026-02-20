#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rubiks_cube_markers_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("rubiks_cube_marker", 1);

    ros::Publisher desk_marker_pub =
        nh.advertise<visualization_msgs::Marker>("desk_marker", 1);

    ros::Rate rate(30.0);

    while (ros::ok())
    {

        visualization_msgs::Marker marker;
        marker.header.frame_id = "rubiks_cube";
        marker.pose.position.y = 0.0125;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Tamanho 2,5 cm
        marker.scale.x = 0.025;
        marker.scale.y = 0.025;
        marker.scale.z = 0.025;

        // Cor vermelho
        marker.color.r = 1.0;
        marker.color.g = 0.22;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0);

        marker_pub.publish(marker);

        

        // Mesa para referência
        visualization_msgs::Marker marker_desk;
        marker_desk.header.frame_id = "desk";
        marker_desk.type = visualization_msgs::Marker::CUBE;
        marker_desk.action = visualization_msgs::Marker::ADD;

        marker_desk.scale.x = 0.50;
        marker_desk.scale.y = 0.90;
        marker_desk.scale.z = 0.01;

        // Cor branco translúcido
        marker_desk.color.r = 1.0;
        marker_desk.color.g = 1.0;
        marker_desk.color.b = 1.0;
        marker_desk.color.a = 0.5;

        marker_desk.lifetime = ros::Duration(0);
        desk_marker_pub.publish(marker_desk);

        rate.sleep();
    }

    return 0;
}
