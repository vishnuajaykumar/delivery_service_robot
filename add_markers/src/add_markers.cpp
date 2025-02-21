#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const double PICKUP_X = 2.595;
const double PICKUP_Y = -1.450;
const double DROPOFF_X = -2.94;
const double DROPOFF_Y = -2.29;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate r(1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pickup_dropoff";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok()) return 0;
        ROS_WARN_ONCE("Waiting for a subscriber to visualization_marker");
        sleep(1);
    }

    // Publish marker at pickup zone
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker_pub.publish(marker);
    ROS_INFO("Marker placed at pickup zone");
    ros::Duration(5.0).sleep();

    // Hide marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Marker hidden");
    ros::Duration(5.0).sleep();

    // Publish marker at drop-off zone
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = DROPOFF_X;
    marker.pose.position.y = DROPOFF_Y;
    marker_pub.publish(marker);
    ROS_INFO("Marker placed at drop-off zone");
    
    ros::spin(); // Keep node alive
    return 0;
}
