#include <stdio.h>
#include <vector>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <visualization_msgs/MarkerArray.h>


class PhaseSpaceViz
{
  public:

    // the node handle
    ros::NodeHandle nh_;

    // node handle in the private namespace
    ros::NodeHandle priv_nh_;
    
    // publishers
    ros::Publisher pub_rviz_markers_;

    // subscribers
    ros::Subscriber sub_phase_space_markers_;

    // the rviz markers
    visualization_msgs::MarkerArray rviz_markers_;
    visualization_msgs::Marker rviz_marker_;
    visualization_msgs::Marker id_marker_;
    
  public:

    // callback functions
    void publishRVIZMarkers();

    // constructor
    PhaseSpaceViz(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

        // initialize the RVIZ markers
        // the leds
        rviz_marker_.ns = "my_namespace";
        rviz_marker_.header.frame_id = "world";
        rviz_marker_.type = visualization_msgs::Marker::CUBE;
        rviz_marker_.action = visualization_msgs::Marker::ADD;
        rviz_marker_.scale.x = 0.2;
        rviz_marker_.scale.y = 0.2;
        rviz_marker_.scale.z = 1.0;
        rviz_marker_.pose.orientation.x = 0.0;
        rviz_marker_.pose.orientation.y = 0.0;
        rviz_marker_.pose.orientation.z = 0.0;
        rviz_marker_.pose.orientation.w = 1.0;
        rviz_marker_.color.r = 1.0f;
        rviz_marker_.color.g = 0.0f;
        rviz_marker_.color.b = 0.0f;
        rviz_marker_.color.a = 1.0;
        rviz_marker_.lifetime = ros::Duration();
        
        
        // advertise topics
        pub_rviz_markers_ = nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("/rviz_markers"), 10);

    }
    
    //! Empty stub
    ~PhaseSpaceViz() {}

};

// this function is called as fast as ROS can from the main loop directly
void PhaseSpaceViz::publishRVIZMarkers()
{
    for(int i = 1; i<100 ; ++i)
    {
        // fill the led marker
        
        rviz_marker_.id = i;
        rviz_marker_.pose.position.x = i;
        rviz_marker_.pose.position.y = i;
        rviz_marker_.pose.position.z = i;


        rviz_markers_.markers.push_back(rviz_marker_);
    }
    pub_rviz_markers_.publish(rviz_markers_);
    rviz_markers_.markers.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "phase_space_viz_publisher");
    ros::NodeHandle nh;

    PhaseSpaceViz node(nh);
    int i=0;
    while(i<100)
    {
        node.publishRVIZMarkers();
        ros::Duration(0.01).sleep();
        i++;
        ROS_INFO("i : %d",i);
    }
    
    return 0;
}