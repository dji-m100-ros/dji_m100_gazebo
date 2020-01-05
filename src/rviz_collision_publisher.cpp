#include <gazebo_msgs/GetModelState.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace {
constexpr const char *kModelSrvName = "/gazebo/get_model_state";
constexpr double radius = 2.5;
} // namespace
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_collision_publisher");
  ros::NodeHandle nh;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools("/world", "/obstacles"));
  ros::ServiceClient gazebo_client =
      nh.serviceClient<gazebo_msgs::GetModelState>(kModelSrvName);
  gazebo_client.waitForExistence();

  gazebo_msgs::GetModelState get_model_state;
  std::vector<geometry_msgs::Point> sphere_centers;
  int nr_of_obstacles;
  nh.getParam("nr_of_obstacles",nr_of_obstacles);
  for (int i = 0; i < nr_of_obstacles; i++) {
    std::string sphere = "collision_sphere_clone_" + std::to_string(i);
    get_model_state.request.model_name = sphere;
    if (gazebo_client.call(get_model_state)) {
      sphere_centers.push_back(get_model_state.response.pose.position);
    }
  }
  visual_tools_->loadMarkerPub();
  visual_tools_->waitForMarkerPub();
  visual_tools_->setLifetime(0);
  visual_tools_->publishSpheres(sphere_centers, rviz_visual_tools::MAGENTA, radius*2.0,
                                "Obstacles");
  visual_tools_->trigger();
  ros::Duration(1.0).sleep();
  return 0;
}