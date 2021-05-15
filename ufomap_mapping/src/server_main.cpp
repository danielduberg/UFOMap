
#include "ufomap_mapping/server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_ufomap_server = std::make_shared<ufomap_mapping::UFOMapServer>();
  
  rclcpp::spin(node_ufomap_server);
  
  rclcpp::shutdown();

  return 0;
}