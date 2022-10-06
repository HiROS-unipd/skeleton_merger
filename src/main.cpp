#include "skeleton_merger/Merger.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::skeletons::Merger>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
