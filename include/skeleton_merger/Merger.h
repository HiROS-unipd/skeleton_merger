#ifndef hiros_skeleton_merger_Merger_h
#define hiros_skeleton_merger_Merger_h

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// Custom ROS message dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

// Custom external packages dependencies
#include "skeletons/types.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
namespace skeletons {

class Merger : public rclcpp::Node {
 public:
  Merger();
  ~Merger();

 private:
  struct Parameters {
    std::string input_topic{};
    std::string output_topic{};

    int n_detectors{};
    double max_delta_t{};

    double max_position_delta{};
    double max_orientation_delta{};

    int pelvis_marker_id{};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRosTopics();

  void estimateNumberOfDetectors();
  bool okToMerge() const;
  void mergeSkeletons();
  void publish();
  void addSkeletonToBuffer();

  void computeAvgSkeleton(const int& id);
  void removeFlippedTracks(
      std::vector<hiros::skeletons::types::Skeleton>& tracks);
  void removeOutliers(std::vector<hiros::skeletons::types::Skeleton>& tracks);
  void removeOutlierMarkers(
      std::vector<hiros::skeletons::types::Skeleton>& tracks);
  void removeOutlierLinks(
      std::vector<hiros::skeletons::types::Skeleton>& tracks);
  void cleanupLinks(hiros::skeletons::types::Skeleton& skel);

  std::vector<unsigned int> split(
      const std::vector<hiros::skeletons::types::KinematicState>& states,
      const double& max_position_delta,
      const double& max_orientation_delta) const;

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  Parameters params_{};

  rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      sub_{};
  rclcpp::Publisher<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr pub_{};

  unsigned long n_detectors_{0};
  std::set<std::string> src_frames_{};

  hiros::skeletons::types::SkeletonGroup last_skeleton_group_{};
  hiros::skeletons::types::SkeletonGroup merged_skeletons_{};
  hiros::skeletons::types::SkeletonGroup prev_merged_skeletons_{};

  // map<skeleton_id, vector<skeletons>>
  std::map<int, std::vector<hiros::skeletons::types::Skeleton>>
      skeletons_to_merge_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
