#ifndef hiros_skeleton_merger_Merger_h
#define hiros_skeleton_merger_Merger_h

// ROS dependencies
#include <ros/ros.h>

// Custom ROS Message dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace merge {

    struct MergerParameters
    {
      std::string node_name;

      std::string input_topic;
      std::string output_topic;

      int n_detectors;
      double max_delta_t;
    };

    class Merger
    {
    public:
      Merger() {}
      ~Merger() {}

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();

      void callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      void estimateNumberOfDetectors();
      bool okToMerge() const;
      void mergeSkeletons();
      void publish();
      void addSkeletonToBuffer();

      void computeAvgSkeleton(const int& t_id);

      ros::NodeHandle m_nh{"~"};

      MergerParameters m_params{};

      ros::Subscriber m_in_skeleton_group_sub{};
      ros::Publisher m_out_skeleton_group_pub{};

      unsigned long m_n_detectors{0};
      std::set<std::string> m_src_frames{};

      hiros::skeletons::types::SkeletonGroup m_last_skeleton_group{};
      hiros::skeletons::types::SkeletonGroup m_merged_skeletons{};

      // map<skeleton_id, vector<skeletons>>
      std::map<int, std::vector<hiros::skeletons::types::Skeleton>> m_skeletons_to_merge{};

      bool m_configured{false};
    };

  } // namespace merge
} // namespace hiros

#endif
