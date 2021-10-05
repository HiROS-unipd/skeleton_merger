#ifndef hiros_skeleton_merger_utils_h
#define hiros_skeleton_merger_utils_h

// Custom External Packages dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace merge {
    namespace utils {

      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Skeleton& t_sk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Marker& t_mk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Link& t_lk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);

      double wavg(const double& t_e1, const double& t_e2, const double& t_w1 = 1, const double& t_w2 = 1);
      tf2::Vector3
      wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1 = 1, const double& t_w2 = 1);
      tf2::Quaternion
      wavg(const tf2::Quaternion& t_q1, const tf2::Quaternion& t_q2, const double& t_w1 = 1, const double& t_w2 = 1);

      hiros::skeletons::types::KinematicState wavg(const hiros::skeletons::types::KinematicState& t_ks1,
                                                   const hiros::skeletons::types::KinematicState& t_ks2,
                                                   const double& t_w1 = 1,
                                                   const double& t_w2 = 1);

      hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& t_mk1,
                                           const hiros::skeletons::types::Marker& t_mk2,
                                           const double& t_w1 = 1,
                                           const double& t_w2 = 1);

      hiros::skeletons::types::Link wavg(const hiros::skeletons::types::Link& t_lk1,
                                         const hiros::skeletons::types::Link& t_lk2,
                                         const double& t_w1 = 1,
                                         const double& t_w2 = 1);

    } // namespace utils
  } // namespace merge
} // namespace hiros

#endif
