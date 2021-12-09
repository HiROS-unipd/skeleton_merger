// Standard dependencies
#include <numeric>
#include <set>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_merger/utils.h"

void hiros::merge::utils::merge(hiros::skeletons::types::Skeleton& t_s1,
                                const skeletons::types::Skeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  t_s1.src_time = wavg(t_s1.src_time, t_s2.src_time, t_w1, t_w2);

  for (auto& s2_mk : t_s2.markers) {
    merge(t_s1, s2_mk, t_w1, t_w2, t_weight_by_confidence);
  }

  for (auto& s2_lk : t_s2.links) {
    merge(t_s1, s2_lk, t_w1, t_w2, t_weight_by_confidence);
  }

  t_s1.bounding_box = skeletons::utils::computeBoundingBox(t_s1);
}

void hiros::merge::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::Marker& t_mk2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasMarker(t_mk2.id)) {
    auto& mk1 = t_sk1.getMarker(t_mk2.id);
    mk1 = t_weight_by_confidence ? wavg(mk1, t_mk2, t_w1 * mk1.confidence, t_w2 * t_mk2.confidence)
                                 : wavg(mk1, t_mk2, t_w1, t_w2);
  }
  else {
    t_sk1.addMarker(t_mk2);
  }
}

void hiros::merge::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::Link& t_lk2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasLink(t_lk2.id)) {
    auto& lk1 = t_sk1.getLink(t_lk2.id);
    lk1 = t_weight_by_confidence ? wavg(lk1, t_lk2, t_w1 * lk1.confidence, t_w2 * t_lk2.confidence)
                                 : wavg(lk1, t_lk2, t_w1, t_w2);
  }
  else {
    t_sk1.addLink(t_lk2);
  }

  // Fix link center if possible
  if (t_sk1.hasMarker(t_lk2.parent_marker) && t_sk1.hasMarker(t_lk2.child_marker)) {
    t_sk1.getLink(t_lk2.id).center.pose.position = wavg(t_sk1.getMarker(t_lk2.parent_marker).center.pose.position,
                                                        t_sk1.getMarker(t_lk2.child_marker).center.pose.position);
  }

  // Align link orientation if possible
  alignLinkOrientation(t_sk1, t_lk2.id);
}

void hiros::merge::utils::alignLinkOrientation(hiros::skeletons::types::Skeleton& t_sk, const int& t_lk_id)
{
  if (!t_sk.hasLink(t_lk_id)) {
    return;
  }

  auto& link = t_sk.getLink(t_lk_id);

  if (skeletons::utils::isNaN(link.center.pose.orientation) || !t_sk.hasMarker(link.parent_marker)
      || !t_sk.hasMarker(link.child_marker)) {
    return;
  }

  auto link_axis =
    (t_sk.getMarker(link.parent_marker).center.pose.position - t_sk.getMarker(link.child_marker).center.pose.position)
      .normalized();

  auto closest_cartesian_axis =
    closestCartesianAxis(tf2::quatRotate(link.center.pose.orientation.inverse(), link_axis).normalized());

  // Axis of the link orientation SoR to be aligned to the link axis
  auto quat_axis = tf2::quatRotate(link.center.pose.orientation, closest_cartesian_axis).normalized();

  auto rot_axis = quat_axis.cross(link_axis).normalized();
  auto rot_angle = acos(quat_axis.dot(link_axis));
  // Rotation to align the link orientation to the link axis
  auto rot_quat = tf2::Quaternion(rot_axis, rot_angle);

  link.center.pose.orientation = rot_quat * link.center.pose.orientation;
}

tf2::Vector3 hiros::merge::utils::closestCartesianAxis(const tf2::Vector3& t_vec)
{
  auto closest_axis_idx = t_vec.closestAxis(); // 0: x, 1: y, 2: z

  tf2::Vector3DoubleData signs;
  signs.m_floats[0] = t_vec.x() >= 0 ? 1 : -1;
  signs.m_floats[1] = t_vec.y() >= 0 ? 1 : -1;
  signs.m_floats[2] = t_vec.z() >= 0 ? 1 : -1;

  tf2::Vector3DoubleData closest_axis_serialized{0, 0, 0};
  closest_axis_serialized.m_floats[closest_axis_idx] = signs.m_floats[closest_axis_idx];

  tf2::Vector3 closest_axis;
  closest_axis.deSerialize(closest_axis_serialized);

  return closest_axis;
}

double hiros::merge::utils::wavg(const double& t_e1, const double& t_e2, const double& t_w1, const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return (1 - weight) * t_e1 + weight * t_e2;
}

tf2::Vector3
hiros::merge::utils::wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_v1.lerp(t_v2, weight);
}

tf2::Quaternion hiros::merge::utils::wavg(const tf2::Quaternion& t_q1,
                                          const tf2::Quaternion& t_q2,
                                          const double& t_w1,
                                          const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_q1.normalized().slerp(t_q2.normalized(), weight).normalize();
}

hiros::skeletons::types::KinematicState hiros::merge::utils::wavg(const skeletons::types::KinematicState& t_ks1,
                                                                  const skeletons::types::KinematicState& t_ks2,
                                                                  const double& t_w1,
                                                                  const double& t_w2)
{
  skeletons::types::KinematicState avg_ks;

  avg_ks.pose.position = wavg(t_ks1.pose.position, t_ks2.pose.position, t_w1, t_w2);
  avg_ks.pose.orientation = wavg(t_ks1.pose.orientation, t_ks2.pose.orientation, t_w1, t_w2);
  avg_ks.velocity.linear = wavg(t_ks1.velocity.linear, t_ks2.velocity.linear, t_w1, t_w2);
  avg_ks.velocity.angular = wavg(t_ks1.velocity.angular, t_ks2.velocity.angular, t_w1, t_w2);
  avg_ks.acceleration.linear = wavg(t_ks1.acceleration.linear, t_ks2.acceleration.linear, t_w1, t_w2);
  avg_ks.acceleration.angular = wavg(t_ks1.acceleration.angular, t_ks2.acceleration.angular, t_w1, t_w2);

  return avg_ks;
}

hiros::skeletons::types::Marker hiros::merge::utils::wavg(const skeletons::types::Marker& t_mk1,
                                                          const skeletons::types::Marker& t_mk2,
                                                          const double& t_w1,
                                                          const double& t_w2)
{
  if (t_mk1.id != t_mk2.id) {
    std::cerr << "Warning: trying to average different markers" << std::endl;
    return skeletons::types::Marker();
  }

  skeletons::types::Marker avg_mk(t_mk1);

  avg_mk.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_mk1.confidence + t_w2 * t_mk2.confidence) / (t_w1 + t_w2) : 0.;
  avg_mk.center = wavg(t_mk1.center, t_mk2.center, t_w1, t_w2);

  return avg_mk;
}

hiros::skeletons::types::Link hiros::merge::utils::wavg(const skeletons::types::Link& t_lk1,
                                                        const skeletons::types::Link& t_lk2,
                                                        const double& t_w1,
                                                        const double& t_w2)
{
  if (t_lk1.id != t_lk2.id) {
    std::cerr << "Warning: trying to average different links" << std::endl;
    return skeletons::types::Link();
  }

  skeletons::types::Link avg_lk(t_lk1);

  avg_lk.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_lk1.confidence + t_w2 * t_lk2.confidence) / (t_w1 + t_w2) : 0.;
  avg_lk.center = wavg(t_lk1.center, t_lk2.center, t_w1, t_w2);

  return avg_lk;
}

std::vector<unsigned int>
hiros::merge::utils::split(const std::vector<hiros::skeletons::types::KinematicState>& t_states,
                           const double& t_max_position_delta,
                           const double& t_max_orientation_delta)
{
  if (t_states.empty()) {
    return {};
  }

  std::vector<unsigned int> missing_idxs(t_states.size());
  // Fill with 0, 1, ..., size(t_states)
  std::iota(std::begin(missing_idxs), std::end(missing_idxs), 0);

  std::vector<std::set<unsigned int>> groups;

  while (!missing_idxs.empty()) {
    // Initialize the group with the first unassigned kinematic state
    groups.push_back(std::set<unsigned int>{missing_idxs.front()});

    auto group_idx = groups.size() - 1;
    for (const auto& first_ks_idx : groups.at(group_idx)) {
      for (unsigned int ks_idx = 0; ks_idx < t_states.size(); ++ks_idx) {
        // Fill the group with kinematic states close to the first one
        auto pos_dist =
          hiros::skeletons::utils::distance(t_states.at(first_ks_idx).pose.position, t_states.at(ks_idx).pose.position);
        auto or_dist = hiros::skeletons::utils::distance(t_states.at(first_ks_idx).pose.orientation,
                                                         t_states.at(ks_idx).pose.orientation);

        if ((t_max_position_delta <= 0 || pos_dist < t_max_position_delta)
            && (t_max_orientation_delta <= 0 || or_dist < t_max_orientation_delta)) {
          groups.at(group_idx).insert(ks_idx);
        }
      }
    }

    // Erase indexes that were assigned to the group from missing_idxs
    missing_idxs.erase(std::remove_if(missing_idxs.begin(),
                                      missing_idxs.end(),
                                      [&](const auto& e) { return groups.at(group_idx).contains(e); }),
                       missing_idxs.end());
  }

  // If all the kinematic states belong to the same group, return the only group
  if (groups.size() == 1) {
    return std::vector<unsigned int>(groups.front().begin(), groups.front().end());
  }

  // Sort by size
  std::sort(
    std::begin(groups), std::end(groups), [](const auto& lhs, const auto& rhs) { return lhs.size() > rhs.size(); });

  // If the two largest groups have the same number of elements, return empty set
  if (groups.at(0).size() == groups.at(1).size()) {
    return {};
  }

  // Return the group with the largest number of elements
  return std::vector<unsigned int>(groups.front().begin(), groups.front().end());
}
