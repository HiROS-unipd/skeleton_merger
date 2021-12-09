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
