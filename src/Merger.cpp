// Custom external packages dependencies
#include "skeleton_tracker/utils.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_merger/Merger.h"

hiros::skeletons::Merger::Merger() : Node("hiros_skeleton_merger") { start(); }

hiros::skeletons::Merger::~Merger() { stop(); }

void hiros::skeletons::Merger::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Running" << BASH_MSG_RESET);
}

void hiros::skeletons::Merger::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Stopped" << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::skeletons::Merger::configure() {
  getParams();
  setupRosTopics();
}

void hiros::skeletons::Merger::getParams() {
  getParam("input_topic", params_.input_topic);
  getParam("output_topic", params_.output_topic);
  getParam("n_detectors", params_.n_detectors);
  getParam("max_delta_t", params_.max_delta_t);
  getParam("max_position_delta", params_.max_position_delta);
  getParam("max_orientation_delta", params_.max_orientation_delta);
  getParam("pelvis_marker_id", params_.pelvis_marker_id);

  if (params_.n_detectors > 0) {
    n_detectors_ = static_cast<unsigned long>(params_.n_detectors);
    // If n_detectors <= 0, then m_n_detectors will be estimated online based on
    // the skeletons src_frames
  }
}

void hiros::skeletons::Merger::setupRosTopics() {
  sub_ = create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.input_topic, 10,
      std::bind(&Merger::callback, this, std::placeholders::_1));

  pub_ = create_publisher<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.output_topic, 10);
}

void hiros::skeletons::Merger::estimateNumberOfDetectors() {
  if (params_.n_detectors > 0) {
    return;
  }

  for (const auto& skel : last_skeleton_group_.skeletons) {
    src_frames_.insert(skel.src_frame);
  }

  n_detectors_ = src_frames_.size();
}

bool hiros::skeletons::Merger::okToMerge() const {
  for (auto& pair : skeletons_to_merge_) {
    if (last_skeleton_group_.hasSkeleton(pair.first)) {
      auto& last_track{last_skeleton_group_.getSkeleton(pair.first)};
      auto last_src_time{last_track.src_time};
      auto last_src_frame{last_track.src_frame};

      // If one of the tracks to merge has its src_frame equal to the one of the
      // latest received track, and its src_time is lower than the one of the
      // last received track, then the last received track belongs to a new
      // frame, and it is time to compute the average track
      if (std::find_if(pair.second.begin(), pair.second.end(),
                       [&](const auto& elem) {
                         return (elem.src_frame == last_src_frame &&
                                 elem.src_time < last_src_time);
                       }) != pair.second.end()) {
        return true;
      }

      // If the delta between the src_time of the last received track and the
      // oldest track to merge is greater than max_delta_t, then the track
      // belongs to a new frame, and it is time to compute the average track
      if (params_.max_delta_t > 0 && !pair.second.empty() &&
          (last_src_time - pair.second.front().src_time) >
              params_.max_delta_t) {
        return true;
      }

      // If the number of tracks to merge is equal to the number of detectors,
      // then the last received track will necessarily belong to a new frame,
      // and it is time to compute the average track
      if (pair.second.size() == n_detectors_) {
        return true;
      }
    }
  }

  return false;
}

void hiros::skeletons::Merger::mergeSkeletons() {
  for (const auto& pair : skeletons_to_merge_) {
    computeAvgSkeleton(pair.first);
  }
}

void hiros::skeletons::Merger::publish() {
  merged_skeletons_.time = now().seconds();
  merged_skeletons_.frame = last_skeleton_group_.frame;

  pub_->publish(utils::toMsg(merged_skeletons_));

  prev_merged_skeletons_ = merged_skeletons_;
  merged_skeletons_ = {};
  skeletons_to_merge_.clear();
}

void hiros::skeletons::Merger::addSkeletonToBuffer() {
  for (const auto& track : last_skeleton_group_.skeletons) {
    skeletons_to_merge_[track.id].push_back(track);
  }
}

void hiros::skeletons::Merger::computeAvgSkeleton(const int& id) {
  auto& tracks_to_merge{skeletons_to_merge_.at(id)};

  removeFlippedTracks(tracks_to_merge);
  removeOutliers(tracks_to_merge);

  if (tracks_to_merge.empty()) {
    return;
  }

  auto& avg_track{tracks_to_merge.front()};
  for (auto track_idx{1u}; track_idx < tracks_to_merge.size(); ++track_idx) {
    utils::merge(avg_track, tracks_to_merge.at(track_idx), track_idx, 1, true);
  }

  cleanupLinks(avg_track);

  merged_skeletons_.addSkeleton(avg_track);
}

void hiros::skeletons::Merger::removeFlippedTracks(
    std::vector<hiros::skeletons::types::Skeleton>& tracks) {
  if (params_.pelvis_marker_id < 0) {
    return;
  }

  // If there is only one track it is not possible to understand if it is
  // flipped
  if (tracks.size() <= 1) {
    return;
  }

  std::vector<hiros::skeletons::types::KinematicState> states{};

  for (auto track_idx{0u}; track_idx < tracks.size(); ++track_idx) {
    if (tracks.at(track_idx).hasMarker(params_.pelvis_marker_id)) {
      states.push_back(
          tracks.at(track_idx).getMarker(params_.pelvis_marker_id).center);
    }
  }

  if (prev_merged_skeletons_.hasSkeleton(tracks.front().id) &&
      prev_merged_skeletons_.getSkeleton(tracks.front().id)
          .hasMarker(params_.pelvis_marker_id)) {
    states.push_back(prev_merged_skeletons_.getSkeleton(tracks.front().id)
                         .getMarker(params_.pelvis_marker_id)
                         .center);
  }

  auto indexes_to_keep{split(states, -1, M_PI / 2.)};

  auto idx{-1};
  tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                              [&](const auto& e) {
                                ++idx;
                                return (std::find(indexes_to_keep.begin(),
                                                  indexes_to_keep.end(), idx) ==
                                        indexes_to_keep.end());
                              }),
               tracks.end());
}

void hiros::skeletons::Merger::removeOutliers(
    std::vector<hiros::skeletons::types::Skeleton>& tracks) {
  if (params_.max_position_delta <= 0 && params_.max_orientation_delta <= 0) {
    return;
  }

  if (tracks.empty()) {
    return;
  }

  removeOutlierMarkers(tracks);
  removeOutlierLinks(tracks);
}

void hiros::skeletons::Merger::removeOutlierMarkers(
    std::vector<hiros::skeletons::types::Skeleton>& tracks) {
  std::vector<unsigned int> indexes_to_erase{};
  std::vector<hiros::skeletons::types::KinematicState> states{};

  for (auto mk_idx{0}; mk_idx < static_cast<int>(tracks.front().max_markers);
       ++mk_idx) {
    indexes_to_erase.clear();
    states.clear();

    for (auto track_idx{0u}; track_idx < tracks.size(); ++track_idx) {
      auto& track{tracks.at(track_idx)};

      if (track.hasMarker(mk_idx)) {
        indexes_to_erase.push_back(track_idx);
        states.push_back(track.getMarker(mk_idx).center);
      }
    }

    if (prev_merged_skeletons_.hasSkeleton(tracks.front().id) &&
        prev_merged_skeletons_.getSkeleton(tracks.front().id)
            .hasMarker(mk_idx)) {
      states.push_back(prev_merged_skeletons_.getSkeleton(tracks.front().id)
                           .getMarker(mk_idx)
                           .center);
    }

    auto res{split(states, params_.max_position_delta,
                   params_.max_orientation_delta)};

    indexes_to_erase.erase(
        std::remove_if(indexes_to_erase.begin(), indexes_to_erase.end(),
                       [&](const auto& e) {
                         return (std::find(res.begin(), res.end(), e) !=
                                 res.end());
                       }),
        indexes_to_erase.end());

    for (auto idx{0u}; idx < tracks.size(); ++idx) {
      if (std::find(indexes_to_erase.begin(), indexes_to_erase.end(), idx) !=
          indexes_to_erase.end()) {
        tracks.at(idx).removeMarker(mk_idx);
      }
    }
  }
}

void hiros::skeletons::Merger::removeOutlierLinks(
    std::vector<hiros::skeletons::types::Skeleton>& tracks) {
  std::vector<unsigned int> indexes_to_erase{};
  std::vector<hiros::skeletons::types::KinematicState> states{};

  for (auto lk_idx{0}; lk_idx < static_cast<int>(tracks.front().max_links);
       ++lk_idx) {
    indexes_to_erase.clear();
    states.clear();

    for (auto track_idx{0u}; track_idx < tracks.size(); ++track_idx) {
      auto& track{tracks.at(track_idx)};

      if (track.hasLink(lk_idx)) {
        indexes_to_erase.push_back(track_idx);
        states.push_back(track.getLink(lk_idx).center);
      }
    }

    if (prev_merged_skeletons_.hasSkeleton(tracks.front().id) &&
        prev_merged_skeletons_.getSkeleton(tracks.front().id).hasLink(lk_idx)) {
      states.push_back(prev_merged_skeletons_.getSkeleton(tracks.front().id)
                           .getLink(lk_idx)
                           .center);
    }

    auto res{split(states, params_.max_position_delta,
                   params_.max_orientation_delta)};

    indexes_to_erase.erase(
        std::remove_if(indexes_to_erase.begin(), indexes_to_erase.end(),
                       [&](const auto& e) {
                         return (std::find(res.begin(), res.end(), e) !=
                                 res.end());
                       }),
        indexes_to_erase.end());

    for (auto idx{0u}; idx < tracks.size(); ++idx) {
      if (std::find(indexes_to_erase.begin(), indexes_to_erase.end(), idx) !=
          indexes_to_erase.end()) {
        tracks.at(idx).removeLink(lk_idx);
      }
    }
  }
}

void hiros::skeletons::Merger::cleanupLinks(
    hiros::skeletons::types::Skeleton& skel) {
  skel.links.erase(std::remove_if(skel.links.begin(), skel.links.end(),
                                  [&](const auto& lk) {
                                    return (!skel.hasMarker(lk.parent_marker) ||
                                            !skel.hasMarker(lk.parent_marker));
                                  }),
                   skel.links.end());
}

std::vector<unsigned int> hiros::skeletons::Merger::split(
    const std::vector<hiros::skeletons::types::KinematicState>& states,
    const double& max_position_delta,
    const double& max_orientation_delta) const {
  if (states.empty()) {
    return {};
  }

  std::vector<unsigned int> missing_idxs(states.size());
  // Fill with 0, 1, ..., size(states) - 1
  std::iota(std::begin(missing_idxs), std::end(missing_idxs), 0);

  std::vector<std::set<unsigned int>> groups{};

  while (!missing_idxs.empty()) {
    // Initialize the group with the first unassigned kinematic state
    groups.push_back(std::set<unsigned int>{missing_idxs.front()});

    for (const auto& first_ks_idx : groups.back()) {
      for (const auto& ks_idx : missing_idxs) {
        // Fill the group with kinematic states close to the first one
        auto pos_dist{hiros::skeletons::utils::distance(
            states.at(first_ks_idx).pose.position,
            states.at(ks_idx).pose.position)};
        auto or_dist{hiros::skeletons::utils::distance(
            states.at(first_ks_idx).pose.orientation,
            states.at(ks_idx).pose.orientation)};

        if ((max_position_delta <= 0 || std::isnan(pos_dist) ||
             pos_dist < max_position_delta) &&
            (max_orientation_delta <= 0 || std::isnan(or_dist) ||
             or_dist < max_orientation_delta)) {
          groups.back().insert(ks_idx);
        }
      }
    }

    // Erase indexes that were assigned to the group from missing_idxs
    missing_idxs.erase(std::remove_if(missing_idxs.begin(), missing_idxs.end(),
                                      [&](const auto& e) {
                                        return (groups.back().count(e) > 0);
                                      }),
                       missing_idxs.end());
  }

  // If all the kinematic states belong to the same group, return the only group
  if (groups.size() == 1) {
    return std::vector<unsigned int>(groups.front().begin(),
                                     groups.front().end());
  }

  // Sort by size
  std::sort(
      std::begin(groups), std::end(groups),
      [](const auto& lhs, const auto& rhs) { return lhs.size() > rhs.size(); });

  // If the two largest groups have the same number of elements, return empty
  // set
  if (groups.at(0).size() == groups.at(1).size()) {
    return {};
  }

  // Return the group with the largest number of elements
  return std::vector<unsigned int>(groups.front().begin(),
                                   groups.front().end());
}

void hiros::skeletons::Merger::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  last_skeleton_group_ = hiros::skeletons::utils::toStruct(msg);
  estimateNumberOfDetectors();

  if (okToMerge()) {
    mergeSkeletons();
    publish();
  }

  addSkeletonToBuffer();
}
