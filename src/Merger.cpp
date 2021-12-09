// Custom External Packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_merger/Merger.h"
#include "skeleton_merger/utils.h"

void hiros::merge::Merger::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Merger... Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);
  m_nh.getParam("input_topic", m_params.input_topic);
  m_nh.getParam("output_topic", m_params.output_topic);

  if (m_params.input_topic.empty() || m_params.output_topic.empty()) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Merger Error: Required topics configuration not provided. Unable to continue");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  m_nh.getParam("n_detectors", m_params.n_detectors);
  m_nh.getParam("max_delta_t", m_params.max_delta_t);
  m_nh.getParam("max_position_delta", m_params.max_position_delta);
  m_nh.getParam("max_orientation_delta", m_params.max_orientation_delta);

  if (m_params.n_detectors > 0) {
    m_n_detectors = static_cast<unsigned long>(m_params.n_detectors);
    // If n_detectors <= 0, then m_n_detectors will be estimated online based on the skeletons src_frames
  }

  setupRosTopics();

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Merger... CONFIGURED" << BASH_MSG_RESET);
}

void hiros::merge::Merger::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Merger... Starting");

  if (!m_configured) {
    configure();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Merger... RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void hiros::merge::Merger::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Merger... Stopping");

  if (m_in_skeleton_group_sub) {
    m_in_skeleton_group_sub.shutdown();
  }

  if (m_out_skeleton_group_pub) {
    m_out_skeleton_group_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Merger... STOPPED" << BASH_MSG_RESET);
  ros::shutdown();
}

void hiros::merge::Merger::setupRosTopics()
{
  m_in_skeleton_group_sub = m_nh.subscribe(m_params.input_topic, 1, &Merger::callback, this);

  while (m_in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_DELAYED_THROTTLE(
      2, "Hi-ROS Skeleton Merger Warning: No input messages on skeleton group topic '" << m_params.input_topic << "'");
  }

  m_out_skeleton_group_pub = m_nh.advertise<hiros_skeleton_msgs::SkeletonGroup>(m_params.output_topic, 1);
}

void hiros::merge::Merger::callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  if (!ros::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  m_last_skeleton_group = hiros::skeletons::utils::toStruct(t_msg);
  estimateNumberOfDetectors();

  if (okToMerge()) {
    mergeSkeletons();
    publish();
  }

  addSkeletonToBuffer();
}

void hiros::merge::Merger::estimateNumberOfDetectors()
{
  if (m_params.n_detectors > 0) {
    return;
  }

  for (const auto& skel : m_last_skeleton_group.skeletons) {
    m_src_frames.insert(skel.src_frame);
  }

  m_n_detectors = m_src_frames.size();
}

bool hiros::merge::Merger::okToMerge() const
{
  for (auto& pair : m_skeletons_to_merge) {
    if (m_last_skeleton_group.hasSkeleton(pair.first)) {
      auto& last_track = m_last_skeleton_group.getSkeleton(pair.first);
      auto last_src_time = last_track.src_time;
      auto last_src_frame = last_track.src_frame;

      // If one of the tracks to merge has its src_frame equal to the one of the latest received track, and its src_time
      // is lower than the one of the last received track, then the last received track belongs to a new frame, and it
      // is time to compute the average track
      if (std::find_if(
            pair.second.begin(),
            pair.second.end(),
            [&](const auto& elem) { return (elem.src_frame == last_src_frame && elem.src_time < last_src_time); })
          != pair.second.end()) {
        return true;
      }

      // If the delta between the src_time of the last received track and the oldest track to merge is greater than
      // max_delta_t, then the track belongs to a new frame, and it is time to compute the average track
      if (m_params.max_delta_t > 0 && !pair.second.empty()
          && (last_src_time - pair.second.front().src_time) > m_params.max_delta_t) {
        return true;
      }

      // If the number of tracks to merge is equal to the number of detectors, then the last received track will
      // necessarily belong to a new frame, and it is time to compute the average track
      if (pair.second.size() == m_n_detectors) {
        return true;
      }
    }
  }

  return false;
}

void hiros::merge::Merger::mergeSkeletons()
{
  for (auto& pair : m_skeletons_to_merge) {
    computeAvgSkeleton(pair.first);
  }
}

void hiros::merge::Merger::publish()
{
  m_merged_skeletons.time = ros::Time::now().toSec();
  m_merged_skeletons.frame = m_last_skeleton_group.frame;

  m_out_skeleton_group_pub.publish(hiros::skeletons::utils::toMsg(m_merged_skeletons));

  m_prev_merged_skeletons = m_merged_skeletons;
  m_merged_skeletons = {};
  m_skeletons_to_merge.clear();
}

void hiros::merge::Merger::addSkeletonToBuffer()
{
  for (const auto& track : m_last_skeleton_group.skeletons) {
    m_skeletons_to_merge[track.id].push_back(track);
  }
}

void hiros::merge::Merger::computeAvgSkeleton(const int& t_id)
{
  auto& tracks_to_merge = m_skeletons_to_merge.at(t_id);

  removeOutliers(tracks_to_merge);

  if (tracks_to_merge.empty()) {
    return;
  }

  auto& avg_track = tracks_to_merge.front();
  for (unsigned int track_idx = 1; track_idx < tracks_to_merge.size(); ++track_idx) {
    utils::merge(avg_track, tracks_to_merge.at(track_idx), track_idx, 1, true);
  }

  m_merged_skeletons.addSkeleton(avg_track);
}

void hiros::merge::Merger::removeOutliers(std::vector<hiros::skeletons::types::Skeleton>& t_tracks)
{
  if (m_params.max_position_delta <= 0 && m_params.max_orientation_delta <= 0) {
    return;
  }

  if (t_tracks.empty()) {
    return;
  }

  removeOutlierMarkers(t_tracks);
  removeOutlierLinks(t_tracks);
}

void hiros::merge::Merger::removeOutlierMarkers(std::vector<hiros::skeletons::types::Skeleton>& t_tracks)
{
  std::vector<unsigned int> indexes_to_erase;
  std::vector<hiros::skeletons::types::KinematicState> states;

  for (int mk_idx = 0; mk_idx < static_cast<int>(t_tracks.front().max_markers); ++mk_idx) {
    indexes_to_erase.clear();
    states.clear();

    for (unsigned int track_idx = 0; track_idx < t_tracks.size(); ++track_idx) {
      auto& track = t_tracks.at(track_idx);

      if (track.hasMarker(mk_idx)) {
        indexes_to_erase.push_back(track_idx);
        states.push_back(track.getMarker(mk_idx).center);
      }
    }

    if (m_prev_merged_skeletons.hasSkeleton(t_tracks.front().id)
        && m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).hasMarker(mk_idx)) {
      // In this way the previous track has double weight for the outliers detection
      states.push_back(m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).getMarker(mk_idx).center);
      states.push_back(m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).getMarker(mk_idx).center);
    }

    auto res = utils::split(states, m_params.max_position_delta, m_params.max_orientation_delta);

    indexes_to_erase.erase(
      std::remove_if(indexes_to_erase.begin(),
                     indexes_to_erase.end(),
                     [&](const int& e) { return (std::find(res.begin(), res.end(), e) != res.end()); }),
      indexes_to_erase.end());

    for (unsigned int idx = 0; idx < t_tracks.size(); ++idx) {
      if (std::find(indexes_to_erase.begin(), indexes_to_erase.end(), idx) != indexes_to_erase.end()) {
        t_tracks.at(idx).removeMarker(mk_idx);
      }
    }
  }
}

void hiros::merge::Merger::removeOutlierLinks(std::vector<hiros::skeletons::types::Skeleton>& t_tracks)
{
  std::vector<unsigned int> indexes_to_erase;
  std::vector<hiros::skeletons::types::KinematicState> states;

  for (int lk_idx = 0; lk_idx < static_cast<int>(t_tracks.front().max_links); ++lk_idx) {
    indexes_to_erase.clear();
    states.clear();

    for (unsigned int track_idx = 0; track_idx < t_tracks.size(); ++track_idx) {
      auto& track = t_tracks.at(track_idx);

      if (track.hasLink(lk_idx)) {
        indexes_to_erase.push_back(track_idx);
        states.push_back(track.getLink(lk_idx).center);
      }
    }

    if (m_prev_merged_skeletons.hasSkeleton(t_tracks.front().id)
        && m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).hasLink(lk_idx)) {
      // In this way the previous track has double weight for the outliers detection
      states.push_back(m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).getLink(lk_idx).center);
      states.push_back(m_prev_merged_skeletons.getSkeleton(t_tracks.front().id).getLink(lk_idx).center);
    }

    auto res = utils::split(states, m_params.max_position_delta, m_params.max_orientation_delta);

    indexes_to_erase.erase(
      std::remove_if(indexes_to_erase.begin(),
                     indexes_to_erase.end(),
                     [&](const int& e) { return (std::find(res.begin(), res.end(), e) != res.end()); }),
      indexes_to_erase.end());

    for (unsigned int idx = 0; idx < t_tracks.size(); ++idx) {
      if (std::find(indexes_to_erase.begin(), indexes_to_erase.end(), idx) != indexes_to_erase.end()) {
        t_tracks.at(idx).removeLink(lk_idx);
      }
    }
  }
}
