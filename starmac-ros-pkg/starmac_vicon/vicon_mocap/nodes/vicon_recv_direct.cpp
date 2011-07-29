/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <Client.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon_mocap/viconGrabPose.h>
#include <iostream>
#include <algorithm>

#include <vicon_mocap/Markers.h>
#include <vicon_mocap/Marker.h>

using std::min;
using std::max;
using std::string;

using namespace ViconDataStreamSDK::CPP;

string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

class ViconReceiver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  double min_freq;
  double max_freq;
  diagnostic_updater::FrequencyStatus freq_status;
  // Parameters:
  string StreamMode;
  string HostName;
  string SubjectName;
  string SegmentName;
  string tf_ref_frame_id;
  string tf_tracked_frame_id;
  double update_rate;
  double vicon_capture_rate;
  // Publisher
  ros::Publisher pose_pub;
  ros::Publisher markers_pub;
  // Timer
  ros::Timer updateTimer;
  // TF Broadcaster
  tf::TransformBroadcaster tf_broadcast;
  //geometry_msgs::PoseStamped vicon_pose;
  tf::Transform flyer_transform;
  ros::Time now_time;
  // TODO: Make the following configurable:
  ros::ServiceServer m_grab_vicon_pose_service_server;
  ViconDataStreamSDK::CPP::Client MyClient;
  double max_period_between_updates;
  double latest_dt;
  double latest_jitter;
  double max_abs_jitter;
  double last_callback_duration;
  unsigned int lastFrameNumber;
  unsigned int frameCount;
  unsigned int droppedFrameCount;
  ros::Time time_datum;
  unsigned int frame_datum;
  double latest_time_bias;
  unsigned int time_bias_reset_count;
  unsigned int n_markers;
  unsigned int n_unlabeled_markers;
  double time_bias_reset;
  bool segment_data_enabled;
  bool marker_data_enabled;
  bool unlabeled_marker_data_enabled;
  bool enable_tf_broadcast;

public:
  ViconReceiver() :
    nh_priv("~"), diag_updater(), min_freq(0.1), max_freq(1000),
        freq_status(diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq)), StreamMode("ClientPullPreFetch"),
        HostName(""), SubjectName(""), SegmentName(""), tf_ref_frame_id("/enu"),
        tf_tracked_frame_id("/pelican1/flyer_vicon"), update_rate(100), vicon_capture_rate(50),
        max_period_between_updates(0), max_abs_jitter(0), lastFrameNumber(0), frameCount(0), droppedFrameCount(0),
        frame_datum(0), latest_time_bias(0), time_bias_reset_count(0), n_markers(0), n_unlabeled_markers(0),
        time_bias_reset(0.05), segment_data_enabled(false), marker_data_enabled(false),
        unlabeled_marker_data_enabled(false), enable_tf_broadcast(false)

  {
    // Diagnostics
    diag_updater.add("ViconReceiver Status", this, &ViconReceiver::diagnostics);
    diag_updater.add(freq_status);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("stream_mode", StreamMode, StreamMode);
    nh_priv.param("datastream_hostport", HostName, HostName);
    nh_priv.param("subject_name", SubjectName, SubjectName);
    nh_priv.param("segment_name", SegmentName, SegmentName);
    nh_priv.param("update_rate", update_rate, update_rate);
    nh_priv.param("vicon_capture_rate", vicon_capture_rate, vicon_capture_rate);
    nh_priv.param("tf_ref_frame_id", tf_ref_frame_id, tf_ref_frame_id);
    nh_priv.param("tf_tracked_frame_id", tf_tracked_frame_id, tf_tracked_frame_id);
    nh_priv.param("time_bias_reset", time_bias_reset, time_bias_reset);
    nh_priv.param("enable_tf_broadcast", enable_tf_broadcast, enable_tf_broadcast);
    ROS_ASSERT(init_vicon());
    // Service Server
    ROS_INFO("setting up grab_vicon_pose service server ... ");
    m_grab_vicon_pose_service_server = nh_priv.advertiseService("grab_vicon_pose",
                                                                &ViconReceiver::grab_vicon_pose_callback, this);
    // Publishers
    pose_pub = nh_priv.advertise<geometry_msgs::TransformStamped> ("output", 10);
    markers_pub = nh_priv.advertise<vicon_mocap::Markers> ("markers", 10);
    // Timer
    double update_timer_period = 1 / update_rate;
    min_freq = 0.95 * vicon_capture_rate;
    max_freq = 1.05 * vicon_capture_rate;
    updateTimer = nh.createTimer(ros::Duration(update_timer_period), &ViconReceiver::updateCallback, this);
  }

  ~ViconReceiver()
  {
    ROS_ASSERT(shutdown_vicon());
  }

private:
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    stat.add("max dt", max_period_between_updates);
    stat.add("latest dt", latest_dt);
    stat.add("latest jitter", latest_jitter);
    stat.add("max abs jitter", max_abs_jitter);
    stat.add("latest callback runtime", last_callback_duration);
    stat.add("latest VICON frame number", lastFrameNumber);
    stat.add("dropped frames", droppedFrameCount);
    stat.add("framecount", frameCount);
    stat.add("latest time bias", latest_time_bias);
    stat.add("time bias reset count", time_bias_reset_count);
    stat.add("# markers", n_markers);
    stat.add("# unlabeled markers", n_unlabeled_markers);
  }

  bool init_vicon()
  {
    ROS_INFO_STREAM("Connecting to Vicon DataStream SDK at " << HostName << " ...");

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(HostName);
      ROS_INFO(".");
      sleep(1);
      ros::spinOnce();
      if (!ros::ok())
        return false;
    }
    ROS_ASSERT(MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... connected!");

    ROS_INFO_STREAM("Setting Stream Mode to " << StreamMode);
    {
      Output_SetStreamMode result;

      if (StreamMode == "ServerPush")
      {
        result = MyClient.SetStreamMode(StreamMode::ServerPush);
      }
      else if (StreamMode == "ClientPull")
      {
        result = MyClient.SetStreamMode(StreamMode::ClientPull);
      }
      else if (StreamMode == "ClientPullPreFetch")
      {
        result = MyClient.SetStreamMode(StreamMode::ClientPullPreFetch);
      }
      else
      {
        ROS_FATAL("Unknown stream mode -- options are ServerPush, ClientPull, ClientPullPreFetch");
        ros::shutdown();
      }

      if (result.Result != Result::Success)
      {
        ROS_FATAL("Set stream mode call failed -- shutting down");
        ros::shutdown();
      }
    }
    {
      Output_SetAxisMapping result;
      result = MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up); // 'Z-up'
      if (result.Result != Result::Success)
      {
        ROS_FATAL("Set axis mapping call failed -- shutting down");
        ros::shutdown();
      }
      Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
      ROS_INFO_STREAM("Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
          << Adapt(_Output_GetAxisMapping.YAxis) << " Z-" << Adapt(_Output_GetAxisMapping.ZAxis));
      Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
      ROS_INFO_STREAM("Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
          << _Output_GetVersion.Point);
    }
    return true;
  }

  void updateCallback(const ros::TimerEvent& e)
  {
    static bool got_first = false;
    latest_dt = (e.current_real - e.last_real).toSec();
    latest_jitter = (e.current_real - e.current_expected).toSec();
    max_abs_jitter = max(max_abs_jitter, fabs(latest_jitter));
    Result::Enum the_result;
    bool was_new_frame = true;

    if ((not segment_data_enabled) and pose_pub.getNumSubscribers() > 0)
    {
      the_result = MyClient.EnableSegmentData().Result;
      if (the_result != Result::Success)
      {
        ROS_ERROR("Enable segment data call failed");
      }
      else
      {
        ROS_ASSERT(MyClient.IsSegmentDataEnabled().Enabled);
        ROS_INFO("Segment data enabled");
        segment_data_enabled = true;
      }
    }

    if (segment_data_enabled)
    {
      while (was_new_frame and (the_result = MyClient.GetFrame().Result) == Result::Success)
        ;
      {
        now_time = ros::Time::now(); // try to grab as close to getting message as possible
        was_new_frame = process_frame();
        got_first = true;
      }

      if (the_result != Result::NoFrame)
      {
        ROS_ERROR_STREAM("GetFrame() returned " << (int)the_result);
      }

      if (got_first)
      {
        max_period_between_updates = max(max_period_between_updates, latest_dt);
        last_callback_duration = e.profile.last_duration.toSec();
      }
    }
    diag_updater.update();
  }

  bool shutdown_vicon()
  {
    ROS_INFO_STREAM("Disconnecting from Vicon DataStream SDK");
    MyClient.Disconnect();
    ROS_ASSERT(!MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... disconnected.");
    return true;
  }

  bool process_frame()
  {
    static ros::Time lastTime;
    Output_GetFrameNumber OutputFrameNum = MyClient.GetFrameNumber();
    if ((OutputFrameNum.Result != Result::Success) and (OutputFrameNum.Result != Result::NoFrame))
    {
      ROS_ERROR("GetFrameNumber() returned %d", int(OutputFrameNum.Result));
      return false;
    }
    if (frame_datum == 0 or (fabs(latest_time_bias) > time_bias_reset)) // this is the first frame we've gotten
    {
      frame_datum = OutputFrameNum.FrameNumber;
      time_datum = now_time;
      ROS_INFO("Time bias reset: marking VICON frame number %d as datum time %d.%d", frame_datum, time_datum.sec,
               time_datum.nsec);
      time_bias_reset_count++;
    }

    //frameCount++;
//    ROS_INFO_STREAM("Grabbed a frame: " << OutputFrameNum.FrameNumber);
    int frameDiff = 0;
    if (lastFrameNumber != 0)
    {
      frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber;
      frameCount += frameDiff;
      if ((frameDiff) > 1)
      {
        droppedFrameCount += frameDiff - 1;
        double droppedFramePct = (double)droppedFrameCount / frameCount * 100;
        ROS_DEBUG_STREAM((frameDiff - 1) << " more (total " << droppedFrameCount << "/" << frameCount << ", "
            << droppedFramePct << "%) frame(s) dropped. Consider adjusting rates.");
      }
    }
    lastFrameNumber = OutputFrameNum.FrameNumber;

    if (frameDiff == 0)
    {
      return false;
    }
    else
    {
      freq_status.tick();
      //double latencyInMs = MyClient.GetLatencyTotal().Total * 1000;
      ros::Time frame_time = time_datum + ros::Duration((lastFrameNumber - frame_datum) / vicon_capture_rate);
      latest_time_bias = (frame_time - now_time).toSec();

      process_subjects(frame_time);
      process_markers(frame_time, lastFrameNumber);

      lastTime = now_time;
      return true;
    }
  }

  void process_subjects(const ros::Time& frame_time)
  {
    if (pose_pub.getNumSubscribers() > 0)
    {

      // We know the subject and segment names a priori, so don't bother enumerating, just grab the data:
      // Flyer:
      Output_GetSegmentGlobalTranslation trans = MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
      Output_GetSegmentGlobalRotationQuaternion quat = MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                                                                   SegmentName);
      if (trans.Result == Result::Success and quat.Result == Result::Success)
      {
        if ((not trans.Occluded) and (not quat.Occluded))
        {
          flyer_transform.setOrigin(tf::Vector3(trans.Translation[0] / 1000, trans.Translation[1] / 1000,
                                                trans.Translation[2] / 1000));
          flyer_transform.setRotation(tf::Quaternion(quat.Rotation[0], quat.Rotation[1], quat.Rotation[2],
                                                     quat.Rotation[3]));
          //ros::Time thisTime = now_time - ros::Duration(latencyInMs / 1000);
          tf::StampedTransform transform = tf::StampedTransform(flyer_transform, frame_time, tf_ref_frame_id,
                                                                tf_tracked_frame_id);
          if (enable_tf_broadcast)
            tf_broadcast.sendTransform(transform);

          geometry_msgs::TransformStamped pose_msg;
          tf::transformStampedTFToMsg(transform, pose_msg);
          pose_pub.publish(pose_msg);

        }
        else
        {
          ROS_WARN_STREAM("occluded, not publishing...");
        }
      }
      else
      {
        ROS_WARN("GetSegmentGlobalTranslation/Rotation failed (result = %d, %d), not publishing...",
                 (int)(trans.Result), (int)(quat.Result));
      }
    }
  }

  void process_markers(const ros::Time& frame_time, unsigned int vicon_frame_num)
  {
    if (markers_pub.getNumSubscribers() > 0)
    {
      if (not marker_data_enabled)
      {
        MyClient.EnableMarkerData();
        ROS_ASSERT(MyClient.IsMarkerDataEnabled().Enabled);
        marker_data_enabled = true;
      }
      if (not unlabeled_marker_data_enabled)
      {
        MyClient.EnableUnlabeledMarkerData();
        ROS_ASSERT(MyClient.IsUnlabeledMarkerDataEnabled().Enabled);
        unlabeled_marker_data_enabled = true;
      }
      n_markers = 0;
      vicon_mocap::Markers markers_msg;
      markers_msg.header.stamp = frame_time;
      markers_msg.frame_number = vicon_frame_num;
      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // Get labeled markers
      for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
      {
        std::string this_subject_name = MyClient.GetSubjectName(SubjectIndex).SubjectName;
        // Count the number of markers
        unsigned int num_subject_markers = MyClient.GetMarkerCount(SubjectName).MarkerCount;
        n_markers += num_subject_markers;
        //std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
        for (unsigned int MarkerIndex = 0; MarkerIndex < num_subject_markers; ++MarkerIndex)
        {
          vicon_mocap::Marker this_marker;
          this_marker.marker_name = MyClient.GetMarkerName(this_subject_name, MarkerIndex).MarkerName;
          this_marker.subject_name = this_subject_name;
          this_marker.segment_name
              = MyClient.GetMarkerParentName(this_subject_name, this_marker.marker_name).SegmentName;

          // Get the global marker translation
          Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
              MyClient.GetMarkerGlobalTranslation(this_subject_name, this_marker.marker_name);

          this_marker.translation.x = _Output_GetMarkerGlobalTranslation.Translation[0];
          this_marker.translation.y = _Output_GetMarkerGlobalTranslation.Translation[1];
          this_marker.translation.z = _Output_GetMarkerGlobalTranslation.Translation[2];
          this_marker.occluded = _Output_GetMarkerGlobalTranslation.Occluded;

          markers_msg.markers.push_back(this_marker);
        }
      }
      // get unlabeled markers
      unsigned int UnlabeledMarkerCount = MyClient.GetUnlabeledMarkerCount().MarkerCount;
      //ROS_INFO("# unlabeled markers: %d", UnlabeledMarkerCount);
      n_markers += UnlabeledMarkerCount;
      n_unlabeled_markers = UnlabeledMarkerCount;
      for (unsigned int UnlabeledMarkerIndex = 0; UnlabeledMarkerIndex < UnlabeledMarkerCount; ++UnlabeledMarkerIndex)
      {
        // Get the global marker translation
        Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
            MyClient.GetUnlabeledMarkerGlobalTranslation(UnlabeledMarkerIndex);

        if (_Output_GetUnlabeledMarkerGlobalTranslation.Result == Result::Success)
        {
          vicon_mocap::Marker this_marker;
          this_marker.translation.x = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0];
          this_marker.translation.y = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1];
          this_marker.translation.z = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2];
          this_marker.occluded = false; // unlabeled markers can't be occluded
          markers_msg.markers.push_back(this_marker);
        }
        else
        {
          ROS_WARN("GetUnlabeledMarkerGlobalTranslation failed (result = %d)",
                   (int)(_Output_GetUnlabeledMarkerGlobalTranslation.Result));

        }
      }
      markers_pub.publish(markers_msg);
    }
  }

  bool grab_vicon_pose_callback(vicon_mocap::viconGrabPose::Request& req, vicon_mocap::viconGrabPose::Response& resp)
  {
    ROS_INFO("Got request for a VICON pose");
    updateTimer.stop();
    tf::StampedTransform transform;
    //tf::Quaternion quat;

    if (not segment_data_enabled)
    {
      MyClient.EnableSegmentData();
      ROS_ASSERT(MyClient.IsSegmentDataEnabled().Enabled);
      segment_data_enabled = true;
    }

    // Gather data:
    int N = req.n_measurements;
    double accum_trans[3] = {0, 0, 0};
    double accum_quat[4] = {0, 0, 0, 0};
    Result::Enum the_result;
    int n_success = 0;
    for (int k = 0; k < N; k++)
    {
      ros::Duration(1 / vicon_capture_rate).sleep(); // Wait at least as long as vicon needs to capture the next frame
      if ((the_result = MyClient.GetFrame().Result) == Result::Success)
      {
        try
        {
          Output_GetSegmentGlobalTranslation trans = MyClient.GetSegmentGlobalTranslation(req.subject_name,
                                                                                          req.segment_name);
          Output_GetSegmentGlobalRotationQuaternion quat =
              MyClient.GetSegmentGlobalRotationQuaternion(req.subject_name, req.segment_name);
          if ((!trans.Occluded) && (!quat.Occluded))
          {
            accum_trans[0] += trans.Translation[0] / 1000;
            accum_trans[1] += trans.Translation[1] / 1000;
            accum_trans[2] += trans.Translation[2] / 1000;
            accum_quat[0] += quat.Rotation[3];
            accum_quat[1] += quat.Rotation[0];
            accum_quat[2] += quat.Rotation[1];
            accum_quat[3] += quat.Rotation[2];
            n_success++;
          }
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          resp.success = false;
          return false; // TODO: should we really bail here, or just try again?
        }
      }
      else
      {
        if (the_result != Result::NoFrame)
        {
          ROS_ERROR_STREAM("GetFrame() returned " << (int)the_result);
        }
      }
    }

    ROS_INFO("Averaging %d measurements", n_success);
    // Average the data:
    double normalized_quat[4];
    double quat_norm = sqrt(accum_quat[0] * accum_quat[0] + accum_quat[1] * accum_quat[1] + accum_quat[2]
        * accum_quat[2] + accum_quat[3] * accum_quat[3]);
    for (int i = 0; i < 4; i++)
    {
      normalized_quat[i] = accum_quat[i] / quat_norm;
    }
    double normalized_vector[3];
    // Copy to inputs:
    for (int i = 0; i < 3; i++)
    {
      normalized_vector[i] = accum_trans[i] / n_success;
    }

    // copy what we used to service call response:
    resp.success = true;
    resp.pose.header.stamp = ros::Time::now();
    resp.pose.header.frame_id = tf_ref_frame_id;
    resp.pose.pose.position.x = normalized_vector[0];
    resp.pose.pose.position.y = normalized_vector[1];
    resp.pose.pose.position.z = normalized_vector[2];
    resp.pose.pose.orientation.w = normalized_quat[0];
    resp.pose.pose.orientation.x = normalized_quat[1];
    resp.pose.pose.orientation.y = normalized_quat[2];
    resp.pose.pose.orientation.z = normalized_quat[3];

    updateTimer.start();
    return true;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_recv_direct");
  ViconReceiver vr;
  ros::spin();
  return 0;
}
