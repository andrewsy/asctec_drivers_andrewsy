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
#include "flyer_controller/control_mode.h"
#include "flyer_controller/hover_modes.h"
// msg:
//#include "flyer_controller/control_mode_hover_info.h"
#include "flyer_controller/control_mode_autosequence_info.h"
#include "flyer_controller/Autosequence.h"
#include "flyer_controller/GetAutosequence.h"

namespace flyer_controller
{

const int BTN_PROCEED = 8;
const int BTN_PAUSE = 7;

struct autosequence_point
{
  hover_point point;
  bool pause;
};

typedef vector<autosequence_point> autosequence;

namespace AutosequenceTypes
{
enum WaypointUpdateStates
{
  WAITING_PROCEED = 0, MOVING = 1, PAUSED = 2, IDLE = 3
};
typedef WaypointUpdateStates WaypointUpdateState;
}

class ControlModeAutosequence : public HoverMode
{
private:
  // Parameters
  double waypoint_update_rate; // [Hz] rate at which to update the controller setpoint
  double reached_tolerance; // [m] when within this lateral distance of an autosequence point, consider that point to be 'reached'
  double reached_tolerance_yaw; // [deg] when within this angular distance of an autosequence point's yaw value (if defined), consider that point to be 'reached'
  double waypoint_speed; // [m/s] lateral speed at which to move the controller setpoint between autosequence points
  double max_waypoint_lead; // [m] how far from the current position can the moving waypoint be (per axis)? (NOT IMPLEMENTED)
  // Publishers
  ros::Publisher autosequence_info_pub;
  // Services
  ros::ServiceServer get_autosequence_srv;
  // Timers
  ros::Timer waypoint_update_timer;
  // Members
  bool executing;
  string current_autosequence;
  unsigned int current_point;
  AutosequenceTypes::WaypointUpdateState wp_update_state;
  hover_point segment_start;
  hover_point segment_end;
  bool proceed_commanded;
  bool pause_commanded;
  bool cancel_commanded;
  control_mode_autosequence_info autosequence_info_msg;

public:
  ControlModeAutosequence() :
    HoverMode("autosequence"), waypoint_update_rate(10), reached_tolerance(0.05), reached_tolerance_yaw(5.0),
        waypoint_speed(0.05), max_waypoint_lead(0.1), executing(false), current_autosequence(""), current_point(0),
        wp_update_state(AutosequenceTypes::IDLE), proceed_commanded(false), pause_commanded(false),
        cancel_commanded(false)
  {

  }

  void onInit()
  {
    HoverMode::onInit();
    nh_priv.param("waypoint_update_rate", waypoint_update_rate, waypoint_update_rate);
    nh_priv.param("reached_tolerance", reached_tolerance, reached_tolerance);
    nh_priv.param("reached_tolerance_yaw", reached_tolerance_yaw, reached_tolerance_yaw);
    nh_priv.param("waypoint_speed", waypoint_speed, waypoint_speed);
    nh_priv.param("max_waypoint_lead", max_waypoint_lead, max_waypoint_lead);

    autosequence_info_pub = nh_priv.advertise<control_mode_autosequence_info> ("autosequence_info", 10);
    get_autosequence_srv = nh_priv.advertiseService("get_autosequence",
                                                    &ControlModeAutosequence::get_autosequence_callback, this);
  }

private:
  map<string, autosequence> autosequences;

  bool autosequence_exists(const string autosequence_name)
  {
    map<string, autosequence>::iterator it;
    it = autosequences.find(autosequence_name);
    return (not (it == autosequences.end()));
  }

  /* Autosequence control_mode_cmd syntax:
   Define a new autosequence, made up of the named hover_points (which should have been
   previously defined with set hover_point commands):
   define autosequence my_autosequence point_name1 [pause] [,point_name2 [pause]] ... [,point_nameN]

   Set commanded altitude to 1.23 (can only be lower than joystick altitude):
   alt_override 1.23

   Shorthand to define a single-point autosequence and execute it: (NOT IMPLEMENTED!)
   goto 1.23 -0.5 45 // north east [yaw]

   Execute autosequence:
   execute autosequence foo

   Proceed with autosequence:
   proceed

   Cancel autosequence:
   cancel

   */
  bool parseControlModeCmdDerived(const string cmd)
  {
    vector < string > words;
    boost::split(words, cmd, boost::is_any_of(" \t\n,"), boost::token_compress_on);
    int nw = words.size();

    if (words[0] == "define" and words[1] == "autosequence")
    {
      string autosequence_name = words[2];
      if (autosequence_exists(autosequence_name))
      {
        NODELET_WARN_STREAM("Autosequence '" << autosequence_name << "' exists and will be redefined.");
      }
      int i = 3;
      autosequence new_autosequence;
      bool fail = false;
      while ((i < nw) and not fail)
      {
        bool pause = false;
        if ((i < nw - 1) and words[i + 1] == "pause")
        {
          pause = true;
        }
        autosequence_point new_autosequence_point;
        if (hover_point_exists(words[i]))
        {
          NODELET_DEBUG_STREAM("Adding point " << words[i] << " to autosequence");
          new_autosequence_point.point = hover_points[words[i]];
          new_autosequence_point.pause = pause;
          new_autosequence.push_back(new_autosequence_point);
          if (pause)
            i += 2;
          else
            i++;
        }
        else
        {
          NODELET_ERROR_STREAM("Unknown hover point '" << words[i] << "'.");
          fail = true;
        }
      }
      if (!fail)
      {
        autosequences[autosequence_name] = new_autosequence;
        NODELET_INFO_STREAM("Completed autosequence definition for '" << autosequence_name << "' with "
            << new_autosequence.size() << " points");
        //        publish_autosequence_trajectory_viz(new_autosequence);
      }
      else
      {
        NODELET_ERROR("Autosequence definition failed");
      }

    }
    else if (words[0] == "execute" and words[1] == "autosequence")
    {
      if (nw == 3)
      {
        if (autosequence_exists(words[2]))
        {
          if (not executing)
          {
            start_autosequence(words[2]);
          }
          else
          {
            NODELET_ERROR("Autosequence already executing");
          }
        }
        else
        {
          NODELET_ERROR_STREAM("Unknown autosequence '" << words[2] << "'");
        }
      }
      else
      {
        NODELET_ERROR("Invalid command");
      }
    }
    else if (words[0] == "proceed")
    {
      if (executing and ((wp_update_state == AutosequenceTypes::WAITING_PROCEED) or (wp_update_state
          == AutosequenceTypes::PAUSED)))
      {
        proceed_commanded = true;
      }
      else
      {
        NODELET_ERROR("Invalid command");
      }
    }
    else if (words[0] == "pause")
    {
      if (executing and (wp_update_state == AutosequenceTypes::MOVING))
      {
        pause_commanded = true;
      }
      else
      {
        NODELET_ERROR("Invalid command");
      }

    }
    else if (words[0] == "cancel")
    {
      if (executing)
      {
        NODELET_WARN_STREAM("Cancelling autosequence by operator request");
        cancel_commanded = true;
      }
      else
      {
        NODELET_ERROR("Invalid command");
      }
    }
    else
    {
      return false;
    }
    return true;
  }

  void start_autosequence(const string& autosequence_name)
  {
    NODELET_INFO("Starting autosequence - press PROCEED on joystick");
    //    publish_autosequence_trajectory_viz(autosequences[autosequence_name]);
    executing = true;
    wp_update_state = AutosequenceTypes::WAITING_PROCEED;
    current_autosequence = autosequence_name;
    current_point = 0;
    //set_hover_point(autosequences[autosequence_name][current_point].point);
    segment_start.north = latest_state.pose.pose.position.x;
    segment_start.east = latest_state.pose.pose.position.y;
    segment_end = autosequences[autosequence_name][current_point].point;
    //    publish_current_destination_marker(segment_end);
    NODELET_INFO_STREAM("Destination: " << segment_end.north << " " << segment_end.east << " " << segment_end.yaw);
    waypoint_update_timer = nh.createTimer(ros::Duration(1 / waypoint_update_rate),
                                           &ControlModeAutosequence::updateWaypointCallback, this);
  }

  void cancel_autosequence()
  {
    executing = false;
    cancel_commanded = false;
    wp_update_state = AutosequenceTypes::IDLE;
    current_autosequence = "";
    current_point = 0;
    waypoint_update_timer.stop();
  }

  void proceed_autosequence()
  {
    NODELET_INFO("Proceeding");
    proceed_commanded = false;
    wp_update_state = AutosequenceTypes::MOVING;
    NODELET_INFO_STREAM("Destination: " << segment_end.north << " " << segment_end.east << " " << segment_end.yaw);
  }

  void pause_autosequence()
  {
    NODELET_INFO("Pausing (operator commanded)");
    wp_update_state = AutosequenceTypes::PAUSED;
    pause_commanded = false;
  }

  void complete_autosequence()
  {
    NODELET_INFO("Autosequence complete");
    cancel_autosequence();
  }

  void updateWaypointCallback(const ros::TimerEvent& event)
  {
    static joy::Joy prev_joy;
    if (cancel_commanded)
    {
      cancel_autosequence();
    }
    else
    {
      switch (wp_update_state)
      {
        case AutosequenceTypes::IDLE:
          break;
        case AutosequenceTypes::PAUSED:
          if (proceed_commanded or ((latest_joy.buttons[BTN_PROCEED] == 1) and (prev_joy.buttons[BTN_PROCEED] == 0)))
          {
            proceed_autosequence();
          }
          break;
        case AutosequenceTypes::WAITING_PROCEED:
          if (proceed_commanded or ((latest_joy.buttons[BTN_PROCEED] == 1) and (prev_joy.buttons[BTN_PROCEED] == 0)))
          {
            proceed_autosequence();
          }
          break;
        case AutosequenceTypes::MOVING:
          if (pause_commanded or ((latest_joy.buttons[BTN_PAUSE] == 1) and (prev_joy.buttons[BTN_PAUSE] == 0)))
          {
            pause_autosequence();
          }
          else
          {
            if (reached_point(segment_end))
            {
              do_reached_point();
            }
            else
            {
              hover_point new_point;
              new_point.north_vel = 0;
              new_point.east_vel = 0;
              new_point.name = "_autosequence_waypoint";
              calc_waypoint_location(new_point, (event.current_real - event.last_real).toSec());
              set_hover_point(new_point, false);
            }
          }
          break;
      }
    }

    prev_joy = latest_joy;
  }

  void do_reached_point()
  {
    NODELET_DEBUG_STREAM("Reached " << segment_end.name);
    set_hover_point(segment_end, false);
    if (current_point < autosequences[current_autosequence].size() - 1)
    {
      //set_hover_point(autosequences[current_autosequence][current_point + 1].point);
      segment_start = segment_end;
      segment_end = autosequences[current_autosequence][current_point + 1].point;
      //                publish_current_destination_marker(segment_end);
      if (autosequences[current_autosequence][current_point].pause)
      {
        wp_update_state = AutosequenceTypes::PAUSED;
        NODELET_INFO("Pausing (per autosequence definition) - press PROCEED button");
        NODELET_INFO_STREAM("Destination: " << segment_end.north << " " << segment_end.east << " " << segment_end.yaw);
      }
      else
      {
        NODELET_DEBUG_STREAM("Destination: " << segment_end.north << " " << segment_end.east << " " << segment_end.yaw);
        // stay in MOVING
      }
      current_point++;
    }
    else
    {
      complete_autosequence();
    }

  }

  bool reached_point(const hover_point& point)
  {
    double north_err;
    double east_err;
    double yaw_err;
    get_current_error_to_point(point, north_err, east_err, yaw_err);
    double error_norm = sqrt(north_err * north_err + east_err * east_err);
    bool reached = (error_norm <= reached_tolerance and (fabs(angles::to_degrees(yaw_err)) < reached_tolerance_yaw));
    //NODELET_INFO_STREAM("Reached: " << (reached ? "true" : "false") << " Error_norm = " << error_norm << " Yaw_err = " << yaw_err);
    return reached;
  }

  double norm2(double x1, double x2)
  {
    return sqrt(x1 * x1 + x2 * x2);
  }

  void calc_waypoint_location(hover_point& new_point, const double dt)
  {
    //ros::Duration t_segment = ros::Time::now() - segment_start_time;
    //    double current_north, current_east;
    //    get_current_lateral_position(current_north, current_east);
    double delta_north = segment_end.north - segment_start.north;
    double delta_east = segment_end.east - segment_start.east;
    //    NODELET_INFO_STREAM("delta_north = " << delta_north << " delta_east = " << delta_east);
    double norm_delta = norm2(delta_north, delta_east);
    double dir_north = 0;
    double dir_east = 0;
    if (norm_delta > 0.001)
    {
      dir_north = delta_north / norm_delta;
      dir_east = delta_east / norm_delta;
    }
    //      NODELET_INFO_STREAM("dir_north = " << dir_north << " dir_east = " << dir_east);
    //      NODELET_INFO_STREAM("dt = " << dt);
    new_point.north = north_cmd + dir_north * dt * waypoint_speed;
    new_point.east = east_cmd + dir_east * dt * waypoint_speed;
    new_point.north_vel = dir_north * waypoint_speed;
    new_point.east_vel = dir_east * waypoint_speed;
    double norm_now = norm2(new_point.north - segment_start.north, new_point.east - segment_start.east);
    //double distance = min(norm_delta, waypoint_speed * t_segment.toSec());
    if (norm_now > norm_delta)
    {
      new_point.north = segment_end.north;
      new_point.east = segment_end.east;
    }

    if (not isnan(segment_end.yaw))
    {
      new_point.yaw = segment_end.yaw;
    }
    else
    {
      new_point.yaw = yaw_cmd;
    }
  }

  bool get_autosequence_callback(flyer_controller::GetAutosequence::Request& req,
                                 flyer_controller::GetAutosequence::Response& resp)
  {
    if (autosequence_exists(req.autosequence_name))
    {
      resp.found = true;
      resp.autosequence.name = req.autosequence_name;
      resp.autosequence.num_points = autosequences[req.autosequence_name].size();
      for (int i = 0; i < resp.autosequence.num_points; i++)
      {
        autosequence_point cur_pt = autosequences[req.autosequence_name][i];
        flyer_controller::AutosequencePoint new_ap;
        new_ap.pause = cur_pt.pause;
        new_ap.hover_point.name = cur_pt.point.name;
        new_ap.hover_point.x = cur_pt.point.north;
        new_ap.hover_point.y = cur_pt.point.east;
        new_ap.hover_point.vx = cur_pt.point.north_vel;
        new_ap.hover_point.vy = cur_pt.point.east_vel;
        new_ap.hover_point.yaw = cur_pt.point.yaw;
        resp.autosequence.points.push_back(new_ap);
      }
    }
    else
    {
      resp.found = false;
    }
    return true;
  }

  void process_joystick(const joy::JoyConstPtr & joy_msg)
  {
  }

  void reportStatusTimerCallback(const ros::TimerEvent& e)
  {
    HoverMode::reportStatusTimerCallback(e);
    control_mode_autosequence_infoPtr autosequence_info_msg(new control_mode_autosequence_info);
    autosequence_info_msg->current_autosequence = current_autosequence;
    autosequence_info_msg->current_point = current_point;
    autosequence_info_msg->header.stamp = e.current_real;
    switch (wp_update_state)
    {
      case AutosequenceTypes::IDLE:
        autosequence_info_msg->status = control_mode_autosequence_info::IDLE;
        break;
      case AutosequenceTypes::PAUSED:
        autosequence_info_msg->status = control_mode_autosequence_info::PAUSED;
        break;
      case AutosequenceTypes::WAITING_PROCEED:
        autosequence_info_msg->status = control_mode_autosequence_info::WAITING_PROCEED;
        break;
      case AutosequenceTypes::MOVING:
        autosequence_info_msg->status = control_mode_autosequence_info::MOVING;
        break;
    }
    // @todo: don't redo this every iteration..
    autosequence_info_msg->defined_autosequences.clear();
    for (map<string, autosequence>::iterator iter = autosequences.begin(); iter != autosequences.end(); ++iter)
    {
      autosequence_info_msg->defined_autosequences.push_back(iter->first);
    }
    autosequence_info_pub.publish(autosequence_info_msg);

  }

}; // class

PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeAutosequence, flyer_controller::ControlModeAutosequence, nodelet::Nodelet)
;

} // namespace
