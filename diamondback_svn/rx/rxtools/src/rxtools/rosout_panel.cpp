/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
********************************************************************/

#include "rosout_panel.h"
#include "rosout_setup_dialog.h"
#include "rosout_list_control.h"

#include <wx/wx.h>
#include <wx/artprov.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <algorithm>

#include <boost/bind.hpp>

#include "rosout_text_filter.h"
#include "rosout_text_filter_control.h"
#include "rosout_severity_filter.h"
#include "rosout_severity_filter_control.h"

namespace rxtools
{

RosoutPanel::RosoutPanel(wxWindow* parent, int id, wxPoint pos, wxSize size, int style)
: RosoutPanelBase(parent, id, pos, size, style)
, enabled_(false)
, message_id_counter_(0)
, max_messages_(20000)
, needs_refilter_(false)
, refilter_timer_(0.0f)
, pause_(false)
, logger_level_frame_(0)
{
  wxInitAllImageHandlers();

  nh_.setCallbackQueue(&callback_queue_);

  process_timer_ = new wxTimer(this);
  process_timer_->Start(250);

  Connect(process_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(RosoutPanel::onProcessTimer), NULL, this);

  table_->setModel(this);

  setTopic("/rosout_agg");
  setEnabled(true);

  std::string icon_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/icons/";
  delete_filter_bitmap_ = wxBitmap(wxString::FromAscii((icon_path + "delete-filter-16.png").c_str()), wxBITMAP_TYPE_PNG);

  wxBitmap add_bitmap(wxString::FromAscii((icon_path + "add-16.png").c_str()), wxBITMAP_TYPE_PNG);
  add_filter_button_->SetBitmapLabel(add_bitmap);

  add_filter_button_->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RosoutPanel::onAddFilterPressed), NULL, this);

  {
    RosoutSeverityFilterPtr filter(new RosoutSeverityFilter);
    RosoutSeverityFilterControl* control = new RosoutSeverityFilterControl(this, filter);
    severity_filter_ = filter;
    severity_sizer_->Add(control, 0, wxEXPAND);
    severity_filter_->getChangedSignal().connect(boost::bind(&RosoutPanel::onFilterChanged, this, _1));
  }

  createTextFilter();

  filters_window_->SetMinSize(wxSize(-1, filters_[0].panel->GetSize().GetHeight() + add_filter_button_->GetSize().GetHeight() + 5));
}

RosoutPanel::~RosoutPanel()
{
  unsubscribe();

  Disconnect(process_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(RosoutPanel::onProcessTimer), NULL, this);

  delete process_timer_;

  clear();
}

void RosoutPanel::clear()
{
  table_->SetItemCount(0);
  messages_.clear();
  ordered_messages_.clear();
}

void RosoutPanel::setEnabled(bool enabled)
{
  if (enabled_ == enabled)
  {
    return;
  }

  enabled_ = enabled;
  if (enabled)
  {
    subscribe();
  }
  else
  {
    unsubscribe();
  }
}

void RosoutPanel::subscribe()
{
  if (!enabled_ || topic_.empty())
  {
    return;
  }

  sub_ = nh_.subscribe(topic_, 0, &RosoutPanel::incomingMessage, this);
}

void RosoutPanel::unsubscribe()
{
  sub_.shutdown();
}

void RosoutPanel::setTopic(const std::string& topic)
{
  if (topic == topic_)
  {
    return;
  }

  unsubscribe();

  topic_ = topic;

  subscribe();
}

void RosoutPanel::setMessages(const M_IdToMessage& messages)
{
  messages_ = messages;

  if (messages.empty())
  {
    message_id_counter_ = 0;
  }
  else
  {
    message_id_counter_ = messages.rbegin()->first;
  }

  refilter();
}

RosoutFrame* RosoutPanel::createNewFrame()
{
  RosoutFrame* frame = new RosoutFrame(0);
  frame->rosout_panel_->setMessages(messages_);
  frame->Show();
  frame->Raise();

  return frame;
}

void RosoutPanel::onNewWindow(wxCommandEvent& event)
{
  createNewFrame();
}

void RosoutPanel::onLoggerLevelsClose(wxCloseEvent& event)
{
  logger_level_frame_->Show(false);
  event.Veto();
}

void RosoutPanel::onLoggerLevels(wxCommandEvent& event)
{
  if (!logger_level_frame_)
  {
    logger_level_frame_ = new LoggerLevelFrame(this);
    logger_level_frame_->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(RosoutPanel::onLoggerLevelsClose), NULL, this);
  }

  logger_level_frame_->Show();
  logger_level_frame_->Raise();
}

bool filterEnabledCheckboxEqual(wxWindowID id, const RosoutPanel::FilterInfo& info)
{
  return info.enabled_cb && info.enabled_cb->GetId() == id;
}

void RosoutPanel::onFilterEnableChecked(wxCommandEvent& event)
{
  V_FilterInfo::iterator it = std::find_if(filters_.begin(), filters_.end(), boost::bind(filterEnabledCheckboxEqual, event.GetId(), _1));
  if (it != filters_.end())
  {
    FilterInfo& info = *it;
    info.filter->setEnabled(event.IsChecked());
    refilter();
  }
}

bool filterDeleteButtonEqual(wxWindowID id, const RosoutPanel::FilterInfo& info)
{
  return info.delete_button && info.delete_button->GetId() == id;
}

void RosoutPanel::onFilterDelete(wxCommandEvent& event)
{
  V_FilterInfo::iterator it = std::find_if(filters_.begin(), filters_.end(), boost::bind(filterDeleteButtonEqual, event.GetId(), _1));
  if (it != filters_.end())
  {
    FilterInfo& info = *it;
    removeFilter(info.filter);
  }
}

bool filterUpButtonEqual(wxWindowID id, const RosoutPanel::FilterInfo& info)
{
  return info.up_button && info.up_button->GetId() == id;
}

void RosoutPanel::onFilterMoveUp(wxCommandEvent& event)
{
  V_FilterInfo::iterator it = std::find_if(filters_.begin(), filters_.end(), boost::bind(filterUpButtonEqual, event.GetId(), _1));
  if (it != filters_.end() && it != filters_.begin())
  {
    FilterInfo& info = *it;

    filters_sizer_->Detach(info.panel);
    size_t new_index = it - filters_.begin() - 1;
    filters_sizer_->Insert(new_index, info.panel, 0, wxEXPAND|wxBOTTOM, 1);
    filters_sizer_->Layout();
    std::swap(*it, *(it - 1));

    resizeFiltersPane();
    updateFilterBackgrounds();
  }
}

bool filterDownButtonEqual(wxWindowID id, const RosoutPanel::FilterInfo& info)
{
  return info.down_button && info.down_button->GetId() == id;
}

void RosoutPanel::onFilterMoveDown(wxCommandEvent& event)
{
  V_FilterInfo::iterator it = std::find_if(filters_.begin(), filters_.end(), boost::bind(filterDownButtonEqual, event.GetId(), _1));
  if (it != filters_.end() && it != filters_.end() - 1)
  {
    FilterInfo& info = *it;

    filters_sizer_->Detach(info.panel);
    size_t new_index = it - filters_.begin() + 1;
    filters_sizer_->Insert(new_index, info.panel, 0, wxEXPAND|wxBOTTOM, 1);
    filters_sizer_->Layout();
    std::swap(*it, *(it + 1));

    resizeFiltersPane();
    updateFilterBackgrounds();
  }
}

void RosoutPanel::updateFilterBackgrounds()
{
  for (size_t i = 0; i < filters_.size(); ++i)
  {
    FilterInfo& info = filters_[i];
    if (i % 2 == 0)
    {
      info.panel->SetBackgroundColour(*wxLIGHT_GREY);
      info.control->SetBackgroundColour(*wxLIGHT_GREY);
    }
    else
    {
      info.panel->SetBackgroundColour(wxNullColour);
      info.control->SetBackgroundColour(wxNullColour);
    }
  }
}

void RosoutPanel::addFilter(const RosoutFilterPtr& filter, wxWindow* control)
{
  table_->preItemChanges();

  FilterInfo info;
  info.filter = filter;
  info.control = control;

  info.panel = new wxPanel(filters_window_, wxID_ANY);
  filters_sizer_->Add(info.panel, 0, wxEXPAND|wxBOTTOM, 1);
  info.panel->SetPosition(wxPoint(0, info.panel->GetSize().GetHeight() * filters_.size()));

  if (filters_.size() % 2 == 0)
  {
    info.panel->SetBackgroundColour(*wxLIGHT_GREY);
    info.control->SetBackgroundColour(*wxLIGHT_GREY);
  }
  else
  {
    info.panel->SetBackgroundColour(wxNullColour);
    info.control->SetBackgroundColour(wxNullColour);
  }

  control->Reparent(info.panel);

  info.sizer = new wxBoxSizer(wxHORIZONTAL);
  info.panel->SetSizer(info.sizer);


#if 01
  //if (disableable)
  {
    info.enabled_cb = new wxCheckBox(info.panel, wxID_ANY, wxT("Enabled"));
    info.enabled_cb->SetValue(filter->isEnabled());
    info.sizer->Add(info.enabled_cb, 0, wxALIGN_CENTER_VERTICAL);
    info.enabled_cb->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(RosoutPanel::onFilterEnableChecked), NULL, this);
  }
#endif

  info.sizer->Add(control, 1, wxALIGN_CENTER_VERTICAL);

  info.delete_button = 0;
  //if (removable)
  {
    info.delete_button = new wxBitmapButton(info.panel, wxID_ANY, delete_filter_bitmap_);
    info.sizer->Add(info.delete_button, 0, wxALIGN_CENTER_VERTICAL);
    info.delete_button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RosoutPanel::onFilterDelete), NULL, this);

    info.down_button = new wxBitmapButton(info.panel, wxID_ANY, wxArtProvider::GetBitmap(wxART_GO_DOWN, wxART_OTHER, wxSize(16, 16)));
    info.sizer->Add(info.down_button, 0, wxALIGN_CENTER_VERTICAL);
    info.down_button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RosoutPanel::onFilterMoveDown), NULL, this);

    info.up_button = new wxBitmapButton(info.panel, wxID_ANY, wxArtProvider::GetBitmap(wxART_GO_UP, wxART_OTHER, wxSize(16, 16)));
    info.sizer->Add(info.up_button, 0, wxALIGN_CENTER_VERTICAL);
    info.up_button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RosoutPanel::onFilterMoveUp), NULL, this);
  }

  filters_.push_back(info);

  resizeFiltersPane();

  filters_window_->Scroll(-1, 100000);

  filter->getChangedSignal().connect(boost::bind(&RosoutPanel::onFilterChanged, this, _1));

  needs_refilter_ = true;

  table_->postItemChanges();
}

void RosoutPanel::resizeFiltersPane()
{
  filters_window_->Layout();

  wxSize sizer_size = filters_window_->GetSizer()->GetMinSize();
  if (sizer_size.GetHeight() > 150)
  {
    filters_window_->SetMinSize(wxSize(-1, 150));
    filters_window_->GetSizer()->FitInside(filters_window_);
  }
  else
  {
    filters_window_->SetMinSize(wxSize(-1, sizer_size.GetHeight()));
  }

  Layout();
  Refresh();
}

bool filterEquals(const RosoutFilterPtr& filter, const RosoutPanel::FilterInfo& info)
{
  return info.filter == filter;
}

void RosoutPanel::removeFilter(const RosoutFilterPtr& filter)
{
  V_FilterInfo::iterator it = std::find_if(filters_.begin(), filters_.end(), boost::bind(filterEquals, filter, _1));
  if (it != filters_.end())
  {
    FilterInfo& info = *it;
    info.panel->Destroy();
    filters_.erase(it);

    resizeFiltersPane();
    updateFilterBackgrounds();

    refilter();
  }
}

void RosoutPanel::clearFilters()
{
  while (!filters_.empty())
  {
    removeFilter(filters_.front().filter);
  }
}

void RosoutPanel::onFilterChanged(const RosoutFilter*)
{
  needs_refilter_ = true;
}

template<class T>
void printStuff(const std::string& name, T* win)
{
  wxPoint point = win->GetPosition();
  wxSize size = win->GetSize();
  ROS_INFO("%s: x: %d, y: %d,      w: %d, h: %d", name.c_str(), point.x, point.y, size.GetWidth(), size.GetHeight());
}

#define PRINT_STUFF(description) \
  ROS_INFO(description); \
  printStuff("  filters_sizer_", filters_sizer_); \
  printStuff("  filters_window_->GetSizer()", filters_window_->GetSizer()); \
  printStuff("  filters_window_", filters_window_); \
  for (size_t i = 0; i < filters_.size(); ++i) \
  { \
    { \
      std::stringstream ss; \
      ss << "    panel " << i; \
      printStuff(ss.str(), filters_[i].panel); \
    } \
    { \
      std::stringstream ss; \
      ss << "    sizer " << i; \
      printStuff(ss.str(), filters_[i].sizer); \
    } \
  }

void RosoutPanel::onProcessTimer(wxTimerEvent& evt)
{
  callback_queue_.callAvailable(ros::WallDuration());

  processMessages();

  refilter_timer_ += 0.25f;
  if (needs_refilter_ && refilter_timer_ > 0.5f)
  {
    refilter_timer_ = 0.0f;
    needs_refilter_ = false;
    refilter();
  }

  //PRINT_STUFF("onProcessTimer");
}

RosoutTextFilterPtr RosoutPanel::createTextFilter()
{
  RosoutTextFilterPtr filter(new RosoutTextFilter);
  RosoutTextFilterControl* control = new RosoutTextFilterControl(filters_window_, filter);
  addFilter(filter, control);

  return filter;
}

void RosoutPanel::onAddFilterPressed(wxCommandEvent& event)
{
  createTextFilter();
}

void RosoutPanel::onClear(wxCommandEvent& event)
{
  clear();
}

void RosoutPanel::addMessageToTable(const rosgraph_msgs::Log::ConstPtr& message, uint32_t id)
{
  ordered_messages_.push_back(id);
}

rosgraph_msgs::LogConstPtr RosoutPanel::getMessageByIndex(uint32_t index) const
{
  if (index >= ordered_messages_.size())
  {
    return rosgraph_msgs::LogConstPtr();
  }

  M_IdToMessage::const_iterator it = messages_.find(ordered_messages_[index]);
  ROS_ASSERT(it != messages_.end());

  return it->second;
}

bool RosoutPanel::filter(uint32_t id) const
{
  // No filters, always include
  if (filters_.empty())
  {
    return true;
  }

  M_IdToMessage::const_iterator it = messages_.find(id);
  ROS_ASSERT(it != messages_.end());

  const rosgraph_msgs::LogConstPtr& message = it->second;

  // First run through the severity filter
  if (!severity_filter_->filter(message))
  {
    return false;
  }

  {
    V_FilterInfo::const_iterator it = filters_.begin();
    V_FilterInfo::const_iterator end = filters_.end();
    for (; it != end; ++it)
    {
      const FilterInfo& info = *it;
      const RosoutFilterPtr& filter = info.filter;
      if (filter->isEnabled() && filter->isValid())
      {
        if (!filter->filter(message))
        {
          return false;
        }
      }
    }
  }

  return true;
}

void RosoutPanel::validateOrderedMessages()
{
#if VALIDATE_FILTERING
  typedef std::set<uint32_t> S_u32;
  S_u32 s;
  V_u32::iterator it = ordered_messages_.begin();
  V_u32::iterator end = ordered_messages_.end();
  for (; it != end; ++it)
  {
    uint32_t id = *it;
    if (!s.insert(id).second)
    {
      ROS_BREAK();
    }
  }
#endif
}

void RosoutPanel::refilter()
{
  table_->preItemChanges();

  ordered_messages_.clear();
  M_IdToMessage::iterator it = messages_.begin();
  M_IdToMessage::iterator end = messages_.end();
  for (; it != end; ++it)
  {
    uint32_t id = it->first;
    rosgraph_msgs::Log::ConstPtr& message = it->second;

    if (filter(id))
    {
      addMessageToTable(message, id);
    }
  }

  validateOrderedMessages();

  table_->SetItemCount(ordered_messages_.size());

  table_->postItemChanges();
}

void RosoutPanel::popMessage()
{
  M_IdToMessage::iterator it = messages_.begin();
  if (!ordered_messages_.empty() && ordered_messages_.front() == it->first)
  {

    ordered_messages_.erase(ordered_messages_.begin());
    table_->SetItemCount(ordered_messages_.size());

    // Removing early messages means subtracting 1 from the current selection
    const S_int32& selection = table_->getSelection();
    S_int32 new_selection;

    S_int32::const_iterator it = selection.begin();
    S_int32::const_iterator end = selection.end();
    for (; it != end; ++it)
    {
      int32_t new_index = *it - 1;
      if (new_index >= 0)
      {
        new_selection.insert(new_index);
      }
    }

    table_->setSelection(new_selection);
  }

  messages_.erase(it);
}

void RosoutPanel::processMessage(const rosgraph_msgs::Log::ConstPtr& message)
{
  uint32_t id = message_id_counter_++;

  messages_.insert(std::make_pair(id, message));

  if (filter(id))
  {
    addMessageToTable(message, id);
  }

  validateOrderedMessages();

  if (messages_.size() > max_messages_)
  {
    popMessage();
  }
}

void RosoutPanel::processMessages()
{
  if (message_queue_.empty())
  {
    return;
  }

  table_->preItemChanges();

  V_Log::iterator it = message_queue_.begin();
  V_Log::iterator end = message_queue_.end();
  for (; it != end; ++it)
  {
    rosgraph_msgs::Log::ConstPtr& message = *it;

    processMessage(message);
  }

  message_queue_.clear();

  table_->SetItemCount(ordered_messages_.size());

  table_->postItemChanges();
}

void RosoutPanel::incomingMessage(const rosgraph_msgs::Log::ConstPtr& msg)
{
  if (!pause_)
  {
    message_queue_.push_back(msg);
  }
}

void RosoutPanel::onPause(wxCommandEvent& evt)
{
  pause_ = evt.IsChecked();
}

void RosoutPanel::onSetup(wxCommandEvent& evt)
{
  RosoutSetupDialog dialog(this, topic_, max_messages_);

  if (dialog.ShowModal() == wxOK)
  {
    setTopic(dialog.getTopic());
    setBufferSize(dialog.getBufferSize());
  }
}

void RosoutPanel::setBufferSize(uint32_t size)
{
  max_messages_ = size;
  while (messages_.size() >= max_messages_)
  {
    popMessage();
  }
}

RosoutMessageSummary RosoutPanel::getMessageSummary(double duration) const
{
  RosoutMessageSummary summary;
  ros::Time search_end(0);

  if (ros::Time::now().toSec() - duration > 0)
  {
    search_end = ros::Time::now() - ros::Duration(duration);
  }

  M_IdToMessage::const_reverse_iterator it = messages_.rbegin();
  M_IdToMessage::const_reverse_iterator end = messages_.rend();
  for (; it != end; ++it)
  {
    const rosgraph_msgs::Log::ConstPtr& msg = it->second;

    if (msg->header.stamp < search_end)
    {
      break;
    }

    switch (msg->level)
    {
    case rosgraph_msgs::Log::DEBUG:
      ++summary.debug;
      break;
    case rosgraph_msgs::Log::INFO:
      ++summary.info;
      break;
    case rosgraph_msgs::Log::WARN:
      ++summary.warn;
      break;
    case rosgraph_msgs::Log::ERROR:
      ++summary.error;
      break;
    case rosgraph_msgs::Log::FATAL:
      ++summary.fatal;
      break;
    }
  }

  return summary;
}

} // namespace rxtools
