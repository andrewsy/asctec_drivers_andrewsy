//#line 2 "/opt/ros/diamondback/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
// *********************************************************
// 
// File autogenerated for the flyer_controller package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __flyer_controller__CONTROLLERCONFIG_H__
#define __flyer_controller__CONTROLLERCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>

namespace flyer_controller
{
  class controllerConfigStatics;
  
  class controllerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(controllerConfig &config, const controllerConfig &max, const controllerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const controllerConfig &config1, const controllerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, controllerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const controllerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, controllerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const controllerConfig &config) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T controllerConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (controllerConfig::* field);

      virtual void clamp(controllerConfig &config, const controllerConfig &max, const controllerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const controllerConfig &config1, const controllerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, controllerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const controllerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, controllerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const controllerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }
    };

//#line 14 "../cfg/controller.cfg"
      double KP;
//#line 15 "../cfg/controller.cfg"
      double KI;
//#line 16 "../cfg/controller.cfg"
      double KD;
//#line 17 "../cfg/controller.cfg"
      std::string command_frame;
//#line 18 "../cfg/controller.cfg"
      double external_frame_heading;
//#line 138 "/opt/ros/diamondback/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;
      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("controllerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      __toMessage__(msg, __param_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const controllerConfig &__max__ = __getMax__();
      const controllerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const controllerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const controllerConfig &__getDefault__();
    static const controllerConfig &__getMax__();
    static const controllerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    
  private:
    static const controllerConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void controllerConfig::ParamDescription<std::string>::clamp(controllerConfig &config, const controllerConfig &max, const controllerConfig &min) const
  {
    return;
  }

  class controllerConfigStatics
  {
    friend class controllerConfig;
    
    controllerConfigStatics()
    {
//#line 14 "../cfg/controller.cfg"
      __min__.KP = 0.0;
//#line 14 "../cfg/controller.cfg"
      __max__.KP = 30.0;
//#line 14 "../cfg/controller.cfg"
      __default__.KP = 15.0;
//#line 14 "../cfg/controller.cfg"
      __param_descriptions__.push_back(controllerConfig::AbstractParamDescriptionConstPtr(new controllerConfig::ParamDescription<double>("KP", "double", 0, "Proportional gain", "", &controllerConfig::KP)));
//#line 15 "../cfg/controller.cfg"
      __min__.KI = 0.0;
//#line 15 "../cfg/controller.cfg"
      __max__.KI = 2.0;
//#line 15 "../cfg/controller.cfg"
      __default__.KI = 1.0;
//#line 15 "../cfg/controller.cfg"
      __param_descriptions__.push_back(controllerConfig::AbstractParamDescriptionConstPtr(new controllerConfig::ParamDescription<double>("KI", "double", 0, "Integral gain", "", &controllerConfig::KI)));
//#line 16 "../cfg/controller.cfg"
      __min__.KD = 0.0;
//#line 16 "../cfg/controller.cfg"
      __max__.KD = 30.0;
//#line 16 "../cfg/controller.cfg"
      __default__.KD = 15.0;
//#line 16 "../cfg/controller.cfg"
      __param_descriptions__.push_back(controllerConfig::AbstractParamDescriptionConstPtr(new controllerConfig::ParamDescription<double>("KD", "double", 0, "Derivative gain", "", &controllerConfig::KD)));
//#line 17 "../cfg/controller.cfg"
      __min__.command_frame = "";
//#line 17 "../cfg/controller.cfg"
      __max__.command_frame = "";
//#line 17 "../cfg/controller.cfg"
      __default__.command_frame = "internal";
//#line 17 "../cfg/controller.cfg"
      __param_descriptions__.push_back(controllerConfig::AbstractParamDescriptionConstPtr(new controllerConfig::ParamDescription<std::string>("command_frame", "str", 0, "Command frame (internal/external)", "{'enum_description': 'Enum to set the command frame.', 'enum': [{'srcline': 11, 'description': 'internal frame', 'srcfile': '../cfg/controller.cfg', 'cconsttype': 'const char * const', 'value': 'internal', 'ctype': 'std::string', 'type': 'str', 'name': 'internal'}, {'srcline': 12, 'description': 'external frame', 'srcfile': '../cfg/controller.cfg', 'cconsttype': 'const char * const', 'value': 'external', 'ctype': 'std::string', 'type': 'str', 'name': 'external'}]}", &controllerConfig::command_frame)));
//#line 18 "../cfg/controller.cfg"
      __min__.external_frame_heading = 0.0;
//#line 18 "../cfg/controller.cfg"
      __max__.external_frame_heading = 359.0;
//#line 18 "../cfg/controller.cfg"
      __default__.external_frame_heading = 0.0;
//#line 18 "../cfg/controller.cfg"
      __param_descriptions__.push_back(controllerConfig::AbstractParamDescriptionConstPtr(new controllerConfig::ParamDescription<double>("external_frame_heading", "double", 0, "Heading [deg] that external command frame points to", "", &controllerConfig::external_frame_heading)));
//#line 239 "/opt/ros/diamondback/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
    
      for (std::vector<controllerConfig::AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        __description_message__.parameters.push_back(**i);
      __max__.__toMessage__(__description_message__.max, __param_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__); 
    }
    std::vector<controllerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    controllerConfig __max__;
    controllerConfig __min__;
    controllerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;
    static const controllerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static controllerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &controllerConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const controllerConfig &controllerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const controllerConfig &controllerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const controllerConfig &controllerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<controllerConfig::AbstractParamDescriptionConstPtr> &controllerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const controllerConfigStatics *controllerConfig::__get_statics__()
  {
    const static controllerConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = controllerConfigStatics::get_instance();
    
    return statics;
  }

//#line 11 "../cfg/controller.cfg"
      const char * const controller_internal = "internal";
//#line 12 "../cfg/controller.cfg"
      const char * const controller_external = "external";
}

#endif // __CONTROLLERRECONFIGURATOR_H__
