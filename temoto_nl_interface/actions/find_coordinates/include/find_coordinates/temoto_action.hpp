#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "find_coordinates/input_parameters.hpp"

/**
 * @brief Class that integrates TeMoto Base Subsystem specific and Action Engine specific codebases.
 *
 */
class TemotoAction : public ActionBase
{
public:

  TemotoAction()
  {}

  /**
   * @brief Get the Name of the action
   *
   * @return const std::string&
   */
  const std::string& getName()
  {
    return getUmrfNodeConst().getFullName();
  }

  virtual void updateParameters(const ActionParameters& parameters_in)
  {
  }

  input_parameters_t params_in;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.location = params.getParameterData<std::string>("location");
    params_in.pose.frame_id = params.getParameterData<std::string>("pose::frame_id");
    params_in.pose.position.x = params.getParameterData<double>("pose::position::x");
    params_in.pose.position.y = params.getParameterData<double>("pose::position::y");
    params_in.pose.position.z = params.getParameterData<double>("pose::position::z");
    params_in.pose.orientation.r = params.getParameterData<double>("pose::orientation::r");
    params_in.pose.orientation.p = params.getParameterData<double>("pose::orientation::p");
    params_in.pose.orientation.y = params.getParameterData<double>("pose::orientation::y");
  }

  void setOutputParameters()
  {
  }
};