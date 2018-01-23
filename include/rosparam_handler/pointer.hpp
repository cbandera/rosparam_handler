/**
 * \file pointer.h
 *
 *  Created on: Oct 28, 2017
 *
 *  \authors: Jeremie Deray
 */

#ifndef _ROSPARAM_HANDLER_POINTER_H_
#define _ROSPARAM_HANDLER_POINTER_H_

#include <boost/shared_ptr.hpp>

namespace rosparam_handler {

/// \brief forward declaration
struct ParametersBase;

/// \brief base pointer declaration
using ParametersPtr = boost::shared_ptr<ParametersBase>;
using ParametersConstPtr = boost::shared_ptr<const ParametersBase>;

template <typename T>
boost::shared_ptr<T> static_parameters_cast(const rosparam_handler::ParametersPtr& ptr)
{
  return boost::static_pointer_cast<T>(ptr);
}

template <typename T>
boost::shared_ptr<T> dynamic_parameters_cast(const rosparam_handler::ParametersPtr& ptr)
{
  return boost::dynamic_pointer_cast<T>(ptr);
}

} /* namespace rosparam_handler */

#endif /* _ROSPARAM_HANDLER_POINTER_H_ */
