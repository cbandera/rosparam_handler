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

} /* namespace rosparam_handler */

#endif /* _ROSPARAM_HANDLER_POINTER_H_ */
