#pragma once

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

namespace rosparam_handler {

/**
 * Sets the logger level according to a standardized parameter
 * name 'verbosity'.
 *
 * @param nodeHandle The ROS node handle to search for the
 * parameter 'verbosity'.
 */
inline void setLoggerLevel(const ros::NodeHandle& nodeHandle) {

	std::string verbosity;
	if (!nodeHandle.getParam("verbosity", verbosity)) {
		verbosity = "warning";
	}

	ros::console::Level level_ros;
	bool valid_verbosity {true};
	if (verbosity == "debug") {
		level_ros = ros::console::levels::Debug;
	} else if (verbosity == "info") {
		level_ros = ros::console::levels::Info;
	} else if (verbosity == "warning") {
		level_ros = ros::console::levels::Warn;
	} else if (verbosity == "error") {
		level_ros = ros::console::levels::Error;
	} else if (verbosity == "fatal") {
		level_ros = ros::console::levels::Fatal;
	} else {
		ROS_WARN_STREAM(
				"Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
		valid_verbosity = false;
	}
	if (valid_verbosity) {
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level_ros)) {
			ros::console::notifyLoggerLevelsChanged();
			ROS_DEBUG_STREAM("Verbosity set to " << verbosity);
		}
	}
}

/**
 * Show summary about node containing name, namespace,
 * subscribed and advertised topics.
 */
inline void showNodeInfo() {

	using namespace ros::this_node;

	std::vector<std::string> subscribed_topics, advertised_topics;
	getSubscribedTopics(subscribed_topics);
	getAdvertisedTopics(advertised_topics);

	std::ostringstream msg_subscr, msg_advert;
	for (auto const& t : subscribed_topics) {
		msg_subscr << t << std::endl;
	}
	for (auto const& t : advertised_topics) {
		msg_advert << t << std::endl;
	}

	ROS_INFO_STREAM(
			"Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl << "Subscribed topics: " << std::endl << msg_subscr.str() << "Advertised topics: " << std::endl << msg_advert.str());
}

} // rosparam_handler
