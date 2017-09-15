#pragma once

#include <limits>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

/// \brief Helper function to test for std::vector
template <typename T>
using is_vector = std::is_same<T, std::vector<typename T::value_type, typename T::allocator_type>>;

/// \brief Helper function to test for std::map
template <typename T>
using is_map = std::is_same<
    T, std::map<typename T::key_type, typename T::mapped_type, typename T::key_compare, typename T::allocator_type>>;

/// \brief Outstream helper for std:vector
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
    if (!v.empty()) {
        out << '[';
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
        out << "\b\b]";
    }
    return out;
}

/// \brief Outstream helper for std:map
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& stream, const std::map<T1, T2>& map) {
    stream << '{';
    for (typename std::map<T1, T2>::const_iterator it = map.begin(); it != map.end(); ++it) {
        stream << (*it).first << " --> " << (*it).second << ", ";
    }
    stream << '}';
    return stream;
}

namespace rosparam_handler {

/// \brief Sets the logger level according to a standardized parameter name 'verbosity'.
///
/// \param nodeHandle The ROS node handle to search for the parameter 'verbosity'.
inline void setLoggerLevel(const ros::NodeHandle& nodeHandle) {

    std::string verbosity;
    if (!nodeHandle.getParam("verbosity", verbosity)) {
        verbosity = "warning";
    }

    ros::console::Level level_ros;
    bool valid_verbosity{true};
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
        ROS_WARN_STREAM("Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
        valid_verbosity = false;
    }
    if (valid_verbosity) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level_ros)) {
            ros::console::notifyLoggerLevelsChanged();
            ROS_DEBUG_STREAM("Verbosity set to " << verbosity);
        }
    }
}

/// \brief Show summary about node containing name, namespace, subscribed and advertised topics.
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

    ROS_INFO_STREAM("Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl
                                << "Subscribed topics: " << std::endl
                                << msg_subscr.str() << "Advertised topics: " << std::endl
                                << msg_advert.str());
}

/// \brief Retrieve node name
///
/// @param privateNodeHandle The private ROS node handle (i.e.
/// ros::NodeHandle("~") ).
/// @return node name
inline std::string getNodeName(const ros::NodeHandle& privateNodeHandle) {
    std::string name_space = privateNodeHandle.getNamespace();
    std::stringstream tempString(name_space);
    std::string name;
    while (std::getline(tempString, name, '/')) {
        ;
    }
    return name;
}

/// \brief ExitFunction for rosparam_handler
inline void exit(const std::string msg = "Runtime Error in rosparam handler.") {
    // std::exit(EXIT_FAILURE);
    throw std::runtime_error(msg);
}

/// \brief Set parameter on ROS parameter server
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
inline void setParam(const std::string key, T val) {
    ros::param::set(key, val);
}

/// \brief Get parameter from ROS parameter server
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
inline bool getParam(const std::string key, T& val) {
    if (!ros::param::has(key)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        return false;
    } else if (!ros::param::get(key, val)) {
        ROS_ERROR_STREAM("Could not retrieve parameter'" << key << "'. Does it have a different type?");
        return false;
    } else {
        return true;
    }
}

/// \brief Get parameter from ROS parameter server or use default value
///
/// If parameter does not exist on server yet, the default value is used and set on server.
/// \param key Parameter name
/// \param val Parameter value
/// \param defaultValue Parameter default value
template <typename T>
inline bool getParam(const std::string key, T& val, const T& defaultValue) {
    if (!getParam(key, val)) {
        val = defaultValue;
        ros::param::set(key, defaultValue);
        ROS_INFO_STREAM("Setting default value.");
        return true;
    } else {
        // Param was already retrieved with last if statement.
        return true;
    }
}

/// \brief Tests that parameter is not set on the parameter server
inline bool testConstParam(const std::string key) {
    if (ros::param::has(key)) {
        ROS_WARN_STREAM("Parameter " << key
                                     << "' was set on the parameter server eventhough it was defined to be constant.");
        return false;
    } else {
        return true;
    }
}

/// \brief Limit parameter to lower bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMin(const std::string key, T& val, T min = std::numeric_limits<T>::min()) {
    if (val < min) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is smaller than minimal allowed value. Correcting value to min=" << min);
        val = min;
    }
}

/// \brief Limit parameter to lower bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMin(const std::string key, std::vector<T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val)
        testMin(key, v, min);
}

/// \brief Limit parameter to lower bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMin(const std::string key, std::map<K, T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val)
        testMin(key, v.second, min);
}

/// \brief Limit parameter to upper bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMax(const std::string key, T& val, T max = std::numeric_limits<T>::max()) {
    if (val > max) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is greater than maximal allowed. Correcting value to max=" << max);
        val = max;
    }
}

/// \brief Limit parameter to upper bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMax(const std::string key, std::vector<T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val)
        testMax(key, v, max);
}

/// \brief Limit parameter to upper bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMax(const std::string key, std::map<K, T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val)
        testMax(key, v.second, max);
}

} // namespace rosparam_handler
