/*
 * cam_interface.h
 *
 *  Created on: Oct 6, 2014
 *      Author: ace
 */

#ifndef CAM_INTERFACE_H_
#define CAM_INTERFACE_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <doro_msgs/TableObjectArray.h>

extern "C"
{
#include <peiskernel/peiskernel.h>
#include <peiskernel/peiskernel_mt.h>
}

#define CAM_PEIS_ID 9898


namespace cam_interface {

/**
 * Subscribe to all things in CAM.
 */
void subscribeToCAM();

/**
 * Retrive the signature of all objects from cam and return them in a doro_msgs::TableObjectArray.
 */
doro_msgs::TableObjectArray getAllObjectSignaturesFromCAM(boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * Retrive the signature of one object from and return it as a doro_msgs::TableObject.
 */
doro_msgs::TableObject getObjectSignatureFromCAM (const std::string& object_name, boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * A convenience function to retrive the position of object from CAM.
 */
geometry_msgs::PointStamped getObjectPositionFromCAM(std::string object_name, boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * A convenience function to retrive the object colors from CAM.
 */
std::vector <uint8_t> getObjectColorFromCAM(std::string object_name);

/**
 * Transform a point to base_link and return as a PointStamped.s
 */
geometry_msgs::PointStamped transformPointToBaseLink(std::vector <double> point_in_map, boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * A convenience function to retrive the object size from CAM.
 */
std::vector <double> getObjectSizeFromCAM(std::string object_name);

/**
 * A convenience function to retrive the object position_tolerance from CAM.
 */
std::vector <double> getObjectPositionToleranceFromCAM(std::string object_name);

/**
 * A convenience function to tokenize and convert to double values.
 */
std::vector<double> extractParams(const char p[]);

/**
 * A convenience function to tokenize a string.
 */
std::vector<std::string> extractTokens(const char p[]);

}



#endif /* CAM_INTERFACE_H_ */
