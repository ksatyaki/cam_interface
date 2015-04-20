/**
 * @file   cam_interface.h
 * @author Chittaranjan S Srinivas
 * 
 * @brief  This file contains prototypes for functions that communicate
 *         with CAM.
 *     
 * Copyright (C) 2015  Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
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
 * Check if an object is in the CAM. 
 * @param object_name Name of the object that you wish to check.
 * @return true if the object is in the CAM. False otherwise.
 */
bool isObjectInCAM(std::string object_name);

/**
 * Subscribe to all things in CAM.
 */
void subscribeToCAM();

/** 
 *  
 * @param object_name The name of the object whose reachable pose you want to get.
 * @param tf_listener_ The tf_listener that would be used with this operation.
 * 
 * @return The reachable pose for given object.
 */
geometry_msgs::PointStamped getObjectReachablePositionFromCAM(std::string object_name, boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * Retrive the signature of all objects from cam and return them in a doro_msgs::TableObjectArray.
 */
doro_msgs::TableObjectArray getAllObjectSignaturesFromCAM(boost::shared_ptr <tf::TransformListener>& tf_listener_);

/**
 * Retrive a vector of strings that contans all the dynamic object's names.
 */
std::vector <std::string> getAllObjectNamesFromCAM();

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
