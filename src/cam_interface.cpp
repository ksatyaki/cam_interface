/*
 * cam_interface.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: ace
 */

#include <cam_interface/cam_interface.h>

namespace cam_interface {

std::vector<double> extractParams(const char p[])
{
	char *copyOfString = new char[strlen(p) + 1];
	strcpy(copyOfString, p);

	std::vector<double> double_values;

	char *pch;
	int cmd_args = 0;

	pch = strtok (copyOfString," ,");
	while (pch != NULL)
	{
		cmd_args++;
		if(!isalpha(pch[0]))
			double_values.push_back(atof(pch));
		pch = strtok (NULL, " ,");
	}

	delete copyOfString;
	return double_values;
}

std::vector<std::string> extractTokens(const char p[])
{
	char *copyOfString = new char[strlen(p) + 1];
	strcpy(copyOfString, p);

	std::vector<std::string> string_tokens;

	char *pch;
	int cmd_args = 0;

	pch = strtok (copyOfString," )(");
	while (pch != NULL)
	{
		cmd_args++;
		string_tokens.push_back(pch);
		pch = strtok (NULL, " )(");
	}

	delete copyOfString;
	return string_tokens;
}

std::vector<double> getItFromCAM(std::string key)
{
	int CAM_peis_id = CAM_PEIS_ID;

	PeisSubscriberHandle c = peiskmt_subscribe(CAM_peis_id, key.c_str());

	PeisTuple *position_tuple = peiskmt_getTuple(CAM_peis_id, key.c_str(), PEISK_KEEP_OLD);
	ROS_INFO("Fetching item from CAM: %s", key.c_str());
	while(!position_tuple)
	{
		position_tuple = peiskmt_getTuple(CAM_peis_id, key.c_str(), PEISK_KEEP_OLD);
		printf(".");
		usleep(100000);
		printf("\b");
		usleep(100000);
	}

	peiskmt_unsubscribe(c);
	return extractParams(position_tuple->data);
}

std::vector <double> getObjectPositionToleranceFromCAM(std::string object_name)
{
	if(!peisk_isRunning())
	{
		printf("Error. PEIS Kernel is not running. Can't get objects from CAM.");
		exit(-1);
	}

	return getItFromCAM(object_name + ".pos.tolerance");
}

std::vector <uint8_t> getObjectColorFromCAM(std::string object_name)
{
	if(!peisk_isRunning())
	{
		printf("Error. PEIS Kernel is not running. Can't get objects from CAM.");
		exit(-1);
	}
	std::vector <uint8_t> colors;
	std::vector <double> colors_double;

	colors_double = getItFromCAM(object_name + ".color.rgb");

	for(std::vector <double>::iterator iter = colors_double.begin(); iter != colors_double.end(); iter++)
		colors.push_back((uint8_t) *iter);

	return colors;
}

std::vector <double> getObjectSizeFromCAM(std::string object_name)
{
	if(!peisk_isRunning())
	{
		printf("Error. PEIS Kernel is not running. Can't get objects from CAM.");
		exit(-1);
	}
	return getItFromCAM(object_name + ".boundingbox");
}

geometry_msgs::PointStamped getObjectPositionFromCAM(std::string object_name, boost::shared_ptr <tf::TransformListener>& tf_listener_)
{
	if(!peisk_isRunning())
	{
		printf("Error. PEIS Kernel is not running. Can't get objects from CAM.");
		exit(-1);
	}

	return transformPointToBaseLink(getItFromCAM (object_name + ".pos.geo"), tf_listener_);
}

geometry_msgs::PointStamped transformPointToBaseLink (std::vector<double> point_in_map, boost::shared_ptr <tf::TransformListener>& tf_listener_)
{
	geometry_msgs::PointStamped object_position;

	if(point_in_map.size() < 3)
	{
		ROS_INFO("There was an error retriving the position of object from CAM. Size was %d.", point_in_map.size());
		object_position.header.frame_id = "para-universe";
		return object_position;
	}

	tf::StampedTransform base_link_to_map;
	tf::Vector3 point_in_map_tf (point_in_map[0], point_in_map[1], point_in_map[2]);

	try
	{
		tf_listener_->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1));
		//tf_listener_->transformPoint("base_link", object_position, _object_position);
		tf_listener_->lookupTransform("base_link", "map", ros::Time(0), base_link_to_map);

	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("What's the problem: %s", ex.what());
		object_position.header.frame_id = "cocked-up";
	}

	tf::Vector3 point_in_base_link = base_link_to_map*point_in_map_tf;

	object_position.header.frame_id = "base_link";
	object_position.header.stamp = ros::Time::now();
	object_position.point.x = point_in_base_link.x();
	object_position.point.y = point_in_base_link.y();
	object_position.point.z = point_in_base_link.z();

	return object_position;
}

doro_msgs::TableObjectArray getAllObjectSignaturesFromCAM(boost::shared_ptr <tf::TransformListener>& tf_listener_)
{
	if(!peisk_isRunning())
	{
		printf("Error. PEIS Kernel is not running. Can't get objects from CAM.");
		exit(-1);
	}
	doro_msgs::TableObjectArray __objects;


	PeisSubscriberHandle c = peiskmt_subscribe(CAM_PEIS_ID, "kernel.all-keys");

	PeisTuple *p = peiskmt_getTuple(CAM_PEIS_ID, "kernel.all-keys", PEISK_KEEP_OLD);
	printf("Wait for tuples from CAM");
	while(!p)
	{
		p = peiskmt_getTuple(CAM_PEIS_ID, "kernel.all-keys", PEISK_KEEP_OLD);
		printf(".");
		usleep(100000);
		printf("\b");
		usleep(100000);
	}
	//printf("Got Tuples, now we decompose.\n");
	peiskmt_unsubscribe(c);
	//printf("\n%s--\n", p->data);

	std::vector <std::string> all_keys = extractTokens(p->data);

	for(std::vector <std::string>::iterator it = all_keys.begin(); it != all_keys.end(); it++)
	{
		if(it->find("kernel") != std::string::npos)
		{
			all_keys.erase(it);
			it--;
		}
	}


	// *************************** //
	// * Extracting Object Names * //
	// *************************** //

	std::vector <std::string> _object_names_;

	for(std::vector <std::string>::iterator it = all_keys.begin(); it != all_keys.end(); it++)
	{
		//printf("--------\n");
		//printf("%s\n", it->c_str());

		// This is based on the assumption that a signature contains at-least a pos.geo
		std::string object_name1 = it->substr(0, it->find(".pos.geo"));
		std::string object_name2 = it->substr(0, it->find(".sift_descriptor"));
		if(object_name1.compare(*it) == 0 && object_name2.compare(*it) != 0)
		{
			if(std::find(_object_names_.begin(), _object_names_.end(), object_name2) == _object_names_.end())
			{
				_object_names_.push_back(object_name2);
				printf("%s added\n", object_name2.c_str());
			}
			//else
				//printf("%s NOT Added\n", object_name2.c_str());

		}
		else if(object_name1.compare(*it) != 0 && object_name2.compare(*it) == 0)
		{
			if(std::find(_object_names_.begin(), _object_names_.end(), object_name1) == _object_names_.end())
			{
				_object_names_.push_back(object_name1);
				printf("%s added\n", object_name1.c_str());
			}
			//else
				//printf("%s NOT Added\n", object_name1.c_str());
		}
		else
		{
			//printf("continue\n");
			continue;
		}
	}

	// *********************************** //
	// * Fetching the signature in parts * //
	// *********************************** //
	for(std::vector <std::string>::iterator it = _object_names_.begin(); it != _object_names_.end(); it++)
	{
		/* Naive method //
		table_object.centroid = getObjectPositionFromCAM(*it, tf_listener_).point;
		table_object.color = getObjectColorFromCAM(*it);
		std::vector <double> tol = getObjectPositionToleranceFromCAM(*it);

		if(tol.size() == 3)
		{
			table_object.centroid_tolerance.x = tol[0];
			table_object.centroid_tolerance.y = tol[1];
			table_object.centroid_tolerance.z = tol[2];
		}
		table_object.cluster_size = getObjectSizeFromCAM(*it);
		*/
		__objects.table_objects.push_back(getObjectSignatureFromCAM (*it, tf_listener_));
	}

	return __objects;

}

doro_msgs::TableObject getObjectSignatureFromCAM (const std::string& object_name, boost::shared_ptr <tf::TransformListener>& tf_listener_)
{
	doro_msgs::TableObject table_object;
	table_object.id = object_name;

	/* Naive method //
			table_object.centroid = getObjectPositionFromCAM(*it, tf_listener_).point;
			table_object.color = getObjectColorFromCAM(*it);
			std::vector <double> tol = getObjectPositionToleranceFromCAM(*it);


			if(tol.size() == 3)
			{
				table_object.centroid_tolerance.x = tol[0];
				table_object.centroid_tolerance.y = tol[1];
				table_object.centroid_tolerance.z = tol[2];
			}

			table_object.cluster_size = getObjectSizeFromCAM(*it);
	 */

	PeisTuple abstract_tuple1, abstract_tuple2;
	peiskmt_initAbstractTuple(&abstract_tuple1);
	peiskmt_initAbstractTuple(&abstract_tuple2);

	peiskmt_setTupleName(&abstract_tuple1, std::string(object_name + ".*").c_str());
	peiskmt_setTupleName(&abstract_tuple2, std::string(object_name + ".*.*").c_str());

	abstract_tuple1.owner = CAM_PEIS_ID;
	abstract_tuple2.owner = CAM_PEIS_ID;

	PeisSubscriberHandle thisSubscriber1 = peiskmt_subscribeByAbstract(&abstract_tuple1);
	PeisSubscriberHandle thisSubscriber2 = peiskmt_subscribeByAbstract(&abstract_tuple2);

	PeisTupleResultSet* tuple_set1;
	PeisTupleResultSet* tuple_set2;

	tuple_set1 = peiskmt_createResultSet();
	tuple_set2 = peiskmt_createResultSet();

	peiskmt_resultSetReset(tuple_set1);
	peiskmt_resultSetReset(tuple_set2);

	peiskmt_getTuplesByAbstract(&abstract_tuple1, tuple_set1);
	printf("\n### Getting object.* tuples for object %s", object_name.c_str());
	while(!peiskmt_resultSetNext(tuple_set1))
	{
		peiskmt_resultSetReset(tuple_set1);
		peiskmt_getTuplesByAbstract(&abstract_tuple1, tuple_set1);
		printf(".");
		usleep(300000);
		printf("\b");
		usleep(300000);
	}

	peiskmt_getTuplesByAbstract(&abstract_tuple2, tuple_set2);
	printf("\n### Getting object.*.* tuples for object %s", object_name.c_str());
	while(!peiskmt_resultSetNext(tuple_set2))
	{
		peiskmt_resultSetReset(tuple_set2);
		peiskmt_getTuplesByAbstract(&abstract_tuple2, tuple_set2);
		printf(".");
		usleep(300000);
		printf("\b");
		usleep(300000);
	}

	while(peisk_isRunning())
	{
		PeisTuple* thisTuple = peiskmt_resultSetValue(tuple_set1);

		char key[100];
		peiskmt_getTupleName(thisTuple, key, 100);
		//printf("\n %s\n", key);

		if(std::string(key).find("boundingbox") != std::string::npos)
		{
			table_object.cluster_size = extractParams(thisTuple->data);
		}

		if(!peiskmt_resultSetNext(tuple_set1))
			break;
	}
	peiskmt_unsubscribe(thisSubscriber1);

	while(peisk_isRunning())
	{
		PeisTuple* thisTuple = peiskmt_resultSetValue(tuple_set2);

		char key[100];
		peiskmt_getTupleName(thisTuple, key, 100);
		//printf("\n %s\n", key);

		std::vector <double> values = extractParams(thisTuple->data);

		if(values.size() == 3)
		{
			if(std::string(key).find("pos.geo") != std::string::npos && std::string(key).find("pos.geo.update") == std::string::npos)
				table_object.centroid = transformPointToBaseLink(values, tf_listener_).point;
			else if(std::string(key).find("pos.tolerance") != std::string::npos)
			{
				table_object.centroid_tolerance.x = values[0];
				table_object.centroid_tolerance.y = values[1];
				table_object.centroid_tolerance.z = values[2];
			}
			else if(std::string(key).find("color.rgb")  != std::string::npos)
			{
				table_object.color.resize(3);
				table_object.color[0] = (uint8_t) values[0];
				table_object.color[1] = (uint8_t) values[1];
				table_object.color[2] = (uint8_t) values[2];
			}
		}

		if(!peiskmt_resultSetNext(tuple_set2))
			break;
	}
	peiskmt_unsubscribe(thisSubscriber2);

	peiskmt_deleteResultSet(tuple_set1);
	peiskmt_deleteResultSet(tuple_set2);

	return table_object;
}

}


