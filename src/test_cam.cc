#include <cam_interface/cam_interface.h>
#include <iostream>

extern "C"
{
#include <peiskernel/peiskernel_mt.h>
}

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "test_cam_interface");
	sleep(1);

	boost::shared_ptr <tf::TransformListener> _tf_;

	_tf_ = boost::shared_ptr <tf::TransformListener> (new tf::TransformListener (ros::Duration(10.0)));

	doro_msgs::TableObjectArray array = cam_interface::getAllObjectSignaturesFromCAM(_tf_);

	std::cout<<array;

	return 0;
}
