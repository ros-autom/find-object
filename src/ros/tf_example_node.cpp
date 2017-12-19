/*
 Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Universite de Sherbrooke nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//the maximum number of objects that can identify
#define max_number_of_objects 100
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include <fstream>
#include <pwd.h>
class TfExample
{
public:
	TfExample() :
	mapFrameId_("/map"),objFramePrefix_("object"),static_objects_(true)

	{
		ros::NodeHandle pnh("~");
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);
		pnh.param("static_objects",static_objects_,static_objects_);

		ros::NodeHandle nh;
		subs_ = nh.subscribe("objectsStamped", 1,&TfExample::objectsDetectedCallback, this);
		//this timer is publishing the tf from the identified objects
		timer_ = nh.createTimer(ros::Duration(0.1),&TfExample::tf_publish_callback, this);
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for whiofstreamch objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		if ((total_objects_ > max_number_of_objects) && static_objects_) //&& static_objects_)
		{

			ROS_ERROR("You cant add more objects!!! The maximum number of objects is 10 ");
			total_objects_ = max_number_of_objects;
			return;
		}

		if (msg->objects.data.size())
		{
			for (unsigned int i = 0; i < msg->objects.data.size(); i += 12)
			{
				// get data
				int id = (int) msg->objects.data[i];
				std::string objectFrameId = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString(); // "object_1", "object_2"

				tf::StampedTransform pose;
				tf::StampedTransform poseCam;
				try
				{
					// Get transformation from "object_#" frame to target frame "map"
					// The timestamp matches the one sent over TF

					tfListener_.lookupTransform(mapFrameId_, objectFrameId,msg->header.stamp, pose);
					tfListener_.lookupTransform(msg->header.frame_id,objectFrameId, msg->header.stamp, poseCam);

				}
				catch (tf::TransformException & ex)
				{

					ROS_WARN("%s", ex.what());
					continue;
				}

				//if the new objects has already identified,return
				if(static_objects_)
				{
					for (int i = 0; i < total_objects_; i++)
					{
						if (((std::abs(pose.getOrigin().x()-poses_[i].x)<1.0) && (std::abs(pose.getOrigin().y()-poses_[i].y)<1.0) && (std::abs(pose.getOrigin().z()-poses_[i].z)<1.0)))
						{
							ROS_ERROR("This objects is already registered ");
							return;
						}
					}

				}
				//else read and save it
				ROS_INFO("A new object has been detected");

				// Here "pose" is the position of the object "id" in "/map" frame.
				ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, mapFrameId_.c_str(), pose.getOrigin().x(),
						pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(),
						pose.getRotation().z(), pose.getRotation().w());



						ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, msg->header.frame_id.c_str(),
						poseCam.getOrigin().x(), poseCam.getOrigin().y(),
						poseCam.getOrigin().z(), poseCam.getRotation().x(),
						poseCam.getRotation().y(), poseCam.getRotation().z(),
						poseCam.getRotation().w());

				poses_[total_objects_].x = pose.getOrigin().x();
				poses_[total_objects_].y = pose.getOrigin().y();
				poses_[total_objects_].z = pose.getOrigin().z();
				objects_id_[total_objects_]=QString("%1_%2").arg((objFramePrefix_.c_str())).arg(total_objects_).toStdString();

				if(static_objects_)
				{


				    //grab the home dir
					struct passwd *pw = getpwuid(getuid());
					const char *homedir = pw->pw_dir;
					std::string ss2 = "/poses.txt";
					std::string ss3 = homedir + ss2;

					static std::ofstream poses_file;
					poses_file.open(ss3, std::ofstream::out | std::ofstream::app);


					if(poses_file.fail())
						ROS_ERROR("Failed to open the poses.txt ");
					else
					{
						poses_file <<objects_id_[total_objects_]<<std::endl<<poses_[total_objects_]<<std::endl;
						poses_file.close();
					}
				}
				total_objects_++;

				ROS_INFO("The total number of object is %d ",total_objects_);
				ROS_INFO("The new object is the %s", objects_id_[total_objects_]);
				first_object_ = true;

			}
		}
	}

	void tf_publish_callback(const ros::TimerEvent& event)
	{

		if ( first_object_ && static_objects_)
		{


			//read the saved objects and broadcast the tf between them and the map
			for (int j = 0; j < total_objects_; j++)
			{
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(poses_[j].x, poses_[j].y, poses_[j].z));
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),mapFrameId_, objects_id_[j]));
			}
		}

	}

private:
 	//the timer begins to publish objects when the first object is detected
	bool first_object_ = false;
	//if static_objects is true, the program will save and broadcast the identify objects continuity
	bool static_objects_;
	int total_objects_ = 0;
	std::string objects_id_[max_number_of_objects];
	std::string mapFrameId_;
	std::string objFramePrefix_;
	ros::Subscriber subs_;
	tf::TransformListener tfListener_;
	tf::TransformBroadcaster tf_broadcaster_;
	ros::Timer timer_;
	geometry_msgs::Vector3 poses_[max_number_of_objects];

};

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "tf_example_node");

	TfExample sync;
	ros::spin();
}
