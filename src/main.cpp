#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Mapper.h>
#include "Trajectory.h"
#include "MapperParameters.h"
#include <iostream>

typedef norlab_icp_mapper::PM PM;
typedef norlab_icp_mapper::T T;

std::unique_ptr<MapperParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
std::unique_ptr<Trajectory> trajectory;
PM::TransformationParameters odomToMap;

void populateTfBuffer(const rosbag::Bag& bag, ros::Time& earliestStamp, ros::Time& latestStamp)
{
	std::vector<std::string> topics = { "/tf_static", "/tf" };
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	earliestStamp = ros::TIME_MAX;
	latestStamp = ros::TIME_MIN;
	for(rosbag::MessageInstance const message: view)
	{
		tf2_msgs::TFMessage::ConstPtr tfs = message.instantiate<tf2_msgs::TFMessage>();
		if(tfs != nullptr)
		{
			for(const geometry_msgs::TransformStamped& tf : tfs->transforms)
			{
				tfBuffer->setTransform(tf, "default_authority", message.getTopic() == "/tf_static");
				
				if(tf.header.stamp < earliestStamp)
				{
					earliestStamp = tf.header.stamp;
				}
				
				if(tf.header.stamp > latestStamp)
				{
					latestStamp = tf.header.stamp;
				}
			}
		}
	}
}

PM::TransformationParameters findTransform(std::string sourceFrame, std::string targetFrame, ros::Time time, int transformDimension)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time);
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(tf, transformDimension);
}

void gotInput(PM::DataPoints input, ros::Time timeStamp)
{
	try
	{
		PM::TransformationParameters sensorToOdom = findTransform(params->sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		
		mapper->processInput(input, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getSensorPose();
		
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		
		PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, params->sensorFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
		
		trajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
	}
	catch(tf2::TransformException& ex)
	{
		std::cerr << ex.what() << std::endl;
		return;
	}
}

void processBag(const std::string& bagPath)
{
	rosbag::Bag bag;
	bag.open(bagPath);
	
	ros::Time earliestStamp, latestStamp;
	populateTfBuffer(bag, earliestStamp, latestStamp);
	
	for(rosbag::MessageInstance const message: rosbag::View(bag, rosbag::TopicQuery(params->pointsIn)))
	{
		sensor_msgs::PointCloud2::ConstPtr cloudMsgIn = message.instantiate<sensor_msgs::PointCloud2>();
		if(cloudMsgIn != nullptr)
		{
			std::cout << std::to_string((double)((cloudMsgIn->header.stamp - earliestStamp).toNSec()) / (latestStamp - earliestStamp).toNSec() * 100) + "%"
					  << std::endl;
			gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(*cloudMsgIn), cloudMsgIn->header.stamp);
		}
		
		sensor_msgs::LaserScan::ConstPtr scanMsgIn = message.instantiate<sensor_msgs::LaserScan>();
		if(scanMsgIn != nullptr)
		{
			std::cout << std::to_string((double)((scanMsgIn->header.stamp - earliestStamp).toNSec()) / (latestStamp - earliestStamp).toNSec() * 100) + "%"
					  << std::endl;
			gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(*scanMsgIn), scanMsgIn->header.stamp);
		}
	}
	
	bag.close();
}

int main(int argc, char** argv)
{
	if(argc < 3)
	{
		std::cerr << "No bag path and/or no config file path specified in program arguments, exiting..." << std::endl;
		std::cout << "USAGE: rosbag_mapper BAG_PATH CONFIG_FILE_PATH" << std::endl;
		return 1;
	}
	std::string bagPath = argv[1];
	std::string configFilePath = argv[2];
	params = std::unique_ptr<MapperParameters>(new MapperParameters(configFilePath));
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::DURATION_MAX));
	mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(new norlab_icp_mapper::Mapper(params->icpConfig, params->inputFiltersConfig,
																					  params->mapPostFiltersConfig, params->mapUpdateCondition,
																					  params->mapUpdateOverlap, params->mapUpdateDelay,
																					  params->mapUpdateDistance,
																					  params->minDistNewPoint, params->sensorMaxRange, params->priorDynamic,
																					  params->thresholdDynamic, params->beamHalfAngle, params->epsilonA,
																					  params->epsilonD, params->alpha, params->beta, params->is3D, false,
																					  params->computeProbDynamic, params->isMapping));
	int euclideanDimension = 2;
	if(params->is3D)
	{
		euclideanDimension = 3;
	}
	trajectory = std::unique_ptr<Trajectory>(new Trajectory(euclideanDimension));
	odomToMap = PM::TransformationParameters::Identity(euclideanDimension + 1, euclideanDimension + 1);
	
	processBag(bagPath);
	
	mapper->getMap().save(params->finalMapName);
	trajectory->save(params->finalTrajectoryName);
	
	return 0;
}
