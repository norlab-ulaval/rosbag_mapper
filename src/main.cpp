#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Mapper.h>
#include "Trajectory.h"
#include <rapidjson/document.h>
#include "rapidjson/filereadstream.h"
#include <iostream>

typedef norlab_icp_mapper::PM PM;
typedef norlab_icp_mapper::T T;

std::string pointsIn = "/velodyne_points";
std::string finalMapName = "map.vtk";
std::string finalTrajectoryName = "trajectory.vtk";
std::string odomFrame = "odom";
std::string sensorFrame = "velodyne";
std::string robotFrame = "base_link";
std::string icpConfig = "";
std::string inputFiltersConfig = "";
std::string mapPostFiltersConfig = "";
std::string mapUpdateCondition = "overlap";
float mapUpdateOverlap = 0.9;
float mapUpdateDelay = 1;
float mapUpdateDistance = 0.5;
float minDistNewPoint = 0.03;
float sensorMaxRange = 80;
float priorDynamic = 0.6;
float thresholdDynamic = 0.9;
float beamHalfAngle = 0.01;
float epsilonA = 0.01;
float epsilonD = 0.01;
float alpha = 0.8;
float beta = 0.99;
bool is3D = true;
bool computeProbDynamic = false;
bool isMapping = true;

std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
std::unique_ptr<Trajectory> trajectory;
PM::TransformationParameters odomToMap;

void parseConfig()
{
	FILE* fileStream = fopen("/home/simon/catkin_ws/src/rosbag_mapper/config.json", "r");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fileStream, readBuffer, sizeof(readBuffer));
	rapidjson::Document document;
	document.ParseStream(is);
	fclose(fileStream);
	
	if(document.FindMember("points_in") != document.MemberEnd())
	{
		pointsIn = document["points_in"].GetString();
	}
	
	if(document.FindMember("final_map_file_name") != document.MemberEnd())
	{
		finalMapName = document["final_map_file_name"].GetString();
	}
	
	if(document.FindMember("final_trajectory_file_name") != document.MemberEnd())
	{
		finalTrajectoryName = document["final_trajectory_file_name"].GetString();
	}
	
	if(document.FindMember("odom_frame") != document.MemberEnd())
	{
		odomFrame = document["odom_frame"].GetString();
	}
	
	if(document.FindMember("sensor_frame") != document.MemberEnd())
	{
		sensorFrame = document["sensor_frame"].GetString();
	}
	
	if(document.FindMember("robot_frame") != document.MemberEnd())
	{
		robotFrame = document["robot_frame"].GetString();
	}
	
	if(document.FindMember("icp_config") != document.MemberEnd())
	{
		icpConfig = document["icp_config"].GetString();
	}
	
	if(document.FindMember("input_filters_config") != document.MemberEnd())
	{
		inputFiltersConfig = document["input_filters_config"].GetString();
	}
	
	if(document.FindMember("map_post_filters_config") != document.MemberEnd())
	{
		mapPostFiltersConfig = document["map_post_filters_config"].GetString();
	}
	
	if(document.FindMember("map_update_condition") != document.MemberEnd())
	{
		mapUpdateCondition = document["map_update_condition"].GetString();
	}
	
	if(document.FindMember("map_update_overlap") != document.MemberEnd())
	{
		mapUpdateOverlap = document["map_update_overlap"].GetFloat();
	}
	
	if(document.FindMember("map_update_delay") != document.MemberEnd())
	{
		mapUpdateDelay = document["map_update_delay"].GetFloat();
	}
	
	if(document.FindMember("map_update_distance") != document.MemberEnd())
	{
		mapUpdateDistance = document["map_update_distance"].GetFloat();
	}
	
	if(document.FindMember("min_dist_new_point") != document.MemberEnd())
	{
		minDistNewPoint = document["min_dist_new_point"].GetFloat();
	}
	
	if(document.FindMember("sensor_max_range") != document.MemberEnd())
	{
		sensorMaxRange = document["sensor_max_range"].GetFloat();
	}
	
	if(document.FindMember("prior_dynamic") != document.MemberEnd())
	{
		priorDynamic = document["prior_dynamic"].GetFloat();
	}
	
	if(document.FindMember("threshold_dynamic") != document.MemberEnd())
	{
		thresholdDynamic = document["threshold_dynamic"].GetFloat();
	}
	
	if(document.FindMember("beam_half_angle") != document.MemberEnd())
	{
		beamHalfAngle = document["beam_half_angle"].GetFloat();
	}
	
	if(document.FindMember("epsilon_a") != document.MemberEnd())
	{
		epsilonA = document["epsilon_a"].GetFloat();
	}
	
	if(document.FindMember("epsilon_d") != document.MemberEnd())
	{
		epsilonD = document["epsilon_d"].GetFloat();
	}
	
	if(document.FindMember("alpha") != document.MemberEnd())
	{
		alpha = document["alpha"].GetFloat();
	}
	
	if(document.FindMember("beta") != document.MemberEnd())
	{
		beta = document["beta"].GetFloat();
	}
	
	if(document.FindMember("is_3D") != document.MemberEnd())
	{
		is3D = document["is_3D"].GetBool();
	}
	
	if(document.FindMember("compute_prob_dynamic") != document.MemberEnd())
	{
		computeProbDynamic = document["compute_prob_dynamic"].GetBool();
	}
	
	if(document.FindMember("is_mapping") != document.MemberEnd())
	{
		isMapping = document["is_mapping"].GetBool();
	}
}

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
		PM::TransformationParameters sensorToOdom = findTransform(sensorFrame, odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		
		mapper->processInput(input, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getSensorPose();
		
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		
		PM::TransformationParameters robotToSensor = findTransform(robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
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
	
	for(rosbag::MessageInstance const message: rosbag::View(bag, rosbag::TopicQuery(pointsIn)))
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
	if(argc < 2)
	{
		std::cerr << "No bag path specified in program arguments, exiting..." << std::endl;
		return 1;
	}
	std::string bagPath = argv[1];
	
	parseConfig();
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::DURATION_MAX));
	mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(new norlab_icp_mapper::Mapper(icpConfig, inputFiltersConfig, mapPostFiltersConfig, mapUpdateCondition,
																					  mapUpdateOverlap, mapUpdateDelay, mapUpdateDistance, minDistNewPoint,
																					  sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA,
																					  epsilonD, alpha, beta, is3D, false, computeProbDynamic,
																					  isMapping));
	int euclideanDimension = 2;
	if(is3D)
	{
		euclideanDimension = 3;
	}
	trajectory = std::unique_ptr<Trajectory>(new Trajectory(euclideanDimension));
	odomToMap = PM::TransformationParameters::Identity(euclideanDimension + 1, euclideanDimension + 1);
	
	processBag(bagPath);
	
	mapper->getMap().save(finalMapName);
	trajectory->save(finalTrajectoryName);
	
	return 0;
}
