#include "MapperParameters.h"
#include <rapidjson/document.h>
#include "rapidjson/filereadstream.h"

MapperParameters::MapperParameters(const std::string& configFilePath):
		pointsIn("/velodyne_points"), finalMapName("map.vtk"), finalTrajectoryName("trajectory.vtk"), odomFrame("odom"), sensorFrame("velodyne"),
		robotFrame("base_link"), icpConfig(""), inputFiltersConfig(""), mapPostFiltersConfig(""), mapUpdateCondition("overlap"),
		mapUpdateOverlap(0.9), mapUpdateDelay(1), mapUpdateDistance(0.5), minDistNewPoint(0.03), sensorMaxRange(80), priorDynamic(0.6), thresholdDynamic(0.9),
		beamHalfAngle(0.01), epsilonA(0.01), epsilonD(0.01), alpha(0.8), beta(0.99), is3D(true), computeProbDynamic(false), isMapping(true)
{
	parseConfig(configFilePath);
}

void MapperParameters::parseConfig(const std::string& configFilePath)
{
	FILE* fileStream = fopen(configFilePath.c_str(), "r");
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
