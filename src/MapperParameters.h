#ifndef ROSBAG_MAPPER_MAPPERPARAMETERS_H
#define ROSBAG_MAPPER_MAPPERPARAMETERS_H

#include <string>

class MapperParameters
{
public:
	MapperParameters(const std::string& configFilePath);
	
	std::string pointsIn;
	std::string finalMapName;
	std::string finalTrajectoryName;
	std::string odomFrame;
	std::string sensorFrame;
	std::string robotFrame;
	std::string icpConfig;
	std::string inputFiltersConfig;
	std::string mapPostFiltersConfig;
	std::string mapUpdateCondition;
	float mapUpdateOverlap;
	float mapUpdateDelay;
	float mapUpdateDistance;
	float minDistNewPoint;
	float sensorMaxRange;
	float priorDynamic;
	float thresholdDynamic;
	float beamHalfAngle;
	float epsilonA;
	float epsilonD;
	float alpha;
	float beta;
	bool is3D;
	bool computeProbDynamic;
	bool isMapping;

private:
	void parseConfig(const std::string& configFilePath);
};


#endif
