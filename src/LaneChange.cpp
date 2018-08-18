#include<iostream>
#include "LaneChange.h"

using namespace std;


Lane::Lane(int lane_number)
{
	this->lane_number = lane_number;
}

Lane::~Lane()
{
}

bool Lane::LaneShift(std::vector<vector<double>> sensor_fusion, double end_path_s, double end_path_d)
{
	bool possible = true;
	double front_dist = 10;
	double back_dist = 30;

	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		double sense_d = sensor_fusion[i][6];
		double sense_s = sensor_fusion[i][5];


		//Ego Vehicle Lane
		if ((sense_d > 4 * lane_number) && (sense_d < 4 * (lane_number + 1)))
		{
			if ((sense_s - end_path_s) < front_dist && (sense_s - end_path_s) > -back_dist)
			{
				possible = false;
			}
		}
	}

	return(possible);
}

double Lane::LaneSpeed(std::vector<vector<double>> sensor_fusion, double end_path_s, double end_path_d)
{

	//If there are no cars in front then the lane speed would be set to 
	double highest_speed = 0.0;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		double sense_d = sensor_fusion[i][6];
		double sense_s = sensor_fusion[i][5];


		//Ego Vehicle Lane
		if ((sense_d > 4 * lane_number) && (sense_d < 4 * (lane_number + 1)))
		{
			if ((sense_s - end_path_s) > 0.0)
			{
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];

				double highest_speed = sqrt(pow(vx, 2) + pow(vy, 2));
			}
		}
	}
	return(highest_speed);
}

