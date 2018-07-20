#ifndef LaneChange_H
#define LaneChange_H
#pragma once
#include<iostream>

//#ifndef LaneChange
//#define LaneChange

using namespace std;

class Lane
{
public:
	int lane_number;
	Lane(int lane_number);
	bool LaneShift(std::vector<vector<double>> sensor_fusion, double end_path_s, double end_path_d);
	double LaneSpeed(std::vector<vector<double>> sensor_fusion, double end_path_s, double end_path_d);
	//~Lane();

private:

};

#endif