/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-11-06 21:03
#
# Filename:		loop_closing.h
#
# Description: 
#
===============================================*/

#pragma once
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <opencv2/core/core.hpp>
#include "ANN/ANN.h"
#include "types.h"

namespace ulysses
{
	class LoopClosing
	{
	public:
		LoopClosing() 
		{
			delta_time=10;
			delta_angle=0.1;//5.73deg
			sqRad_ANN=0.01;//radius=0.1m;
			K_ANN=10;
		}

		~LoopClosing() {}

		void setDebug(bool d) {debug=d;}
		void setDeltaTime(double d) {delta_time=d;}
		void setDeltaAngle(double d) {delta_angle=d*M_PI/180.0;}
		void setDeltaDist(double d) {sqRad_ANN=d*d;}

		bool findLoops(Map *map, Scan *scan);

	private:

		bool debug;

		double sqRad_ANN;
		int K_ANN;

		double delta_time;
		double delta_angle;
	};
}
