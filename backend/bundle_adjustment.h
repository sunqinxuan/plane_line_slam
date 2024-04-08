/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified: 2018-12-27 20:15
#
# Filename: bundle_adjustment.h
#
# Description: 
#
===============================================*/
#pragma once
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/pcl_painter2D.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <limits>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <eigen3/Eigen/Eigenvalues>
#include "ANN/ANN.h"
#include "types.h"

namespace ulysses
{
	class BundleAdjustment
	{
	public:

		BundleAdjustment()
		{
			remove("bundle_adjustment.txt");
		}

		~BundleAdjustment()
		{}

		void setDebug(bool d) {debug=d;}
		void setVisual(bool v) {visual=v;}

		void addScan2Map(Map *map, Scan *scan);

	private:

		bool debug;
		bool visual;
		std::ofstream fp;
	};
}
