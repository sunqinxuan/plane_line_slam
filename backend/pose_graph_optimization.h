/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-11-07 18:55
#
# Filename: pose_graph_optimization.h
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
	class PoseGraphOptimization
	{
	public:

		PoseGraphOptimization() 
		{
			debug=false;
			max_iter_g2o=10;
		}

		~PoseGraphOptimization() {}

		void setDebug(bool d) {debug=d;}

		double optimize(Map *map);

	private:

		bool debug;

		int max_iter_g2o;

	};
}
