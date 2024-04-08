/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2018-02-03 11:14
#
# Filename: factor_graph.h
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
#include "types.h"

namespace ulysses
{
	class Vertex_Plane : public g2o::BaseVertex<3, Eigen::Vector4d>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Vertex_Plane() {}
		virtual bool read(std::istream& is) {}
		virtual bool write(std::ostream& os) const {}

		virtual void setToOriginImpl() {}

		virtual void oplusImpl(const double *update);
	};

	class Edge_CamPlane : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap, Vertex_Plane>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Edge_CamPlane() {}
		virtual bool read(std::istream& is) {}
		virtual bool write(std::ostream& os) const {}

		virtual void computeError();
	};

	class FactorGraph
	{
	public:
		FactorGraph() {}

		void optimizeGraph(Map *map);
	private:
	};
}
