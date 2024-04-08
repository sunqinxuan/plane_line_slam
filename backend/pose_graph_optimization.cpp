/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-11-07 18:55
#
# Filename: pose_graph_optimization.cpp
#
# Description: 
#
===============================================*/
#include "pose_graph_optimization.h"

namespace ulysses
{
	double PoseGraphOptimization::optimize(Map *map)
	{
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock (std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver));
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (std::unique_ptr<DirectBlock>(solver_ptr)); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose(true);

		for(size_t i=0;i<map->scans.size();i++)
		{
			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			g2o::SE3Quat Tcg(map->scans[i]->Tcg.R,map->scans[i]->Tcg.t);
//			g2o::SE3Quat Tcg(map->scans[i]->Tcg_gt.R,map->scans[i]->Tcg_gt.t);
//			pose->setEstimate(Tcg.inverse());
			pose->setEstimate(Tcg);
			pose->setId(i);
			pose->setFixed(false);
			if(i==0) pose->setFixed(true);
			optimizer.addVertex(pose);

			if(i>0)
			{
				int id_cur=map->scans[i]->id;
				int id_ref=map->scans[i]->scan_ref->id;
				g2o::VertexSE3Expmap* v0 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[id_cur]);
				g2o::VertexSE3Expmap* v1 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[id_ref]);
				g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
				edge->setVertex(0,v0);
				edge->setVertex(1,v1);
				g2o::SE3Quat Tcr(map->scans[i]->Tcr.R,map->scans[i]->Tcr.t);
				edge->setMeasurement(Tcr.inverse());
				edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
				edge->setId(i);
				optimizer.addEdge(edge);
			}
		}

		for(size_t i=0;i<map->loop_closure.size();i++)
		{
			if(map->loop_closure[i].match_score==-1)
				continue;
			int id_cur=map->loop_closure[i].scan_cur->id;
			int id_ref=map->loop_closure[i].scan_ref->id;
			g2o::VertexSE3Expmap* v0 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[id_cur]);
			g2o::VertexSE3Expmap* v1 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[id_ref]);
			g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
			edge->setVertex(0,v0);
			edge->setVertex(1,v1);
			g2o::SE3Quat Tcr(map->loop_closure[i].Tcr.R,map->loop_closure[i].Tcr.t);
			edge->setMeasurement(Tcr.inverse());
			edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
			edge->setId(i+map->scans.size());
			optimizer.addEdge(edge);
		}

		optimizer.initializeOptimization();
		int result=optimizer.optimize ( max_iter_g2o );

		for(size_t i=0;i<map->scans.size();i++)
		{
			g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[i]);
//			g2o::SE3Quat Tcg=v->estimate().inverse();
			g2o::SE3Quat Tcg=v->estimate();
			map->scans[i]->Tcg.R=Tcg.rotation().toRotationMatrix();
			map->scans[i]->Tcg.t=Tcg.translation();
		}

		double chi2=optimizer.activeChi2();
		optimizer.clear();

		return chi2;
	}


}
