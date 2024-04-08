/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-04 19:48
#
# Filename:		system_simulation.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_SIMULATE_H_
#define _SYSTEM_SIMULATE_H_

#include "types/types.h"
#include "types/map.h"
#include "features/geometric_feature_matching.h"
//#include "motion/motion_estimation.h"

namespace ulysses
{
	class SystemSimulation
	{
	public:
		SystemSimulation(const std::string &setting_file);
		~SystemSimulation();

		Scan* getCurrentScan() const {return scan_cur;}
		Map* getMap() const {return map;}
		std::string getFolder() const {return folder;}

		bool simulate();
//		bool estimate();

		void visTrajSim(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:
		
		bool debug;
		std::ofstream fp;
		bool loadSavedData, loadNoisyData;
		std::string filename_write, filename_read;
		std::string folder;

		double sigma, overlap;
		int NumPlane, NumLine;

//		int num_scan;
		Scan *scan_cur, *scan_ref;
		Map *map;
//		std::vector<double> timestamps;
//		std::vector<Transform> traj;

		// save the simulated data;
//		Scan *scan_cur_sim, *scan_ref_sim;
//		Map *map_sim;
		// time_delta - time interval (/s) during the simulation;
		double time_delta, current_time;
		// velocity described in the camera frame;
		Eigen::Vector3d linear_velocity, angular_velocity;
		// initial pose of the camera in the world frame;
		ulysses::Transform Tw0;

		GeometricFeatureMatching *feature_matching;
//		MotionEstimation *motion_estimation;
//		GlobalMap *global_map;

		bool generateScan(Scan *scan);
//		void addNoise2Plane(PlaneLM* lm, Plane *p);
//		void addNoise2Line(LineLM* lm, Line *l);

		Vector6d transformVelocity(const Eigen::Vector3d &v, const Eigen::Vector3d &w, const ulysses::Transform &Tgc)
		{
			Vector6d xi;
			xi.block<3,1>(0,0)=v;
			xi.block<3,1>(3,0)=w;
			Eigen::Matrix3d R=Tgc.R;
			Eigen::Vector3d t=Tgc.t;
			Matrix6d Adj=Matrix6d::Zero();
			Adj.block<3,3>(0,0)=R;
			Adj.block<3,3>(0,3)=Transform::skew_sym(t)*R;
			Adj.block<3,3>(3,3)=R;
			xi=Adj*xi;
			return xi;
		}

		Transform expmap(Vector6d xi, double theta)
		{
			Transform T;
			Eigen::Vector3d v=xi.block<3,1>(0,0);
			Eigen::Vector3d w=xi.block<3,1>(3,0);
			if(w.norm()<1e-6)
			{
				T.R.setIdentity();
				T.t=v*theta;
			}
			else
			{
				T.R=expmap_rot(w,theta);
				w.normalize();
				T.t=(Eigen::Matrix3d::Identity()-T.R)*Transform::skew_sym(w)*v+w*w.transpose()*v*theta;
			}
			return T;
		}

		Eigen::Matrix3d expmap_rot(Eigen::Vector3d w, double theta)
		{
			theta=theta*w.norm();
			if(w.norm()>1e-6)
			{
				w.normalize();
			}
			Eigen::Matrix3d R=Eigen::Matrix3d::Identity()+Transform::skew_sym(w)*sin(theta)
							  +Transform::skew_sym(w)*Transform::skew_sym(w)*(1-cos(theta));
			return R;
		}


	};
}

#endif
