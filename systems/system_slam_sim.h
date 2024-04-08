/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-07 14:57
#
# Filename:		system_slam_sim.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_SLAM_SIM_H_
#define _SYSTEM_SLAM_SIM_H_

#include "types/types.h"
#include "types/map.h"
//#include "motion/motion_estimation.h"
//#include "features/edge_point_extraction.h"
//#include "features/plane_extraction.h"
//#include "features/plane_segmentation.h"
//#include "features/line_extraction.h"
#include "features/geometric_feature_matching.h"
#include "evaluate/relative_pose_error.h"
#include <sys/time.h>

namespace ulysses
{
//	void vis_addScan(Scan *scan, Transform Twc, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, bool flag=true);

	class SystemSLAMSim
	{
	public:
		SystemSLAMSim(const std::string &settingFile);
		~SystemSLAMSim()
		{
			delete scan_cur;
			delete map;
			delete feature_matching;
	//		if(plane_method==1) delete plane_extraction;
	//		else if(plane_method==2) delete plane_segmentation;
	//		delete line_extraction;
	//		delete edge_extraction;
	//		delete motion_estimation;
	//		delete global_map;
		}

//		bool trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
//						boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag=false);
//		void showScans(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
//						boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag);
//		void saveTraj(const std::string &file);

//		void useShadow(bool p) {motion_estimation->useShadow(p);}
//		void useLineResidual(bool p) {motion_estimation->useLineResidual(p);}

		Scan* getCurrentScan() const {return scan_cur;}
		Map* getMap() const {return map;}
//		std::string getFolder() const {return save_folder;}

		void saveMap() const 
		{
			if(!debug)
			{
				map->save(save_folder);
				map->saveTimes(save_folder);
			}
		}
//		const Transform getCurrentCamera() const 
//		{
//			return scan_cur->Tcg;
////			if(map->cameras.size()==0)
////			{
////				Transform Identity;
////				return Identity;
////			}
////			else
////				return map->cameras[map->cameras.size()-1];
//		}
//		void setDebug(bool d)
//		{
//			plane_extraction->setDebug(d);
//			line_extraction->setDebug(d);
////			plane_matching->setDebug(d);
////			line_mathing->setDebug(d);
//			motion_estimation->setDebug(d);
//		}
		double track(const double &time_stamp);

	private:

//		void loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth);

		bool debug;
		std::ofstream fp;
		std::string load_folder, save_folder;
		double current_time;
		bool isFirstFrame;

		Scan *scan_cur, *scan_ref;
		Map *map;
		Map *map_true;

//		std::vector<double> timestamps;
//		std::vector<Transform> traj;

//		int num_scan;
//		int optimize_frames;

//		int plane_method;
//		PlaneExtraction *plane_extraction;
//		PlaneSegmentation *plane_segmentation;
//		LineExtraction *line_extraction;
//		EdgePointExtraction *edge_extraction;
		GeometricFeatureMatching *feature_matching;
//		MotionEstimation *motion_estimation;
//		GlobalMap *global_map;
		RelativePoseError *rpe;

	};
}

#endif
