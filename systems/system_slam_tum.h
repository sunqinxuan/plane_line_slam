/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-07 15:02
#
# Filename:		system_slam_tum.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_SLAM_TUM_H_
#define _SYSTEM_SLAM_TUM_H_

#include "types/types.h"
#include "types/map.h"
//#include "motion/motion_estimation.h"
//#include "features/plane_extraction.h"
#include "features/plane_segmentation.h"
#include "features/line_extraction.h"
#include "features/geometric_feature_matching.h"
#include "evaluate/relative_pose_error.h"
#include <sys/time.h>

namespace ulysses
{
	extern double THRES_RAD, THRES_DIST;
	extern CameraIntrinsic camera_intrinsic;

	class SystemSLAMTum
	{
	public:
		SystemSLAMTum(const std::string &settingFile);
		~SystemSLAMTum()
		{
			delete scan_cur;
			delete map;
			delete feature_matching;
			delete plane_segmentation;
	//		if(plane_method==1) delete plane_extraction;
	//		else if(plane_method==2) delete plane_segmentation;
			delete line_extraction;
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
		double time(int i) const {return timestamps[i];}
		int size() const {return timestamps.size();}
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
//
		double track(const double &time_stamp, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void visScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//		{
//			if (!vis->updatePointCloud (scan_cur->points(),"s"))
//				vis->addPointCloud (scan_cur->points(),"s");
//			plane_segmentation->visPlanes(scan_cur,vis);
//			vis->spin();
//		}

//		void loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth);

	private:

		bool debug;
		std::ofstream fp;
		std::string save_folder;
		double current_time;
		bool isFirstFrame;

		Scan *scan_cur, *scan_ref;
		Map *map;
//		Map *map_true;

//		std::vector<double> timestamps;
//		std::vector<Transform> traj;

//		int num_scan;
//		int optimize_frames;

//		int plane_method;
//		PlaneExtraction *plane_extraction;
		PlaneSegmentation *plane_segmentation;
		LineExtraction *line_extraction;
		GeometricFeatureMatching *feature_matching;
//		MotionEstimation *motion_estimation;
//		GlobalMap *global_map;
		RelativePoseError *rpe;

		std::vector<double> timestamps;
		std::map<double,std::string> files_depth;
		std::map<double,std::string> files_rgb;
		std::map<double,Transform> Tcw_truth;

		void loadImages(const std::string &folder);
	};
}

#endif
