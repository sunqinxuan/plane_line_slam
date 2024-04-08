/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-04 19:52
#
# Filename:		system_plane_line_shadow.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "types/types.h"
#include "types/map.h"
#include "motion/motion_estimation.h"
#include "features/edge_point_extraction.h"
#include "features/plane_extraction.h"
#include "features/plane_segmentation.h"
#include "features/line_extraction.h"
#include "features/geometric_feature_matching.h"
#include <sys/time.h>

namespace ulysses
{
	void vis_addScan(Scan *scan, Transform Twc, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, bool flag=true);

	class SystemPlaneLine
	{
	public:
		SystemPlaneLine(const std::string &seq_path, const std::string &setting_file);

		bool trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
						boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag=false);

		void showScans(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
						boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag);

		void shutDown();

		void saveTraj(const std::string &file);

//		void useShadow(bool p) {motion_estimation->useShadow(p);}
		void useLineResidual(bool p) {motion_estimation->useLineResidual(p);}

		Scan* getCurrentScan() const {return scan_cur;}
		const Transform getCurrentCamera() const 
		{
			return scan_cur->Tcg;
//			if(map->cameras.size()==0)
//			{
//				Transform Identity;
//				return Identity;
//			}
//			else
//				return map->cameras[map->cameras.size()-1];
		}
		void setDebug(bool d)
		{
			plane_extraction->setDebug(d);
			line_extraction->setDebug(d);
//			plane_matching->setDebug(d);
//			line_mathing->setDebug(d);
			motion_estimation->setDebug(d);
		}
	protected:
		Scan *scan_cur, *scan_ref;

		void loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth);

	private:
		Map *map;

		std::vector<double> timestamps;
		std::vector<Transform> traj;

		int num_scan;
		bool isFirstFrame;
		int optimize_frames;

		int plane_method;

		PlaneExtraction *plane_extraction;
		PlaneSegmentation *plane_segmentation;
		LineExtraction *line_extraction;
		EdgePointExtraction *edge_extraction;
		GeometricFeatureMatching *feature_matching;
		MotionEstimation *motion_estimation;
		GlobalMap *global_map;

	};
}

#endif
