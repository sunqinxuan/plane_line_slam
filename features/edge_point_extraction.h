/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 15:16
#
# Filename:		edge_point_extraction.h
#
# Description: 
#
===============================================*/
#ifndef _EDGE_H_
#define _EDGE_H_

#include "types/types.h"
#include "types/map.h"

namespace ulysses
{

	class EdgePointExtraction : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		EdgePointExtraction() {}

		EdgePointExtraction(const std::string &settingFile)
		{ 
			remove("edge_point_extraction.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			settings["debug"]>>debug; 
			setDepthDisconThreshold ((double)settings["EdgePointExtraction.PCLOrganizedEdge.DepthDisconThreshold"]);
			setMaxSearchNeighbors ((int)settings["EdgePointExtraction.PCLOrganizedEdge.MaxSearchNeighbors"]);
			setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_HIGH_CURVATURE | EDGELABEL_RGB_CANNY); 
			settings.release();
//			thres_pxl_sq=(double)settings["EdgePointExtraction.thres_pxl"]*(double)settings["EdgePointExtraction.thres_pxl"];
//			thres_occluded_dist=(double)settings["EdgePointExtraction.thres_occluded_dist"];
//			sqRad_ANN=(double)settings["EdgePointExtraction.ANN_search_radius"]*(double)settings["EdgePointExtraction.ANN_search_radius"];
//			K_ANN=(int)settings["EdgePointExtraction.ANN_search_K"];
//			edge_meas=100;
//			thres_ratio=0.2;
//			thres_angle=0.2;
		}

		~EdgePointExtraction() {}

		void setDebug(bool d) {debug=d;}

		void extractEdgePoints(Scan *scan);

		void visEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:
		
		std::ofstream fp;
		bool debug;

//		double thres_pxl_sq;
//		double thres_occluded_dist;
//		double thres_ratio, thres_angle;
//
//		// radius=0.1m;
//		double sqRad_ANN;
//		// K=20;
//		int K_ANN;
//
//		double edge_meas;

	};

}

#endif
