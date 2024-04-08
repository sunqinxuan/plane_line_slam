/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-18 08:52
#
# Filename:		line_extraction.h
#
# Description: 
#
************************************************/

#ifndef _LINE_EXTRACT_H_
#define _LINE_EXTRACT_H_

#include "types/types.h"
#include "types/map.h"
#include "features/edge_point_extraction.h"
#include "LSD/lsd.h"
//#include "LBD/LineMatchingAlgorithm.hpp"
#include "LBD/LineDescriptor.hh"
#include "LBD/PairwiseLineMatching.hh"

namespace ulysses
{
	extern double THRES_RAD, THRES_DIST;

	class LineExtraction
	{
	public:

		LineExtraction() {}

		LineExtraction(const std::string &settingFile)
		{
			remove("line_extraction.txt");
			remove("line_matching_lbd.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			settings["debug"]>>debug;
			settings["LineExtraction.breakDist"]>>break_dist;
			settings["LineExtraction.inliers"]>>inliers;
			rho				   = (double)settings["LineExtraction.HoughLinesP.rho"]; 
			theta			   = (double)settings["LineExtraction.HoughLinesP.theta"]*M_PI/180.0; 
			threshold		   = (int)settings["LineExtraction.HoughLinesP.threshold"]; 
			minLineLength	   = (double)settings["LineExtraction.HoughLinesP.minLineLength"]; 
			maxLineGap		   = (double)settings["LineExtraction.HoughLinesP.maxLineGap"];
			thres_sim_dir	   = (double)settings["LineExtraction.thres_sim_dir"];
			thres_sim_dist	   = (double)settings["LineExtraction.thres_sim_dist"];
			thres_split  	   = (double)settings["LineExtraction.thres_split"];
			min_points_on_line = (int)settings["LineExtraction.min_points_on_line"];
//			thres_line2shadow  = (double)settings["LineExtraction.thres_line2shadow"];
//			thres_shadow2plane = (double)settings["LineExtraction.thres_shadow2plane"];
			settings.release();
			edge_extraction=new EdgePointExtraction(settingFile);
			line_descriptor=new LineDescriptor;
			line_match=new PairwiseLineMatching;
		}

		~LineExtraction() 
		{
			delete edge_extraction; 
			delete line_descriptor; 
			delete line_match;
		}
			

		void setDebug(bool d) {debug=d;}

		double extractLines(Scan *scan);

		void visLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//		void fitLines(Scan *scan);
//		bool fitSphere(EdgePoint *edge_point);

		double extract(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double extract_lbd(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double match_lbd(Scan *scan, Scan *ref);

		void computeLineInfo(Line *line);

	private:
		
		std::ofstream fp;
		bool debug;
		bool visual;
		
//		size_t height,width;

		double break_dist;
		int inliers;

		double rho; 
		double theta; 
		int threshold; 
		double minLineLength; 
		double maxLineGap;

		double thres_sim_dir, thres_sim_dist;
		double thres_split;
		int min_points_on_line;

//		double thres_line2shadow;
//		double thres_shadow2plane;

		EdgePointExtraction *edge_extraction;
		LineDescriptor *line_descriptor;
		PairwiseLineMatching *line_match;
//		ScaleLines key_lines;

		void extractLinesHough(Scan* scan, double scale);

//		void generatePlucker1(Line *line);
		void generatePlucker(Line *line);

		struct linePoint
		{
			linePoint(int i, Eigen::Vector3d p) : index(i), point(p) {}
			int index;
			Eigen::Vector3d point;
			double quality;
			void setQuality(linePoint* p1, linePoint* p2)
			{
				Eigen::Vector3d d1=point-p1->point;
				Eigen::Vector3d d2=point-p2->point;
				quality=d1.norm()+d2.norm();
			}
			void setQuality(linePoint* p1)
			{
				Eigen::Vector3d d1=point-p1->point;
				quality=d1.norm();
			}
			friend std::ostream & operator << (std::ostream &os, const linePoint *p)
			{
				os<<p->point.transpose()<<"\t"<<p->quality;
				return os;
			}
		};
	};

}

#endif
