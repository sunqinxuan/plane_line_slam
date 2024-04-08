/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-07 09:18
#
# Filename:		plane_extraction_base.h
#
# Description: 
#
************************************************/

#ifndef _PLANE_EXTRACTION_H_
#define _PLANE_EXTRACTION_H_

#include "types/types.h"
#include "types/map.h"

namespace ulysses
{

	class PlaneExtractionBase
	{
	public:
		PlaneExtractionBase() {}

		PlaneExtractionBase(const std::string &settingFile)
		{ 
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			settings["debug"]>>debug;
			settings.release();
			remove("plane_extraction.txt");
		}

		~PlaneExtractionBase()
		{
		}

		void setDebug(bool d) {debug=d;}

		virtual double extractPlanes(Scan *scan) =0 ;

		void visPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void computePlaneInfo(Plane *plane);

	protected:

		bool debug;
		std::ofstream fp;

		// plane->inliers should be extracted before calling this;
		void fitPlaneModel(Plane *plane);

		void computeCentroidScatter(Plane *plane);
	};
}

#endif
