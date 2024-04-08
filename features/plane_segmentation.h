/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-07 14:22
#
# Filename:		plane_segmentation.h
#
# Description: 
#
************************************************/

#include "types/types.h"
#include "types/map.h"
#include "features/plane_extraction_base.h"

namespace ulysses
{	
	extern double THRES_RAD, THRES_DIST;

	class PlaneSegmentation : public PlaneExtractionBase//, public pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		PlaneSegmentation(const std::string &settingFile) : PlaneExtractionBase(settingFile)
		{
			cv::FileStorage settings(settingFile,cv::FileStorage::READ);
			settings["PlaneSegmentation.min_plane_size"]>>inliers;
			settings["PlaneSegmentation.thres_angle"]>>ang;
			settings["PlaneSegmentation.thres_dist"]>>dist;
            settings["PlaneSegmentation.thres_curv"]>>curv;
			settings.release();

//			setMinInliers(inliers);
//			setAngularThreshold(pcl::deg2rad(ang));
//			setDistanceThreshold(dist); 
//			setMaximumCurvature(curv);
		}

		double extractPlanes(Scan *scan);

	private:

		int inliers;
		double ang, dist, curv;

		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> *plane_segment;

	};
}
