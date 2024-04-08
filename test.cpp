/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-05 09:13
#
# Filename:		SLAM.cpp
#
# Description: 
#
===============================================*/

#include "types/types.h"
#include "types/map.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

int main(int argc, char *argv[])
{
	std::string load_folder;
	cv::FileStorage settings("settings.yaml",cv::FileStorage::READ);
	settings["loadFolder"]>>load_folder;
	settings.release();

	Scan *scan=new Scan;
	scan->points()=ptrPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointXYZRGBA pt;
	for(int i=0;i<100;i++)
	{
		for(int j=0;j<100;j++)
		{
			pt.x=i*0.1; pt.y=j*0.1; pt.z=10;
			pt.r=255; pt.g=0; pt.b=0;
			scan->points()->push_back(pt);
		}
	}

	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();

	vis->addPointCloud(scan->points(),"scan");
	vis->spin();

	return 0;

}

