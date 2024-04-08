/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-08-05 08:54
#
# Filename:		Kinect2QualisysCalibration.cpp
#
# Description: 
#
===============================================*/

#include "systems/system_extrinsic_calibration.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

int main(int argc, char *argv[])
{
	vtkObject::GlobalWarningDisplayOff();
	// usage: bin/Kinect2QualisysCalibration file_settings
	// argv[1] - setting file;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();

	ulysses::SystemExtrinsicCalibration system(argv[1],argv[2]);
	system.calibrate(argv[3]);
	system.visChessCalib(vis);
//	system.visStatic(vis);

	system.shutDown();

	return 0;
}

