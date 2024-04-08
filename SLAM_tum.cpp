/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-07 16:19
#
# Filename:		SLAM_tum.cpp
#
# Description: 
#
===============================================*/

#include "systems/system_slam_tum.h"
#include "systems/system_relocalization.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

int main(int argc, char *argv[])
{
	bool debug;
	double debug_time;

	std::string settingFile="settings.yaml";
	cv::FileStorage settings(settingFile,cv::FileStorage::READ);
	settings["debug"]>>debug;
	settings["debug_time"]>>debug_time;
	settings.release();

	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();

	std::ofstream fp;
	fp.open("slam_tum.txt");

	ulysses::SystemSLAMTum system("settings.yaml");
//	ulysses::SystemReLocalization relocalize("settings.yaml");

	for(int i=0;i<system.size();i+=10)
	{
//		if(debug) if(fabs(system.time(i)-debug_time)>1e-5) continue;

		fp<<std::endl<<i<<std::endl;
		cout<<std::endl<<i<<std::endl;
		cout<<"tracking: "<<system.track(system.time(i),vis)<<endl;
//		if(i>0) cout<<"relocalizing: "<<relocalize.relocalize(system.time(i))<<endl;
		Scan *scan=system.getCurrentScan();
		fp<<std::endl<<"scan.timestamp = "<<std::fixed<<scan->time()<<"-----------------------"<<std::endl;

//		system.visScan(vis);

//		if(debug) break;
	}

	system.saveMap();

	fp.close();

	return 0;

}

