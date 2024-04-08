/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-07 16:19
#
# Filename:		SLAM_sim.cpp
#
# Description: 
#
===============================================*/

#include "systems/system_slam_sim.h"
#include "systems/system_relocalization.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

int main(int argc, char *argv[])
{
	bool debug;
	double debug_time;

	std::string folder;
	std::string settingFile="settings.yaml";
	cv::FileStorage settings(settingFile,cv::FileStorage::READ);
	settings["loadFolder"]>>folder;
	settings["debug"]>>debug;
	settings["debug_time"]>>debug_time;
	settings.release();

	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();

	std::vector<double> times;
	std::ifstream fp_times;
	fp_times.open(folder+"/timestamp.txt");
	while(!fp_times.eof())
	{
		std::string s; getline(fp_times,s);
		if(!s.empty())
		{
			std::stringstream ss; ss<<s;
			double t; ss>>t;
			times.push_back(t);
		}
	}
	fp_times.close();

	std::ofstream fp;
	fp.open("slam.txt");

	ulysses::SystemSLAMSim system("settings.yaml");
	ulysses::SystemReLocalization relocalize("settings.yaml");

	for(int i=0;i<times.size();i++)
	{
		fp<<std::endl<<i<<std::endl;
		cout<<std::endl<<i<<std::endl;
		cout<<"tracking: "<<system.track(times[i])<<endl;
		if(i>0) cout<<"relocalizing: "<<relocalize.relocalize(times[i])<<endl;
		Scan *scan=system.getCurrentScan();
		fp<<std::endl<<"scan.timestamp = "<<std::fixed<<scan->time()<<"-----------------------"<<std::endl;

		if(debug) if(fabs(times[i]-debug_time)<1e-5) break;
	}

	system.saveMap();


	fp.close();

	return 0;

}

