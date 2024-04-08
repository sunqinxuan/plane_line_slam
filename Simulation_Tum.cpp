/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-21 08:51
#
# Filename:		Simulation_Tum.cpp
#
# Description: 
#
===============================================*/

#include "systems/system_simulation_tum.h"
//#include "systems/system_relocalization.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

bool stop;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "q" && event.keyDown ())
	{
	   viewer->close();
	}
	if (event.getKeySym () == "c" && event.keyDown ())
	{
		stop=true;
	}
}

int main(int argc, char *argv[])
{
	stop=false;
	bool debug;
	double debug_time;
	int start, end, interval;
	std::string mode;

	std::string settingFile="settings.yaml";
	cv::FileStorage settings(settingFile,cv::FileStorage::READ);
	settings["mode"]>>mode;
	settings["debug"]>>debug;
	settings["debug_time"]>>debug_time;
	settings["start"]>>start;
	settings["end"]>>end;
	settings["interval"]>>interval;
	settings.release();

	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());

	ulysses::SystemSimulationTum system("settings.yaml");
	if(end==0) end=system.size();

	if(mode=="test")
	{
		system.test(vis);
	}
	else if(mode=="associate")
	{
		system.associate(vis);
	}
	else if(mode=="localize")
	{
		system.localize(vis);
	}
	else if(mode=="simulate")
	{
		system.simulate(vis);
	}
	else if(mode=="optimize")
	{
		system.optimize(vis);
	}
	else if(mode=="optimize_map")
	{
		system.optimize_map(vis);
	}

	return 0;

}

