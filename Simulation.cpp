/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-04 16:22
#
# Filename:		Simulation.cpp
#
# Description: 
#
===============================================*/

#include <sys/time.h>
#include "systems/system_simulation.h"
//#include "evaluate/relative_pose_error.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

int main(int argc, char *argv[])
{
	bool loadSavedData;
	int num_scans;
	std::string folder;

	std::string settingFile="settings.yaml";
	cv::FileStorage settings(settingFile,cv::FileStorage::READ);
	settings["loadSavedData"]>>loadSavedData;
	settings["loadFolder"]>>folder;
	settings["NumScan"]>>num_scans;
	settings.release();

//	ulysses::Map *m=new ulysses::Map;
//	m->load(folder);
//	m->save("save");
//	std::vector<Scan*> scans;
//	scans.resize(m->sizeCamera());
//	for(int i=0;i<m->sizeCamera();i++)
//	{
//		scans[i]=new Scan(m->time(i));
//		scans[i]->loadFeatures(folder);
//		scans[i]->saveFeatures("save");
//		if(i>0)
//		{
//			scans[i]->ref()=scans[i-1];
//			scans[i]->loadAssociations(folder,m);
//			scans[i]->saveAssociations("save");
//		}
//	}
//	return 0;


	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();

	std::ofstream fp;
	fp.open("simulation.txt");

	ulysses::SystemSimulation system("settings.yaml");
	Scan *scan;
	srand((unsigned)time(NULL));
	ulysses::Map* map=system.getMap();
	for(int i=0;i<num_scans;i++)
	{
		system.simulate();
		scan=system.getCurrentScan();
		fp<<std::endl<<"scan.timestamp = "<<std::fixed<<scan->time()<<"-----------------------"<<std::endl;
		fp<<std::endl<<"landmarks"<<std::endl;
		map->printLandmarks(fp);
////		std::cout<<"scan="<<scan<<", "<<scan->association_sim()<<std::endl;
//		scan->saveFeatures(std::to_string(scan->time()));
//		if(i>0) 
//		{
//			std::cout<<"feature associations"<<std::endl;
//			scan->association_sim()->printAssociatedFeatures(std::cout);
//			scan->saveAssociations("association_sim_"+std::to_string(scan->time()));
//		}
//		scan->printPlanes(fp);
//		scan->printLines(fp);
	}

//	system.visTrajSim(vis);
//
	map->save(system.getFolder());
	std::cout<<"saved map "<<std::endl;

//	map->save(system.getFolder()+"/ulysses");
//	std::cout<<"saved estimated map "<<std::endl;
//
	map->saveTimes(system.getFolder());
	std::cout<<"saved times "<<std::endl;

	fp.close();

	return 0;

}

//			std::string path=seq_path+"camera/";
//			DIR* dir=opendir(path.c_str());
//			dirent* ptr=NULL;
//			std::vector<std::string> indices;
//			while((ptr=readdir(dir))!=NULL)
//			{
//				if(ptr->d_name[0]!='.')
//				{
////					std::string cur_idx=std::string(ptr->d_name);
////					std::string name=path+std::string(ptr->d_name);
////					std::cout<<name<<std::endl;
//					indices.push_back(std::string(ptr->d_name));
//				}
//			}

