/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-06 10:26
#
# Filename:		system_relocalization.cpp
#
# Description: 
#
************************************************/
#include "systems/system_relocalization.h"

namespace ulysses 
{
	SystemReLocalization::SystemReLocalization(const std::string &settingFile)
	{
		remove("SystemReLocalization.txt");
		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["debug"]>>debug;
		settings["loadFolder"]>>load_folder;
		settings.release();

		feature_matching=new GeometricFeatureMatching(settingFile);

		scan_cur=0;// scan_ref=0;
//		map=new Map;
		map_true=new Map;
		map_true->load(load_folder);

		timeval time;
		gettimeofday(&time,NULL);
		current_time=time.tv_sec+time.tv_usec*1e-6;

		if(!debug)
		{
			save_folder="relocalize_"+std::to_string(current_time);
			std::string mkdir_scans="mkdir -p "+save_folder+"/scans";
			if(system(mkdir_scans.c_str()));
		}
	}

	double SystemReLocalization::relocalize(const double &time_stamp)
	{
		fp.open("SystemReLocalization.txt",std::ios::app);
		fp<<std::endl<<"****************************SystemReLocalization::track()*******************************"<<std::endl;
//		timeval time_start, time_end;
		double time_used;

		scan_cur=new Scan(time_stamp);
		scan_cur->loadFeatures(load_folder);

		fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;

		time_used=feature_matching->match(scan_cur,map_true);
		scan_cur->localize();

		fp_reloc.open(save_folder+"/poses.txt",std::ios::app);
		fp_reloc.precision(6); fp_reloc<<std::fixed;
		fp_reloc<<scan_cur->time()<<" "<<scan_cur->Tcr()<<std::endl;
		fp_reloc.close();

		fp<<"estimated landmark associations"<<std::endl;
		scan_cur->association_map()->print(fp);

		LandmarkAssociation* fa=new LandmarkAssociation;
		fa->load(load_folder,scan_cur,map_true);
		fp<<"loaded associations"<<std::endl;
		fa->print(fp);
		double precision, recall;
		scan_cur->association_map()->evalulatePR(fa,precision,recall);
		fp<<"precision="<<precision<<std::endl;
		fp<<"recall="<<recall<<std::endl;
		if(!debug) scan_cur->savePRCMap(save_folder,fa);
		delete fa;

//			double precision, recall;
//			scan_cur->association()->evalulatePR(scan_cur->association_sim(),precision,recall);
//			fp<<std::endl;
//			fp<<"precision="<<precision<<std::endl;
//			fp<<"recall="<<recall<<std::endl;

		if(!debug) scan_cur->saveAssociationsMap(save_folder);

//		map->addCamera(scan_cur->Tcg(),scan_cur->time());
//		rpe->evaluate(time_stamp,map_true->camera(time_stamp),scan_cur->Tcg());

//		scan_ref=scan_cur;

		delete scan_cur;

		fp<<"***************************************************************************************"<<std::endl<<std::endl;
//		std::cout<<"scan_cur="<<scan_cur<<", "<<scan_cur->association_sim()<<std::endl;
		fp.close();
		return time_used;
	}


}
