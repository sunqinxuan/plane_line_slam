/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-06 10:26
#
# Filename:		system_relocalization.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_RELOC_H_
#define _SYSTEM_RELOC_H_

#include "types/types.h"
#include "types/map.h"
#include "features/geometric_feature_matching.h"

namespace ulysses
{
//	void vis_addScan(Scan *scan, Transform Twc, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, bool flag=true);

	class SystemReLocalization
	{
	public:
		SystemReLocalization(const std::string &settingFile);
		~SystemReLocalization()
		{
//			delete scan_cur;
			delete map_true;
			delete feature_matching;
		}

		Scan* getCurrentScan() const {return scan_cur;}
//		Map* getMap() const {return map;}

//		void saveMap() const 
//		{
//			if(!debug)
//			{
//				map->save(save_folder);
//				map->saveTimes(save_folder);
//			}
//		}
		double relocalize(const double &time_stamp);

	private:

		bool debug;
		std::ofstream fp, fp_reloc;
		std::string load_folder, save_folder;
		double current_time;
		bool isFirstFrame;

		Scan *scan_cur;//, *scan_ref;
//		Map *map;
		Map *map_true;

		GeometricFeatureMatching *feature_matching;

		void convertMapFeatures(const Map *map, Scan *scan)
		{
			scan->ref()=new Scan;
			for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
			{
				Feature *feat=new Feature(it->second);
				scan->ref()->addFeature(feat);
			}
		}

	};
}

#endif
