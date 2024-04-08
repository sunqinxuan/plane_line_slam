/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-11-06 21:03
#
# Filename:		loop_closing.cpp
#
# Description: 
#
===============================================*/

#include "loop_closing.h"

namespace ulysses
{
	bool LoopClosing::findLoops(Map *map, Scan *scan)
	{
		bool found_loop=false;
		ANNkd_tree *kdtree;
		ANNpoint cur_pose=annAllocPt(3);
		ANNpointArray map_poses=annAllocPts(map->scans.size(),3);
		Transform Tgc;
		for(size_t i=0;i<map->scans.size();i++)
		{
			Tgc=map->scans[i]->Tcg.inv();
			map_poses[i][0]=Tgc.t(0);
			map_poses[i][1]=Tgc.t(1);
			map_poses[i][2]=Tgc.t(2);
		}
		kdtree=new ANNkd_tree(map_poses,map->scans.size(),3);
		Tgc=scan->Tcg.inv();
		cur_pose[0]=Tgc.t(0);
		cur_pose[1]=Tgc.t(1);
		cur_pose[2]=Tgc.t(2);
		int candidates=kdtree->annkFRSearch(cur_pose,sqRad_ANN,0);
		if(candidates>0)
		{
			ANNidxArray index=new ANNidx[candidates];
			ANNdistArray distance=new ANNdist[candidates];
			candidates=kdtree->annkFRSearch(cur_pose,sqRad_ANN,candidates,index,distance);
//			std::cout<<"findLoops "<<std::endl;
			for(size_t i=0;i<candidates;i++)
			{
				if(index[i]==ANN_NULL_IDX)
					continue;
				LoopClosure loop(scan,map->scans[index[i]]);
				if(loop.delta_time<delta_time || acos((loop.Tcr.R.trace()-1.0)/2.0)>delta_angle)
					continue;
				map->loop_closure.push_back(loop);
				found_loop=true;
//				std::cout<<"\t"<<loop.delta_time<<"\t"<<acos((loop.Tcr.R.trace()-1.0)/2.0)<<"\t"<<loop.Tcr.t.norm()<<std::endl;
			}
			delete index;
			delete distance;
		}
		annDeallocPt(cur_pose);
		annDeallocPts(map_poses);
		delete kdtree;
		return found_loop;
	}
	
}
