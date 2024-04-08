/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-12-11 11:02
#
# Filename:		system_simulation_tum.h
#
# Description: 
#
************************************************/
#ifndef _SYSTEM_SIMULATION_TUM_H_
#define _SYSTEM_SIMULATION_TUM_H_

#include "types/types.h"
#include "types/map.h"
#include "motion/motion_estimation.h"
//#include "features/plane_extraction.h"
#include "features/plane_segmentation.h"
#include "features/line_extraction.h"
#include "features/geometric_feature_matching.h"
#include "features/it_geometric_feature_matching.h"
//#include "evaluate/relative_pose_error.h"
#include <sys/time.h>

extern bool stop;
namespace ulysses
{
	extern double THRES_RAD, THRES_DIST;
	extern CameraIntrinsic camera_intrinsic;

	class SystemSimulationTum
	{
	public:
		SystemSimulationTum(const std::string &settingFile);
		~SystemSimulationTum()
		{
//			delete scan_cur;
			delete map;
			delete feature_matching;
			delete it_gfm;
			delete plane_segmentation;
	//		if(plane_method==1) delete plane_extraction;
	//		else if(plane_method==2) delete plane_segmentation;
			delete line_extraction;
			delete motion_estimation;
	//		delete global_map;
		}

		Scan* getCurrentScan() const {return scan_cur;}
		Map* getMap() const {return map;}
		double time(int i) const {return timestamps[i];}
		int size() const {return timestamps.size();}

		double test(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		double simulate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		double optimize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double optimize_map(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		double associate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double localize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void visScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:

		bool debug;
		double debug_time;
		double start_time, end_time, delta_time;
		int vis_scans;
		std::ofstream fp;
		std::string save_folder;
		std::string load_folder;
		std::string seq_folder;
		double current_time;
		bool isFirstFrame;

		Scan *scan_cur, *scan_ref;
		Map *map;

		PlaneSegmentation *plane_segmentation;
		LineExtraction *line_extraction;
		GeometricFeatureMatching *feature_matching;
		MotionEstimation *motion_estimation;
		BaselessCliff::GeometricFeatureMatching *it_gfm;
//		GlobalMap *global_map;
//		RelativePoseError *rpe;

		std::vector<double> timestamps;
		std::map<double,std::string> files_depth;
		std::map<double,std::string> files_rgb;
		std::map<double,Transform> Tcw_truth;

		void loadImages(const std::string &folder);

		void saveMap() const 
		{
			if(!debug)
			{
				map->save(save_folder);
				map->saveTimes(save_folder); // filenames in /scans folder;
			}
		}

		void constraints(Scan *scan)
		{
			const int N=scan->sizeFeature();
			Eigen::MatrixXd A=Eigen::MatrixXd::Zero(N*3,3);
			int i=0;
			for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++,i++)
			{
				Feature *f=it->second;
				if(f->Type()==PLANE)
				{
					A.block<1,3>(i*3,0)=f->getDirection().transpose();
				}
				else if(f->Type()==LINE)
				{
					A.block<3,3>(i*3,0)=ulysses::Transform::skew_sym(f->getDirection());
				}
			}
			Eigen::Matrix3d ATA=A.transpose()*A;

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ATA);
			Eigen::Vector3d eigenvalues=es.eigenvalues();
//			Eigen::Matrix3d eigenvectors=es.eigenvectors();
//			fp<<"eigenvalues="<<eigenvalues.transpose()<<std::endl;
//			fp<<"eigenvectors="<<std::endl<<eigenvectors<<std::endl<<std::endl;
			
			scan->Constraints()=eigenvalues;
		}

		void saveConstraints()
		{
			fp.open(save_folder+"/constraints.txt");
			const_iterScan it_scan=map->beginScan();
			const_iterCamera it_cam=map->beginCamera();
			Transform Tgw=it_cam->second.inv()*it_scan->second->Tcw();
			for(const_iterScan it=map->beginScan();it!=map->endScan()&&it_cam!=map->endCamera();it++,it_cam++)
			{
				constraints(it->second);
				Transform Tres=it_cam->second.inv()*it->second->Tcw()*Tgw.inv();
				Vector6d xi=Tres.getMotionVector();
				double ratio=it->second->Constraints()(2)/it->second->Constraints()(0);
				fp<<std::fixed<<it->second->time()<<" "
				  <<xi.block<3,1>(0,0).norm()<<" "<<xi.block<3,1>(3,0).norm()<<" "
				  <<it->second->Constraints().transpose()<<" "<<ratio<<std::endl;
			}
			fp.close();
		}
	};
}

#endif
