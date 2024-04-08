/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-12-11 11:03
#
# Filename:		system_simulation_tum.cpp
#
# Description: 
#
************************************************/
#include "systems/system_simulation_tum.h"

namespace ulysses 
{
//	CameraIntrinsic camera_intrinsic;

	SystemSimulationTum::SystemSimulationTum(const std::string &settingFile)
	{
		remove("SystemSimulationTum.txt");

		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["debug"]>>debug;
		settings["debug_time"]>>debug_time;
		settings["start_time"]>>start_time;
		settings["end_time"]>>end_time;
		settings["delta_time"]>>delta_time;
		settings["vis_scans"]>>vis_scans;
		settings["loadFolder"]>>load_folder;
		settings["seqFolder"]>>seq_folder;
		settings["THRES_RAD"]>>THRES_RAD; THRES_RAD*=(M_PI/180.0);
		settings["THRES_DIST"]>>THRES_DIST;
		settings["Camera.fx"]>>camera_intrinsic.fx;
		settings["Camera.fy"]>>camera_intrinsic.fy;
		settings["Camera.cx"]>>camera_intrinsic.cx;
		settings["Camera.cy"]>>camera_intrinsic.cy;
		settings["Camera.width"]>>camera_intrinsic.width;
		settings["Camera.height"]>>camera_intrinsic.height;
		settings["Camera.factor"]>>camera_intrinsic.factor;
		settings.release();

		loadImages(seq_folder);

//		if(plane_method==1) plane_extraction=new PlaneExtraction(settingFile);
//		else if(plane_method==2) 
		plane_segmentation=new PlaneSegmentation(settingFile);
//		
		line_extraction=new LineExtraction(settingFile);
		feature_matching=new GeometricFeatureMatching(settingFile);
		motion_estimation=new MotionEstimation(settingFile);
		it_gfm=new BaselessCliff::GeometricFeatureMatching(settingFile);

//		global_map=new GlobalMap(settingFile);
//		global_map->setDebug(debug);
		scan_cur=0; scan_ref=0;
		map=new Map;
//		map_true=new Map;
//		map_true->load(load_folder);

		timeval time;
		gettimeofday(&time,NULL);
		current_time=time.tv_sec+time.tv_usec*1e-6;

		if(!debug)
		{
			save_folder="slam_"+std::to_string(current_time);
			std::string mkdir_scans="mkdir -p "+save_folder+"/scans";
			if(system(mkdir_scans.c_str()));
		}
	}

	using namespace std;
	double SystemSimulationTum::test(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}

		double last_time=0;
		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(timestamps[i]-start_time<-1e-4) continue;
				if(timestamps[i]>end_time) continue;
			}
			if(timestamps[i]-last_time<delta_time) continue;
			last_time=timestamps[i];

			scan_cur=new Scan(timestamps[i]);
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			cout<<endl;
			cout<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;

			plane_segmentation->extractPlanes(scan_cur);
			line_extraction->extract(scan_cur,vis); //vis->spin();

			cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<endl;
			if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}

			scan_cur->visScan(vis);

			if(scan_ref!=0)
			{
				scan_cur->ref()=scan_ref;
				it_gfm->match(scan_cur,vis);

//				if(debug)
				{
					vis->removeAllPointClouds();
					vis->removeAllCoordinateSystems();
					scan_cur->vis(vis); scan_ref->vis(vis);
					cout<<"before"<<endl; vis->spin();
					vis->removeAllPointClouds();
					vis->removeAllCoordinateSystems();
					scan_cur->vis(vis,1); scan_ref->vis(vis,1);
					cout<<"after"<<endl; vis->spin();
				}

				scan_ref->release();

			}
			scan_ref=scan_cur;

//			scan_cur->release();
//			delete scan_cur;
//			delete scan_cur;

//			if(scan_ref!=0)
//			{
//				double time_lbd=line_extraction->match(scan_cur,scan_ref);
//				cout<<"LBD time = "<<time_lbd<<endl;
//				double precision,recall;
//				scan_cur->association()->evalulatePR(scan_ref->time(),scan_cur->association_map(),precision,recall);
//
//				std::ofstream fp_pr; fp_pr.open(save_folder+"/LBD.txt",std::ios::app);
//				fp_pr<<std::fixed<<scan_cur->time()<<" "<<time_lbd<<" "<<precision<<" "<<recall<<endl;
//				fp_pr.close();
//
//				if(debug)
//				{
//					fp<<"precision="<<precision<<endl;
//					fp<<"recall="<<recall<<endl;
//				}
//	//			if(!debug) 
//				delete scan_ref;
//			}
//			scan_ref=scan_cur;
		}
//		saveMap();
//		cout<<"map saved ! "<<endl;
//
//			if(debug) map->vis(vis);

		if(debug) fp.close();
		return 0;
	}

	double SystemSimulationTum::optimize_map(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}
		map->load(load_folder);
		for(const_iterScan it=map->beginScan();it!=map->endScan();it++)
		{
			if(stop) break;
			if(debug) if(it->first-debug_time<-1e-4) continue;
			if(!debug) if(it->first-start_time<-1e-4) continue;
			if(!debug) if(it->first>end_time) continue;
			scan_cur=it->second;
			if(scan_cur==0) {std::cout<<"error"<<std::endl; return -1;}
			cout<<std::endl;
			cout<<"loading scan "<<std::fixed<<scan_cur->time()<<std::endl;
			scan_cur->loadScan(it->first,files_depth.find(it->first)->second,files_rgb.find(it->first)->second);
			scan_cur->Tcw()=Tcw_truth.find(it->first)->second;
			scan_cur->Tcg()=scan_cur->Tcw();
//			cout<<"Tcw "<<scan_cur->Tcw()<<endl;
			scan_cur->loadFeatures(load_folder);

			for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
			{
				Feature *f=it->second;
				if(f->Type()==PLANE)
				{
					plane_segmentation->computePlaneInfo(f->plane());
				}
				else if(f->Type()==LINE)
				{
					line_extraction->computeLineInfo(f->line());
				}
			}

			if(debug) fp<<endl<<"******************************************************"<<endl;
			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;

			if(debug) scan_cur->visScan(vis);

			if(scan_ref!=0)
			{
				Map* m=new Map;
//				m->addScan(scan_ref,true,false);
				scan_cur->association()=new FeatureAssociation(map,scan_cur,scan_ref);
//				cout<<"associations:"<<endl;
//				scan_cur->association()->print(cout);
//				cout<<"end associations"<<endl;
				m->addAssociation(scan_cur,scan_ref);
//				m->addScan(scan_cur,false,false);
				cout<<"addScan"<<endl;
				motion_estimation->estimate(m);
				scan_ref->Tcg()=m->camera(scan_ref->time());
				scan_cur->Tcg()=m->camera(scan_cur->time());
				m->clearScan();
//				m->clearLandmark();
				delete m;

				if(debug)
				{
					vis->removeAllPointClouds();
					vis->removeAllCoordinateSystems();
					scan_cur->vis(vis); scan_ref->vis(vis);
					cout<<"before"<<endl; vis->spin();
					vis->removeAllPointClouds();
					vis->removeAllCoordinateSystems();
					scan_cur->vis(vis,1); scan_ref->vis(vis,1);
					cout<<"after"<<endl; vis->spin();
				}

				if(!debug) scan_ref->release();
			}

			scan_ref=scan_cur;
//			if(!debug) scan_cur->release();
		}
		if(debug) fp.close();
		motion_estimation->estimate(map);
		saveMap();
		saveConstraints();
		cout<<"map saved !"<<endl;

		if(debug) { map->vis(vis); vis->spin(); }
		return 0;
	}

	double SystemSimulationTum::optimize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}

		double last_time=0;
		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(timestamps[i]-start_time<-1e-4) continue;
				if(timestamps[i]>end_time) continue;
			}
			if(timestamps[i]-last_time<delta_time) continue;
			last_time=timestamps[i];

			scan_cur=new Scan(timestamps[i]);
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

			if(debug) fp<<endl<<"******************************************************"<<endl;
			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			cout<<endl;
			cout<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;

			plane_segmentation->extractPlanes(scan_cur);
			line_extraction->extract(scan_cur,vis); //vis->spin();

			cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<endl;
			if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}

			if(scan_ref!=0)
			{
				scan_cur->ref()=scan_ref;
				double time_it=feature_matching->match(scan_cur,vis);
				map->addScan(scan_cur,false,false);
				cout<<"feature matching: "<<time_it<<endl;

				if(debug)
				{
					fp<<"Tcg="<<scan_cur->Tcg()<<endl;
					fp<<"association: "<<std::endl; scan_cur->association()->print(fp);
					fp<<"landmarks in map: "<<std::endl; map->printLandmarks(fp);
				}

				motion_estimation->estimate(map);
//				cout<<"estimated"<<endl;
				scan_cur->Tcg()=map->camera(scan_cur->time());
//				cout<<scan_cur->Tcg()<<endl;
				scan_ref->Tcg()=map->camera(scan_ref->time());
//				cout<<scan_ref->Tcg()<<endl;
				Transform Tcr=scan_cur->Tcg()*scan_ref->Tcg().inv();
				Transform Tcr_truth=scan_cur->Tcw()*scan_ref->Tcw().inv();
				Transform Td=Tcr*Tcr_truth.inv();
				Vector6d xi=Td.getMotionVector();
				cout<<"translation error = "<<xi.block<3,1>(0,0).norm()<<endl;
				cout<<"rotation error = "<<xi.block<3,1>(3,0).norm()<<endl;
				
//				if(debug)
//				{
//					scan_cur->vis(vis); scan_ref->vis(vis);
//					cout<<"before"<<endl; vis->spin();
//					scan_cur->vis(vis,1); scan_ref->vis(vis,1);
//					cout<<"after"<<endl; vis->spin();
//				}

//				if(!debug)
//				{
//					scan_cur->vis(vis,1); //scan_ref->vis(vis,1);
//					if((i+1)%vis_scans==0) 
//					{
//						vis->spin();
//						vis->removeAllPointClouds();
//						vis->removeAllCoordinateSystems();
//					}
//				}

				scan_cur->visScan(vis);

				scan_ref->release();
			}
			else 
			{
				map->addScan(scan_cur,true,false);
				if(debug) fp<<"landmarks in map: "<<std::endl; map->printLandmarks(fp);

				scan_cur->visScan(vis);
			}
			cout<<"added "<<map->sizeLandmark()<<" landmarks in the map"<<endl;

			scan_ref=scan_cur;
		}
		saveMap();
		saveConstraints();
		cout<<"map saved ! "<<endl;

//			if(debug) map->vis(vis);

		if(debug) fp.close();
		return 0;
	}

	double SystemSimulationTum::localize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}
		map->load(load_folder);
		for(const_iterScan it=map->beginScan();it!=map->endScan();it++)
		{
			if(debug) if(it->first-debug_time<-1e-4) continue;
			if(!debug) if(it->first-start_time<-1e-4) continue;
			scan_cur=it->second;
			if(scan_cur==0) {std::cout<<"error"<<std::endl; return -1;}
			cout<<std::endl;
			cout<<"loading scan "<<std::fixed<<scan_cur->time()<<std::endl;
			scan_cur->loadScan(it->first,files_depth.find(it->first)->second,files_rgb.find(it->first)->second);
			scan_cur->Tcw()=Tcw_truth.find(it->first)->second;
//			cout<<"Tcw "<<scan_cur->Tcw()<<endl;
			scan_cur->loadFeatures(load_folder);

//			scan_cur->visScan(vis); vis->spin();


			double time_it=feature_matching->match(scan_cur,map);
			cout<<"IT time = "<<time_it<<endl;
			Transform Tcr_delta=scan_cur->Tcw()*scan_cur->Tcg().inv();
			Vector6d xi=Tcr_delta.getMotionVector();
			double error_translation=xi.block<3,1>(0,0).norm();
			double error_rotation=xi.block<3,1>(3,0).norm();
			double precision,recall;
			scan_cur->association_map()->evalulatePR(scan_cur->time(),map,precision,recall);

			std::ofstream fp_pr; fp_pr.open(load_folder+"/IT_MAP_"+std::to_string(current_time)+".txt",std::ios::app);
			fp_pr<<std::fixed<<scan_cur->time()<<" "<<time_it<<" "
				 <<precision<<" "<<recall<<" " <<error_translation<<" "<<error_rotation<<endl;
			fp_pr.close();

			cout<<"Tcw="<<scan_cur->Tcw()<<endl;
			cout<<"Tcg="<<scan_cur->Tcg()<<endl;
			std::cout<<"error_trns="<<error_translation<<endl;
			std::cout<<"error_rot ="<<error_rotation<<endl;
			cout<<"precision="<<precision<<endl;
			cout<<"recall="<<recall<<endl;

			if(debug)
			{
				scan_cur->association_map()->vis(scan_cur,vis);
			}
			delete scan_cur;

		}
		if(debug)
		{
			fp<<"landmarks in map: "<<std::endl; map->printLandmarks(fp);
			fp.close();
		}
		return 0;
	}

	double SystemSimulationTum::associate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}
		map->load(load_folder);
		for(const_iterScan it=map->beginScan();it!=map->endScan();it++)
		{
			if(stop) break;
			if(debug) if(it->first-debug_time<-1e-4) continue;
			if(!debug) if(it->first-start_time<-1e-4) continue;
			scan_cur=it->second;
			if(scan_cur==0) {std::cout<<"error"<<std::endl; return -1;}
			cout<<std::endl;
			cout<<"loading scan "<<std::fixed<<scan_cur->time()<<std::endl;
			scan_cur->loadScan(it->first,files_depth.find(it->first)->second,files_rgb.find(it->first)->second);
			scan_cur->Tcw()=Tcw_truth.find(it->first)->second;
//			cout<<"Tcw "<<scan_cur->Tcw()<<endl;
			scan_cur->loadFeatures(load_folder);

			for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();)
			{
//				cout<<"for "<<it->first<<endl;
				if(it->second->Type()==LINE)
				{
//					cout<<"delete "<<it->second<<endl;
					delete it->second;
					it=scan_cur->eraseFeature(it); 
//					cout<<"erase "<<it->first<<endl; 
				}
				else it++;
			}
			scan_cur->printFeatures(cout);
			if(scan_cur->sizeFeature()==0) continue;
//			scan_cur->visScan(vis); vis->spin();

			if(scan_ref!=0)
			{
				scan_cur->ref()=scan_ref;
				double time_it=feature_matching->match(scan_cur,vis);
				cout<<"IT time = "<<time_it<<endl;
				Transform Tcr_delta=scan_cur->Tcw()*scan_ref->Tcw().inv()*scan_cur->Tcr().inv();
				Vector6d xi=Tcr_delta.getMotionVector();
				double error_translation=xi.block<3,1>(0,0).norm();
				double error_rotation=xi.block<3,1>(3,0).norm();
				double precision,recall;
				scan_cur->association()->evalulatePR(scan_cur->time(),scan_ref->time(),map,precision,recall);

				std::ofstream fp_pr; fp_pr.open(load_folder+"/ITPLANE_"+std::to_string(current_time)+".txt",std::ios::app);
				fp_pr<<std::fixed<<scan_cur->time()<<" "<<time_it<<" "
					 <<precision<<" "<<recall<<" "
					 <<error_translation<<" "<<error_rotation<<endl;
				fp_pr.close();

				cout<<"Tcr_truth="<<scan_cur->Tcw()*scan_ref->Tcw().inv()<<endl;
				cout<<"Tcr_estim="<<scan_cur->Tcr()<<endl;
				std::cout<<"error_trns="<<error_translation<<endl;
				std::cout<<"error_rot ="<<error_rotation<<endl;
				cout<<"precision="<<precision<<endl;
				cout<<"recall="<<recall<<endl;

				if(debug)
				{
					scan_cur->association()->vis(scan_cur,vis);
				}

//				vis->removeAllPointClouds();
//				vis->removeAllCoordinateSystems();
//				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//				pcl::transformPointCloud(*scan_cur->ref()->points(),*cloud,scan_cur->Tcr().getMatrix4f());
//				vis->addPointCloud (cloud,std::to_string(scan_cur->ref()->time()));
//				vis->addPointCloud (scan_cur->points(),std::to_string(scan_cur->time()));
//				vis->spin();

//				if(!debug) 
				delete scan_ref;
			}
			scan_ref=scan_cur;
		}
		if(debug)
		{
			fp<<"landmarks in map: "<<std::endl; map->printLandmarks(fp);
	//		map->vis(vis);
			fp.close();
		}
		return 0;
	}

	double SystemSimulationTum::simulate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("SystemSimulationTum.txt",std::ios::app);
		}

		double last_time=0;
		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(timestamps[i]-start_time<-1e-4) continue;
				if(timestamps[i]>end_time) continue;
			}
			if(timestamps[i]-last_time<delta_time) continue;
			last_time=timestamps[i];

			scan_cur=new Scan(timestamps[i]);
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			cout<<endl;
			cout<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			cout<<scan_cur->Tcw().inv()<<endl;

			plane_segmentation->extractPlanes(scan_cur);
//			line_extraction->extract(scan_cur,vis); //vis->spin();
			line_extraction->extract_lbd(scan_cur,vis);

			cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<endl;
			if(scan_cur->sizeFeature()==0) continue;
			if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}
			scan_cur->saveFeatures(save_folder);

			feature_matching->match(scan_cur,map,vis);
			cout<<"added "<<map->sizeLandmark()<<" landmarks in the map"<<endl;
			if(debug)
			{
				fp<<"landmarks in map: "<<std::endl; map->printLandmarks(fp);
				fp<<"association with map: "<<std::endl; scan_cur->association_map()->print(fp);
			}

			if(debug) 
			{
				scan_cur->visScan(vis); //vis->spin();
				map->vis(vis);
//				scan_cur->vis(vis,2); vis->spin();
			}
			if(!debug) scan_cur->release();

			//////////////////////////////////////////////
			//////////// LBD matching ////////////////////
			//////////////////////////////////////////////
			if(scan_ref!=0)
			{
				double time_lbd=line_extraction->match_lbd(scan_cur,scan_ref);
				cout<<"LBD time = "<<time_lbd<<endl;
				double precision,recall;
				scan_cur->association()->evalulatePR(scan_ref->time(),scan_cur->association_map(),precision,recall);

				std::ofstream fp_pr; fp_pr.open(save_folder+"/LBD.txt",std::ios::app);
				fp_pr<<std::fixed<<scan_cur->time()<<" "<<time_lbd<<" "<<precision<<" "<<recall<<endl;
				fp_pr.close();

				if(debug)
				{
					fp<<"precision="<<precision<<endl;
					fp<<"recall="<<recall<<endl;
				}
				delete scan_ref;
			}
			scan_ref=scan_cur;
			//////////////////////////////////////////////
		}
		saveMap();
		cout<<"map saved ! "<<endl;

		if(debug) fp.close();
		return 0;
	}

	void SystemSimulationTum::loadImages(const std::string &folder)
	{
		std::string file_association=folder+"/association.txt";

		std::ifstream fAssociation;
		fAssociation.open(file_association);
		while(!fAssociation.eof())
		{
			std::string s; getline(fAssociation,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				double t,time; std::string sRGB, sD;
				ss>>time>>sD;
				timestamps.push_back(time);
				files_depth.insert(std::pair<double,std::string>(time,folder+"/"+sD));
				ss>>t>>sRGB;
				files_rgb.insert(std::pair<double,std::string>(time,folder+"/"+sRGB));

				double tx,ty,tz,qx,qy,qz,qw;
				ss>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				ulysses::Transform Twc(tx,ty,tz,qx,qy,qz,qw);
				Tcw_truth.insert(std::pair<double,Transform>(time,Twc.inv()));
			}
		}
	}

	void SystemSimulationTum::visScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		Transform Tgc=scan->Tcw().inv();
		std::cout<<Tgc<<std::endl;

		// add point cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::transformPointCloud(*scan->points(),*cloud,Tgc.getMatrix4f());
		for(size_t i=0;i<cloud->height;i++)
		{
			for(size_t j=0;j<cloud->width;j++)
			{
				if(i%2==0&&j%2==0)
				{
					cloud_filtered->push_back(cloud->at(j,i));
				}
			}
		}

//		sprintf(id,"scan%f", scan->time());
		if (!vis->updatePointCloud (cloud_filtered,std::to_string(scan->time())))
			vis->addPointCloud (cloud_filtered,std::to_string(scan->time()));

		pcl::PointXYZRGBA pt1,pt2;

//		// origin of the reference frame;
//		pt1.x=0; pt1.y=0; pt1.z=0;
//		pt2.x=0.3; pt2.y=0; pt2.z=0; 
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,"x");
//		pt2.x=0; pt2.y=0.3; pt2.z=0; 
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,"y");
//		pt2.x=0; pt2.y=0; pt2.z=0.3; 
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,"z");

		// add camera poses;
		double scale=0.1;
		Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
		Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
		Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		// x - red
		pt2.x=pt1.x+x(0)*scale;
		pt2.y=pt1.y+x(1)*scale;
		pt2.z=pt1.z+x(2)*scale;
		sprintf(id,"%fx",scan->time());
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		// y - green
		pt2.x=pt1.x+y(0)*scale;
		pt2.y=pt1.y+y(1)*scale;
		pt2.z=pt1.z+y(2)*scale;
		sprintf(id,"%fy",scan->time());
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		// z - green
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		sprintf(id,"%fz",scan->time());
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);

//		// ===================================================
//		// add groundtruth;
//		Transform Twc=scan->Tcw().inv();
//		x=Twc.R.block<3,1>(0,0);
//		y=Twc.R.block<3,1>(0,1);
//		z=Twc.R.block<3,1>(0,2);
//		pt1.x=Twc.t(0);
//		pt1.y=Twc.t(1);
//		pt1.z=Twc.t(2);
//		// x - magenta
//		pt2.x=pt1.x+x(0)*scale;
//		pt2.y=pt1.y+x(1)*scale;
//		pt2.z=pt1.z+x(2)*scale;
//		sprintf(id,"%fxgt",scan->time());
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,255,id);
//		// y - yellow
//		pt2.x=pt1.x+y(0)*scale;
//		pt2.y=pt1.y+y(1)*scale;
//		pt2.z=pt1.z+y(2)*scale;
//		sprintf(id,"%fygt",scan->time());
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,0,id);
//		// z - cyan
//		pt2.x=pt1.x+z(0)*scale;
//		pt2.y=pt1.y+z(1)*scale;
//		pt2.z=pt1.z+z(2)*scale;
//		sprintf(id,"%fzgt",scan->time());
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,255,id);
//
//		// trajectory
//		pt2.x=Twc.t(0);
//		pt2.y=Twc.t(1);
//		pt2.z=Twc.t(2);
//		sprintf(id,"%ftraj",scan->time());
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);

	}
}
