/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-05 09:32
#
# Filename:		system_simulation.cpp
#
# Description: 
#
************************************************/
#include "systems/system_simulation.h"

namespace ulysses 
{

	SystemSimulation::SystemSimulation(const std::string &setting_file)
	{
		remove("SystemSimulation.txt");
		cv::Mat T0, v, w;

		std::string settingFile=setting_file;
		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["loadSavedData"]>>loadSavedData;
		settings["loadNoisyData"]>>loadNoisyData;
		settings["FileWrite"]>>filename_write;
		settings["FileRead"]>>filename_read;
		settings["sigma"]>>sigma;
		settings["overlap"]>>overlap;
		settings["NumPlane"]>>NumPlane;
		settings["NumLine"]>>NumLine;
		settings["linear_velocity"]>>v;
		settings["angular_velocity"]>>w;
		settings["time_delta"]>>time_delta;
		settings["Tw0"]>>T0;
		// thresholds for feature matching;
		// defined in GeometricFeatureMatching.cpp;
		settings["THRES_RAD"]>>THRES_RAD; THRES_RAD*=(M_PI/180.0);
		settings["THRES_DIST"]>>THRES_DIST;
		settings.release();

		cv::cv2eigen(v,linear_velocity);
		cv::cv2eigen(w,angular_velocity);
		Tw0=ulysses::Transform(T0);
//		std::cout<<"Tw0="<<Tw0<<std::endl;

		scan_cur=0; 
		scan_ref=0;

		map=new Map;
//		map_sim=new Map;
		feature_matching=new GeometricFeatureMatching(settingFile);
//		global_map=new GlobalMap(settingFile);
//		global_map->setDebug(debug);

		timeval time;
		gettimeofday(&time,NULL);
		current_time=time.tv_sec+time.tv_usec*1e-6;

		folder="simulate_"+std::to_string(current_time);
		std::string mkdir_scans="mkdir -p "+folder+"/scans";
		if(system(mkdir_scans.c_str()));
	}

	SystemSimulation::~SystemSimulation()
	{
		delete feature_matching;
//		delete global_map;
		delete map;
		delete scan_cur;
	}

	// scan->time_stamp,id
	bool SystemSimulation::simulate()
	{
		fp.open("SystemSimulation.txt",std::ios::app);
		fp<<std::endl<<"****************************SystemSimulation::simulate()*******************************"<<std::endl;
		timeval time_start, time_end;
		double time_used;

		scan_cur=new Scan(current_time);
		current_time+=time_delta;
//		scan_cur->Tcg.setIdentity();
//		num_scan++;
		bool isFirstFrame=false;

		fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
		std::cout<<std::fixed<<scan_cur->time()<<"-----------------------"<<std::endl;

		// simulation;
		if(scan_ref==0) 
		{
			isFirstFrame=true;
			generateScan(scan_cur);
		}
		else 
		{
			scan_cur->ref()=scan_ref;
			generateScan(scan_cur);

			fp<<"simulated feature associations"<<std::endl;
			scan_cur->association()->print(fp);
			scan_cur->saveAssociations(folder);
			std::cout<<"saved simulated association "<<std::endl;

			delete scan_ref;
		}

		fp<<"simulated features"<<std::endl;
		scan_cur->printFeatures(fp);
		scan_cur->saveFeatures(folder);
		std::cout<<"saved simulated features "<<std::endl;

//		// estimation;
//		map->addCamera(scan_cur->Tcg(),scan_cur->time());
//		std::cout<<"Tcg = "<<scan_cur->Tcg()<<std::endl;
//		std::cout<<"Tcw = "<<scan_cur->Tcw()<<std::endl;
//
//		if(isFirstFrame)
//		{
//			for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
//			{
//				map->addLandmark(new Landmark(it->second,scan_cur->Tcg()));
//			}
//		}
//		else // if(!isFirstFrame)
//		{
//			feature_matching->match(scan_cur);
//			scan_cur->localize();
//
//			fp<<"estimated feature associations"<<std::endl;
//			scan_cur->association()->printAssociatedFeatures(fp);
//
//			map->addAssociation(scan_cur->association());
//
////			std::cout<<"simulated feature associations"<<std::endl;
////			scan_cur->association_sim()->printAssociatedFeatures(std::cout);
//
//			double precision, recall;
//			scan_cur->association()->evalulatePR(scan_cur->association_sim(),precision,recall);
//			fp<<std::endl;
//			fp<<"precision="<<precision<<std::endl;
//			fp<<"recall="<<recall<<std::endl;
//
//			scan_cur->saveAssociations(folder+"/ulysses");
//			std::cout<<"saved estimated association "<<std::endl;
//
////			name=folder+"/scans/"+std::to_string(scan_cur->time())+"_association.txt";
////			std::cout<<name<<std::endl;
//
//			delete scan_ref;
//		}

		scan_ref=scan_cur;

		fp<<"***************************************************************************************"<<std::endl<<std::endl;
//		std::cout<<"scan_cur="<<scan_cur<<", "<<scan_cur->association_sim()<<std::endl;
		fp.close();
		return true;
	}

	bool SystemSimulation::generateScan(Scan *scan)
	{
		fp<<std::endl<<"====================SystemSimulation::generateScan()===================="<<std::endl;
//
//		fp<<"NumPlane="<<NumPlane<<std::endl;
//		fp<<"NumLine="<<NumLine<<std::endl;
//		fp<<"overlap="<<overlap<<std::endl;
		scan->association()=new FeatureAssociation;

		int num_overlap=overlap*double(NumPlane+NumLine);

		if(scan_ref==0) // first scan, scan_ref=0;
		{
			scan->Tcw()=Tw0.inv(); // initial pose of the camera;
//			scan->Tcr().setIdentity();
			fp<<"Twc="<<scan->Tcw().inv()<<std::endl;
			map->addCamera(scan->Tcw(),scan->time());

//			fp<<std::endl<<"no overlap (first frame)"<<std::endl;
//			fp<<"generate "<<NumPlane<<" plane and "<<NumLine<<" line landmarks in map"<<std::endl;
//			fp<<"and simulate observations in scan"<<std::endl;
			for(int i=0;i<NumPlane+NumLine;i++)
			{
				Landmark *lm;
				if(i<NumPlane) lm=new Landmark(new PlaneLM(Eigen::Vector4d::Random()));
				else lm=new Landmark(new LineLM(Vector6d::Random()));
				map->addLandmark(lm);
				Feature *ft=new Feature(lm,scan->Tcw(),sigma);
				scan->addFeature(ft);
//				fp<<i<<std::endl;
//				fp<<lm<<std::endl;
//				fp<<ft<<std::endl;
			}
		}
		else 
		{
			Vector6d xi=transformVelocity(linear_velocity,angular_velocity,scan->ref()->Tcw().inv());
			// Twc=exp(xi,theta)*Twr --- ???
			Transform Twc=expmap(xi,time_delta)*scan->ref()->Tcw().inv();
			scan->Tcw()=Twc.inv();
			fp<<"Twc="<<Twc<<std::endl;
			map->addCamera(scan->Tcw(),scan->time());

//			fp<<overlap<<" overlap with reference scan"<<std::endl;

			std::vector<std::string> indices;
			for(iterFeature it=scan->ref()->beginFeature();it!=scan->ref()->endFeature();it++)
			{ indices.push_back(it->first); }
			std::random_device rd;
			std::mt19937 g(rd());
			std::shuffle(indices.begin(),indices.end(),g);

//			fp<<std::endl<<"sample "<<num_overlap<<" observations from reference scan"<<std::endl;
			int i=0;
			// first num_plane_overlap planes in scan->observed_planes;
			for(std::vector<std::string>::iterator it=indices.begin();it!=indices.end();it++,i++)
			{
				Feature *feat=scan->ref()->findFeature(*it);
				if(i<num_overlap)
				{
					Landmark *lm=feat->ptrLandmark();
					Feature *feat_cur=new Feature(lm,scan->Tcw(),sigma);
					scan->addFeature(feat_cur);
					scan->association()->insert(feat_cur,feat,lm);
//					fp<<i<<std::endl;
//					fp<<lm<<std::endl;
//					fp<<feat_cur<<std::endl;
				}
				else 
				{
//					if(feat->Type()==PLANE) scan->addFeature(new Feature(new Plane));
//					else if(feat->Type()==LINE) scan->addFeature(new Feature(new Line));
					Landmark *lm;
					if(feat->Type()==PLANE) lm=new Landmark(new PlaneLM(Eigen::Vector4d::Random()));
					else if(feat->Type()==LINE) lm=new Landmark(new LineLM(Vector6d::Random()));
					map->addLandmark(lm);
					Feature *ft=new Feature(lm,scan->Tcw(),sigma);
					scan->addFeature(ft);
//					fp<<i<<std::endl;
//					fp<<lm<<std::endl;
//					fp<<ft<<std::endl;
				}
			}
		}

		fp<<"========================================================================"<<std::endl<<std::endl;
		
//		fp<<std::endl<<"features_cur"<<std::endl; scan->printFeatures(fp);
//		fp<<std::endl<<"associated features"<<std::endl; scan->association()->printAssociatedFeatures(fp);
////		fp<<std::endl<<"associated landmarks"<<std::endl; scan->association()->printAssociatedLandmarks(fp);
//		fp<<std::endl<<"map traj"<<std::endl; map->printCameras(fp);
//		fp<<std::endl<<"map landmarks"<<std::endl; map->printLandmarks(fp);
////		fp<<"features_ref"<<std::endl; scan->ref()->printFeatures(fp);
		return true;
	}

//	void SystemSimulation::addNoise2Plane(PlaneLM *lm, Plane *p)
//	{
//		Eigen::Vector4d pi=lm->pi;
////		double d=sqrt(1.0+p->d*p->d);
//
//		Eigen::Matrix4d H=pi*pi.transpose();
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(H);
//		// the eigenvalues are sorted in increasing order;
//		Eigen::Vector4d e1=es.eigenvectors().block<4,1>(0,0);
//		Eigen::Vector4d e2=es.eigenvectors().block<4,1>(0,1);
//		Eigen::Vector4d e3=es.eigenvectors().block<4,1>(0,2);
//
//		unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
//		std::default_random_engine generator(seed);
//		std::normal_distribution<double> distribution(0,sigma);
//		double x1=distribution(generator);
//		double x2=distribution(generator);
//		double x3=distribution(generator);
//		Eigen::Vector4d noise=x1*e1+x2*e2+x3*e3; // noise for pi;
////		Vector3d noise_n=noise.block<3,1>(0,0);
////		double noise_d=noise(3);
//		
//		pi+=noise;
//		p->n=pi.block<3,1>(0,0);
//		p->d=pi(3)/p->n.norm();
//		p->n.normalize();
//	}
//
//	void SystemSimulation::addNoise2Line(LineLM *lm, Line *l)
//	{
//		Vector6d L=lm->L.normalized();
//		Vector6d L_dual;
//		L_dual.block<3,1>(0,0)=L.block<3,1>(3,0);
//		L_dual.block<3,1>(3,0)=L.block<3,1>(0,0);
//
//		Matrix6d H=L*L.transpose()+L_dual*L_dual.transpose();
//		Eigen::SelfAdjointEigenSolver<Matrix6d> es(H);
//		// the eigenvalues are sorted in increasing order;
//		Vector6d e1=es.eigenvectors().block<6,1>(0,0);
//		Vector6d e2=es.eigenvectors().block<6,1>(0,1);
//		Vector6d e3=es.eigenvectors().block<6,1>(0,2);
//		Vector6d e4=es.eigenvectors().block<6,1>(0,3);
//
//		unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
//		std::default_random_engine generator(seed);
//		std::normal_distribution<double> distribution(0,sigma);
//		double x1=distribution(generator);
//		double x2=distribution(generator);
//		double x3=distribution(generator);
//		double x4=distribution(generator);
//		Vector6d noise=x1*e1+x2*e2+x3*e3+x4*e4; // noise for L;
//
////		Vector3d noise_v=noise.block<3,1>(3,0);
////		Vector3d noise_u=noise.block<3,1>(0,0);
////		double sd=sqrt(1.0+l->u.norm()*l->u.norm());
////		fp<<"|delta_v|="<<noise_v.norm()*sd*180.0/M_PI<<"\t|noise_u|="<<noise_u.norm()*sd<<endl;
//		
//		L+=noise;
////		fp<<"L+noise="<<L.transpose()<<endl;
//		L.normalize();
//
//		l->v=L.block<3,1>(3,0);
//		l->u=L.block<3,1>(0,0)/l->v.norm();
//		l->v.normalize();
//	}

	void SystemSimulation::visTrajSim(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		pcl::PointXYZRGBA pt1,pt2;

		std::cout<<"visualize the simulated map"<<std::endl;
		std::cout<<"x axis - red"<<std::endl;
		std::cout<<"y axis - greem"<<std::endl;
		std::cout<<"z axis - blue"<<std::endl;

		// world frame;
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0.2; pt2.y=0; pt2.z=0;
		sprintf(id,"world_x");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0.2; pt2.z=0;
		sprintf(id,"world_y");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0; pt2.z=0.2;
		sprintf(id,"world_z");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
		
		for(int i=0;i<map->sizeCamera();i++)
		{
			Transform Tgc=map->camera(i).inv();

			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			pt2.x=pt1.x+Tgc.R(0,0)*0.1;
			pt2.y=pt1.y+Tgc.R(1,0)*0.1;
			pt2.z=pt1.z+Tgc.R(2,0)*0.1;
			sprintf(id,"%dx",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id); // red

			pt2.x=pt1.x+Tgc.R(0,1)*0.1;
			pt2.y=pt1.y+Tgc.R(1,1)*0.1;
			pt2.z=pt1.z+Tgc.R(2,1)*0.1;
			sprintf(id,"%dy",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id); // green 

			pt2.x=pt1.x+Tgc.R(0,2)*0.1;
			pt2.y=pt1.y+Tgc.R(1,2)*0.1;
			pt2.z=pt1.z+Tgc.R(2,2)*0.1;
			sprintf(id,"%dz",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id); // blue
		}

		vis->spin();
	}
}
