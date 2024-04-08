/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-12-12 09:56
#
# Filename:		map.cpp
#
# Description: 
#
************************************************/

#include "types/map.h"
#include <pcl/filters/voxel_grid.h>

namespace ulysses
{
	CameraIntrinsic camera_intrinsic;

	using namespace std;
	FeatureAssociation::FeatureAssociation(Map *map, Scan* scan_cur, Scan* scan_ref)
	{
		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
		{
			Landmark* lm=it->second;
			for(iterObserv it_obs=lm->beginObserv();it_obs!=lm->endObserv();it_obs++)
			{
				double time=it_obs->first;
				std::vector<std::string> ids=it_obs->second;
				if(fabs(time-scan_cur->time())<1e-4)
				{
//					cout<<endl<<lm->ID()<<endl;
//					cout<<scan_cur->time()<<endl;
					for(int i=0;i<ids.size();i++)
					{
						Feature *ft=scan_cur->findFeature(ids[i]);
						if(ft!=0) {ft->ptrLandmark()=lm;}
					}
				}
				if(fabs(time-scan_ref->time())<1e-4)
				{
//					cout<<endl<<lm->ID()<<endl;
//					cout<<scan_ref->time()<<endl;
					for(int i=0;i<ids.size();i++)
					{
						Feature *ft=scan_ref->findFeature(ids[i]);
						if(ft!=0) {ft->ptrLandmark()=lm;}
					}
				}
			}
		}
		for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
		{
			for(iterFeature it2=scan_ref->beginFeature();it2!=scan_ref->endFeature();it2++)
			{
				Feature *ft_cur=it->second;
				Feature *ft_ref=it2->second;
				if(ft_cur->ptrLandmark()==0 || ft_ref->ptrLandmark()==0) continue;
				if(ft_cur->ptrLandmark()==ft_ref->ptrLandmark())
				{
//					cout<<endl;
//					cout<<ft_cur<<endl;
//					cout<<ft_ref<<endl;
//					cout<<"insert: "<<ft_cur->ptrLandmark()->ID()<<endl;
					insert(ft_cur,ft_ref,ft_cur->ptrLandmark());
				}
			}
		}

	}

	void FeatureAssociation::insert(Feature *cur, Feature *ref, Landmark *lm)
	{
		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		features_cur.push_back(cur);
		features_ref.push_back(ref);
		landmarks.push_back(lm);
	}

	void FeatureAssociation::insert(Feature *cur, Feature *ref, Transform Trg)
	{
		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		features_cur.push_back(cur);
		features_ref.push_back(ref);
		// a new landmark is allocated;
		if(ref->ptrLandmark()==0) landmarks.push_back(new Landmark(ref,Trg));
		else landmarks.push_back(ref->ptrLandmark());
		cur->ptrLandmark()=ref->ptrLandmark();
	}

	void FeatureAssociation::insert(Feature *cur, Feature *ref)
	{
		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		features_cur.push_back(cur);
		features_ref.push_back(ref);
	}

	std::string FeatureAssociation::findFeature(std::string id) 
	{
//			std::ofstream fp; fp.open("find.txt",std::ios::app);
		std::map<std::string,int>::iterator it=indices.find(id);
//			fp<<std::endl<<id<<std::endl;
//			fp<<it->first<<" "<<it->second<<std::endl;
		if(it==indices.end()) return std::string("");
		else return features_ref[it->second]->ID();
	}

	std::string FeatureAssociation::findLandmark(std::string id) 
	{
//			std::ofstream fp; fp.open("find.txt",std::ios::app);
		std::map<std::string,int>::iterator it=indices.find(id);
//			fp<<std::endl<<id<<std::endl;
//			fp<<it->first<<" "<<it->second<<std::endl;
		if(it==indices.end()) return std::string("");
		else return landmarks[it->second]->ID();
	}

	void FeatureAssociation::evalulatePR(FeatureAssociation *truth, double &precision, double &recall)
	{
		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
		for(int i=0;i<size();i++)
		{
			if(truth->findFeature(features_cur[i]->ID())==features_ref[i]->ID())
			{
				true_pos+=1.0;
			}
		}
		precision=true_pos/pos;
		recall=true_pos/neg;
	}

	using namespace std;
	void FeatureAssociation::evalulatePR(double ref_time, LandmarkAssociation *truth, double &precision, double &recall)
	{
		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
		for(int i=0;i<size();i++)
		{
			std::string lm_id=truth->findLandmark(features_cur[i]->ID());
			if(lm_id.empty()) continue;
			else 
			{
				Landmark* lm=truth->getLandmark(lm_id);
				std::vector<std::string> obs=lm->Observ(ref_time);
				if(!obs.empty())
				{
					neg+=obs.size();
					for(int j=0;j<obs.size();j++)
					{
						if(obs[j]==features_ref[i]->ID())
						{
							true_pos+=1.0;
							break;
						}
					}
				}
				
//				for(int t=0;t<lm->sizeObserv();t++)
//				{
//					if(fabs(lm->observTime(t)-ref_time)<1e-6)
//					{
//						neg+=1.0;
//						if(lm->observID(t)==features_ref[i]->ID())
//						{
//							true_pos+=1.0;
//							break;
//						}
//					}
//				}
			}
		}
//		cout<<"true_pos="<<true_pos<<endl;
//		cout<<"pos="<<pos<<endl;
//		cout<<"neg="<<neg<<endl;
		precision=true_pos/pos;
		recall=true_pos/neg;
	}

//	using namespace std;
	void FeatureAssociation::evalulatePR(double cur_time, double ref_time, Map *map, double &precision, double &recall)
	{
		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
		for(int i=0;i<size();i++)
		{
			std::string id_cur=features_cur[i]->ID();
			std::string id_ref=features_ref[i]->ID();
//			cout<<"pair --- "<<id_cur<<" "<<id_ref<<endl;
			for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
			{
				Landmark* lm=it->second;
				std::vector<std::string> obs_cur=lm->Observ(cur_time);
//				if(!obs_cur.empty()) neg+=1.0;
				bool flag=false;
				for(int i_cur=0;i_cur<obs_cur.size();i_cur++)
				{
//					cout<<obs_cur[i_cur]<<" ";
					if(obs_cur[i_cur]==id_cur)
					{
//						flag=true;
						std::vector<std::string> obs_ref=lm->Observ(ref_time);
						if(!obs_ref.empty()) flag=true;
						for(int i_ref=0;i_ref<obs_ref.size();i_ref++)
						{
//							cout<<obs_ref[i_ref]<<" ";
							if(obs_ref[i_ref]==id_ref)
							{
								true_pos+=1.0;
							}
						}
					}
//					cout<<endl;
				}
				if(flag) neg+=1.0;
			}
		}
//		cout<<"true_pos="<<true_pos<<endl;
//		cout<<"pos="<<pos<<endl;
//		cout<<"neg="<<neg<<endl;
		precision=true_pos/pos;
		recall=true_pos/neg;
	}

	void FeatureAssociation::print(std::ostream &os) const 
	{
		for(int i=0;i<features_cur.size();i++)
		{
			os<<features_cur[i]->ID()<<" \t"<<features_ref[i]->ID()<<" \t"<<landmarks[i]->ID()<<std::endl;
		}
	}


	void FeatureAssociation::load(const std::string &folder, Scan *scan, Map *map)
	{
		std::string filename=folder+"/scans/"+std::to_string(scan->time())+"_association.txt";
//		std::cout<<filename<<std::endl;
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				std::string cur,ref,lm;
				ss>>cur>>ref>>lm;
//				std::cout<<"load "<<cur<<", "<<ref<<", "<<lm<<std::endl;
//				std::cout<<scan->findFeature(cur)<<std::endl;
//				std::cout<<scan->ref()->findFeature(ref)<<std::endl;
//				std::cout<<map->findLandmark(lm)<<std::endl;
				insert(scan->findFeature(cur),scan->ref()->findFeature(ref),map->findLandmark(lm));
			}
		}
		fp.close();
	}

	void FeatureAssociation::vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		s->vis(v);
		s->ref()->vis(v);
		for(int i=0;i<features_cur.size();i++)
		{
//			if (!v->updatePointCloud (s->points(), "scan"))
//				v->addPointCloud (s->points(), "scan");
			features_cur[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"cur");
			features_ref[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"ref");
		}
		v->spin();
	}

	void LandmarkAssociation::insert(Feature *cur, Landmark *lm)
	{
		indices_feature.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		// fill the indices_map in Map::addScan();
		// indices_map.insert(std::pair<std::string,int>(lm->ID(),features_cur.size()));
		features_cur.push_back(cur);
		landmarks.push_back(lm);
		if(cur->getDirection().dot(lm->getDirection())<0)
		{
			if(cur->Type()==PLANE)
			{
				Plane* p=cur->plane();
				p->n=-p->n;
				p->d=-p->d;
			}
			else if(cur->Type()==LINE)
			{
				Line* l=cur->line();
				l->v=-l->v;
				l->u=-l->u;
			}
		}
	}

	void LandmarkAssociation::fillIndicesMap()
	{
		for(int i=0;i<landmarks.size();i++)
		{
			indices_map.insert(std::pair<std::string,int>(landmarks[i]->ID(),i));
		}
	}

	std::string LandmarkAssociation::findLandmark(std::string id_feature) 
	{
		std::map<std::string,int>::iterator it=indices_feature.find(id_feature);
		if(it==indices_feature.end()) return std::string("");
		else return landmarks[it->second]->ID();
	}

	std::string LandmarkAssociation::findFeature(std::string id_landmark) 
	{
//		std::cout<<"id_landmark="<<id_landmark<<std::endl;
//		for(std::map<std::string,int>::iterator it=indices_map.begin();it!=indices_map.end();it++)
//		{
//			std::cout<<it->first<<" "<<it->second<<std::endl;
//		}
		std::map<std::string,int>::iterator it=indices_map.find(id_landmark);
		if(it==indices_map.end()) return std::string("");
		else return features_cur[it->second]->ID();
	}

	void LandmarkAssociation::evalulatePR(LandmarkAssociation *truth, double &precision, double &recall)
	{
		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
		for(int i=0;i<size();i++)
		{
			if(truth->findLandmark(features_cur[i]->ID())==landmarks[i]->ID())
			{
				true_pos+=1.0;
			}
		}
		precision=true_pos/pos;
		recall=true_pos/neg;
	}

	void LandmarkAssociation::evalulatePR(double time, Map *map, double &precision, double &recall)
	{
		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
		for(int i=0;i<size();i++)
		{
			std::string id_ft=features_cur[i]->ID();
			std::string id_lm=landmarks[i]->ID();
//			cout<<"pair --- "<<id_cur<<" "<<id_ref<<endl;
			for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
			{
				Landmark* lm=it->second;
				if(lm->ID()!=id_lm) continue;
				std::vector<std::string> obs_cur=lm->Observ(time);
				if(!obs_cur.empty()) neg+=1.0;
				for(int i_cur=0;i_cur<obs_cur.size();i_cur++)
				{
					if(obs_cur[i_cur]==id_ft)
					{
						true_pos+=1.0;
					}
				}
			}
		}
//		cout<<"true_pos="<<true_pos<<endl;
//		cout<<"pos="<<pos<<endl;
//		cout<<"neg="<<neg<<endl;
		precision=true_pos/pos;
		recall=true_pos/neg;
	}
//	void LandmarkAssociation::evalulatePR(double ref_time, LandmarkAssociation *truth, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			std::string lm_id=truth->findLandmark(features_cur[i]->ID());
//			if(lm_id.empty()) continue;
//			else 
//			{
//				Landmark* lm=truth->getLandmark(lm_id);
//				for(int t=0;t<lm->sizeObserv();t++)
//				{
//					if(fabs(lm->observTime(t)-ref_time)<1e-6)
//					{
//						true_pos+=1.0;
//						break;
//					}
//				}
//			}
//		}
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}

	void LandmarkAssociation::print(std::ostream &os) const 
	{
		for(int i=0;i<features_cur.size();i++)
		{
			os<<features_cur[i]->ID()<<" \t"<<landmarks[i]->ID()<<std::endl;
		}
	}

	void LandmarkAssociation::load(const std::string &folder, Scan *scan, Map *map)
	{
		std::string filename=folder+"/scans/"+std::to_string(scan->time())+"_association.txt";
//		std::cout<<filename<<std::endl;
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				std::string cur,ref,lm;
				ss>>cur>>ref>>lm;
//				std::cout<<"load "<<cur<<", "<<ref<<", "<<lm<<std::endl;
//				std::cout<<scan->findFeature(cur)<<std::endl;
//				std::cout<<scan->ref()->findFeature(ref)<<std::endl;
//				std::cout<<map->findLandmark(lm)<<std::endl;
				insert(scan->findFeature(cur),map->findLandmark(lm));
			}
		}
		fp.close();
	}

	void LandmarkAssociation::vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		s->vis(v);
//		s->ref()->vis(v);
		for(int i=0;i<features_cur.size();i++)
		{
//			if (!v->updatePointCloud (s->points(), "scan"))
//				v->addPointCloud (s->points(), "scan");
			features_cur[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"cur");
//			features_ref[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"ref");
		}
		v->spin();
	}

	Map::~Map()
	{
		for(iterLandmark it=landmarks.begin();it!=landmarks.end();it++) {delete it->second;}
		for(std::map<double,Scan*>::iterator it=scans.begin();it!=scans.end();it++) {delete it->second;}
	}

	Transform Map::camera(double time) const 
	{
		const_iterCamera it=cameras.find(time);
		if(it!=cameras.end()) return it->second;
		else return Transform();
	}

	void Map::updateCamera(double time, const Transform &T)
	{
		iterCamera it=cameras.find(time);
		if(it!=cameras.end()) { it->second=T; }
	}

	Scan* Map::scan(double time) const 
	{
		const_iterScan it=scans.find(time);
		if(it!=scans.end()) return it->second;
		else return 0;
	}


	void Map::addScan(Scan *scan, bool firstFr, bool truth)
	{
		scans.insert(std::pair<double,Scan*>(scan->time(),scan));
		if(truth)
		{
			if(firstFr)
			{
				addCamera(scan->Tcw(),scan->time());
				for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
				{ addLandmark(new Landmark(it->second,scan->Tcw()),scan->time(),it->second->ID()); }
			}
			else 
			{
				addCamera(scan->Tcw(),scan->time());
				addAssociation(scan->association_map(),scan->time());
			}
		}
		else
		{
			if(firstFr)
			{
				addCamera(scan->Tcg(),scan->time());
				for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
				{ addLandmark(new Landmark(it->second,scan->Tcg()),scan->time(),it->second->ID()); }
			}
			else 
			{
				addCamera(scan->Tcg(),scan->time());
				addAssociation(scan->association(),scan->time());
			}
		}
	}

	void Map::addCamera(Transform Tcg, double timestamp)
	{
		cameras.insert(std::pair<double,Transform>(timestamp,Tcg));
	}

	void Map::addLandmark(Landmark *lm, const double &time, const std::string &observ)
	{
		iterLandmark it=landmarks.find(lm->ID());
		lm->pushObserv(time,observ);
		if(it==landmarks.end()) 
		{ 
			// new landmark;
			lm->setID(landmarks.size());
			landmarks.insert(std::pair<std::string,Landmark*>(lm->ID(),lm));
		}
	}

	void Map::addAssociation(FeatureAssociation *fa, double time)
	{
		for(int i=0;i<fa->size();i++)
		{
			addLandmark(fa->getLandmark(i),time,fa->getFeature(i)->ID());
		}
	}

	void Map::addAssociation(LandmarkAssociation *la, double time)
	{
		for(int i=0;i<la->size();i++)
		{
			addLandmark(la->getLandmark(i),time,la->getFeature(i)->ID());
		}
		la->fillIndicesMap();
	}

	void Map::addAssociation(Scan *scan_cur,Scan *scan_ref)
	{
		scans.insert(std::pair<double,Scan*>(scan_cur->time(),scan_cur));
		scans.insert(std::pair<double,Scan*>(scan_ref->time(),scan_ref));
		addCamera(scan_cur->Tcg(),scan_cur->time());
		addCamera(scan_ref->Tcg(),scan_ref->time());
		FeatureAssociation *fa=scan_cur->association();
		for(int i=0;i<fa->size();i++)
		{
			Landmark* lm=fa->getLandmark(i);
			Feature* ft_cur=fa->getFeature(i);
			Feature* ft_ref=fa->getFeatureRef(i);
			Landmark* landmark=new Landmark(lm);
			landmark->pushObserv(scan_cur->time(),ft_cur->ID());
			landmark->pushObserv(scan_ref->time(),ft_ref->ID());
			landmarks.insert(std::pair<std::string,Landmark*>(landmark->ID(),landmark));
		}
	}

	Landmark* Map::findLandmark(std::string id) 
	{
		iterLandmark it=landmarks.find(id);
		if(it!=landmarks.end()) return it->second;
		else return 0;
	}

	void Map::printCameras(std::ostream &os) const 
	{
		os.precision(6); os<<std::fixed;
		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
		{
			os<<it->first<<" "<<it->second<<std::endl;
		}
	}

	void Map::printLandmarks(std::ostream &os) const 
	{
		for(const_iterLandmark it=landmarks.begin();
			it!=landmarks.end();it++)
		{
			os<<it->second<<std::endl;
		}
	}
	
	void Map::save(std::string folder) const 
	{
		std::string traj=folder+"/traj.txt";
		std::string map=folder+"/map.txt";
		saveTraj(traj);
		saveMap(map);
		std::string traj_slam=folder+"/traj_slam.txt";
		saveTrajSLAM(traj_slam);
		std::string traj_vo=folder+"/traj_vo.txt";
		saveTrajVO(traj_vo);
	}

	void Map::load(std::string folder)
	{
		std::string traj=folder+"/traj.txt";
		std::string map=folder+"/map.txt";
		loadTraj(traj);
		loadMap(map);
	}

	void Map::saveTimes(const std::string &folder) const 
	{
		std::string filename=folder+"/timestamp.txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		fp.precision(6); fp<<std::fixed;
		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
		{
			fp<<it->first<<std::endl;
		}
		fp.close();
	}

	using namespace std;
	void Map::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();

//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
//		filter.setLeafSize(0.05f,0.05f,0.05f);

		for(std::map<double,Scan*>::iterator it=scans.begin();it!=scans.end();it++)
		{
			Transform T=it->second->Tcw().inv(); T.vis(v,it->second->time());
//			pcl::transformPointCloud(*it->second->points(),*cloud,T.getMatrix4f());
//			filter.setInputCloud(cloud);
//			filter.filter(*cloud_all);
//			if (!v->updatePointCloud(cloud_all,std::to_string(it->first)))
//				v->addPointCloud(cloud_all,std::to_string(it->first));
		}
//		cloud->clear();
//		filter.setInputCloud(cloud_all);
//		filter.filter(*cloud);
//		v->addPointCloud(cloud,"map");
//		v->spin();

		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		int j=0;
		for(iterLandmark it=landmarks.begin();it!=landmarks.end();it++,j++)
		{
//			std::cout<<it->second<<std::endl;
//			std::cout<<it->second->sizeObserv()<<endl;
//			iterObserv it_ob=it->second->beginObserv();
//			cout<<fixed<<it_ob->first<<endl;
			for(iterObserv it_obs=it->second->beginObserv();it_obs!=it->second->endObserv();it_obs++)
			{
				double t=it_obs->first;
//				cout<<fixed<<t<<endl;
				Scan* s=scans.find(t)->second;
				std::vector<std::string> id=it_obs->second;
				for(int i=0;i<id.size();i++)
				{
					Feature* f=s->findFeature(id[i]);
//					cout<<f<<endl;
					if(f==0) return;
					f->vis(v,red[j%14],grn[j%14],blu[j%14],std::to_string(t),s->Tcw());
				}
			}
//			for(int i=0;i<it->second->sizeObserv();i++)
//			{
//				double t=it->second->observTime(i);
////				cout<<t<<endl;
//				Scan* s=scans.find(t)->second;
////				s->vis(v,2);
//				std::string id=it->second->observID(i);
////				s->association_map()->findFeature(it->first);
//				Feature* f=s->findFeature(id);
////				cout<<f<<endl;
//				if(f==0) return;
//				f->vis(v,red[j%14],grn[j%14],blu[j%14],std::to_string(t),s->Tcw());
////				if(f->Type()==PLANE) std::cout<<f->plane()->centroid_point.transpose()<<std::endl;
//			}
		}
		v->spin();
	}

	void Map::saveTrajSLAM(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		fp.precision(6); fp<<std::fixed;
		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
		{
			fp<<it->first<<" "<<it->second.inv()<<std::endl;
		}
		fp.close();
	}

	void Map::saveTrajVO(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		fp.precision(6); fp<<std::fixed;
		for(const_iterScan it=scans.begin();it!=scans.end();it++)
		{
			fp<<it->first<<" "<<it->second->Tcg().inv()<<std::endl;
		}
		fp.close();
	}

	void Map::saveTraj(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		printCameras(fp);
		fp.close();
	}

	void Map::saveMap(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		printLandmarks(fp);
		fp.close();
	}

	void Map::loadTraj(const std::string &filename)
	{
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				double t,tx,ty,tz,qx,qy,qz,qw;
				ss>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat(qw,qx,qy,qz);
				Eigen::Vector3d trans(tx,ty,tz);
//					timestamps.push_back(t);
//					cameras.push_back(ulysses::Transform(quat,trans));
				cameras.insert(std::pair<double,Transform>(t,Transform(quat,trans)));
				scans.insert(std::pair<double,Scan*>(t,new Scan(t)));
			}
		}
		fp.close();
	}

	void Map::loadMap(const std::string &filename)
	{
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		std::stringstream ss; 
		std::string id; 
		int numPoints; 
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				ss.clear(); ss<<s; ss>>id;
				if(id.substr(0,10)=="map_plane_")
				{
					Eigen::Vector4d pi;
					ss>>pi(0)>>pi(1)>>pi(2)>>pi(3);
					landmarks.insert(std::pair<std::string,Landmark*>(id,new Landmark(id,pi)));
				}
				else if(id.substr(0,10)=="map_xline_")
				{
					Vector6d L;
					ss>>L(0)>>L(1)>>L(2)>>L(3)>>L(4)>>L(5);
					landmarks.insert(std::pair<std::string,Landmark*>(id,new Landmark(id,L)));
				}
				Landmark* lm=landmarks.find(id)->second;
//				cout<<lm<<endl;
				// num of observ.s;
				ss>>numPoints;
//				cout<<numPoints<<endl;
				for(int i=0;i<numPoints;i++)
				{
					double observ_time;
					std::string observ_id;
					ss>>observ_time>>observ_id;
					lm->pushObserv(observ_time,observ_id);
//					cout<<observ_time<<", "<<observ_id<<endl;
				}
			}
		}
		fp.close();
	}

	Scan::~Scan()
	{
		point_cloud.reset();
		normal_cloud.reset();
		pixel_cloud.reset();
		if(!img_rgb.empty()) img_rgb.release();
		if(!img_depth.empty()) img_depth.release();
		for(iterFeature it=features.begin();it!=features.end();it++) {delete it->second;}
		for(size_t i=0;i<edge_points.size();i++) delete edge_points[i];
		if(feature_association!=0) delete feature_association;
		if(landmark_association!=0) delete landmark_association;
	}

	void Scan::release()
	{
		point_cloud.reset();
		normal_cloud.reset();
		pixel_cloud.reset();
		for(iterFeature it=features.begin();it!=features.end();it++) {it->second->release();}
		for(size_t i=0;i<edge_points.size();i++) 
		{ delete edge_points[i]; }
		edge_points.resize(0);
		if(!img_rgb.empty()) img_rgb.release();
		if(!img_depth.empty()) img_depth.release();
		if(feature_association!=0) 
		{
			delete feature_association;
			feature_association=0;
		}
		if(landmark_association!=0) 
		{
			delete landmark_association;
			landmark_association=0;
		}
	}

	void Scan::addFeature(Feature *f)
	{
		f->setID(features.size());
		features.insert(std::pair<std::string,Feature*>(f->ID(),f));
	}

	Feature* Scan::findFeature(std::string id) 
	{
		iterFeature it=features.find(id);
		if(it==features.end()) 
		{
//			std::cout<<"cannot find "<<id<<std::endl;
			return 0;
		}
		return it->second;
	}

	iterFeature Scan::eraseFeature(std::string id)
	{
		iterFeature it=features.find(id);
		if(it==features.end()) 
		{
//			std::cout<<"cannot erase "<<id<<" because it does not in the scan"<<std::endl;
		}
		else 
		{
			it=features.erase(it);
		}
		return it;
	}

	void Scan::printFeatures(std::ostream &os)
	{
//			printPlanes(os);
//			printLines(os);
		for(iterFeature it=features.begin();it!=features.end();it++)
		{
			os<<it->second<<std::endl;
		}
	}

	void Scan::saveFeatures(const std::string &folder)
	{
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+".txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		for(iterFeature it=features.begin();it!=features.end();it++)
		{
			it->second->print(fp);
	}
//		printFeatures(fp);
//			printFeatures(std::cout);
		fp.close();
	}

	void Scan::saveAssociations(const std::string &folder)
	{
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+"_association.txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		feature_association->print(fp);
		fp.close();
	}

	void Scan::saveAssociationsMap(const std::string &folder)
	{
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+"_association_map.txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		landmark_association->print(fp);
		fp.close();
	}

//	using namespace std;
	void Scan::loadFeatures(const std::string &folder)
	{
//		ofstream ff; ff.open("loadFeatures.txt");
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+".txt";
		std::ifstream fp;
		std::stringstream ss;
		std::string id, s; 
		int numPoints; 
		fp.open(filename,std::ios::in);
		while(!fp.eof())
		{
			getline(fp,s);
//			ff<<s<<endl;
//			cout<<s<<endl;
			if(!s.empty())
			{
				ss.clear(); ss<<s; ss>>id;
				if(id.substr(0,6)=="plane_")
				{
					Eigen::Vector3d n; double d;
					ss>>n(0)>>n(1)>>n(2)>>d;
					features.insert(std::pair<std::string,Feature*>(id,new Feature(id,n,d)));
				}
				else if(id.substr(0,6)=="xline_")
				{
					Eigen::Vector3d u,v;
					ss>>u(0)>>u(1)>>u(2)>>v(0)>>v(1)>>v(2);
					features.insert(std::pair<std::string,Feature*>(id,new Feature(id,u,v)));
				}
			}
			Feature* ft=features.find(id)->second;
			ft->setPtrPoints(point_cloud);
//			cout<<ft<<endl;
			// num of points;
			getline(fp,s);
//			ff<<s<<endl;
//			cout<<s<<endl;
//			ss.str(""); 
			ss.clear(); ss<<s; ss>>numPoints;
//			cout<<numPoints<<endl;
			for(int i=0;i<numPoints;i++)
			{
				getline(fp,s);
//				ff<<s<<endl;
				int index;
//				ss.str(""); 
				ss.clear(); ss<<s; ss>>index;
				ft->pushIndex(index);
			}
		}
		fp.close();
//		ff.close();
	}

	void Scan::savePRC(const std::string &folder, FeatureAssociation *fa)
	{
		std::string filename=folder+"/PRC.txt";
		std::ofstream fp; 
		fp.open(filename,std::ios::app);
		fp.precision(6); fp<<std::fixed;
		double precision, recall;
		feature_association->evalulatePR(fa,precision,recall);
		fp<<time_stamp<<" "<<precision<<" "<<recall<<std::endl;
		fp.close();
	}

	void Scan::savePRCMap(const std::string &folder, LandmarkAssociation *fa)
	{
		std::string filename=folder+"/PRC_map.txt";
		std::ofstream fp; 
		fp.open(filename,std::ios::app);
		fp.precision(6); fp<<std::fixed;
		double precision, recall;
		landmark_association->evalulatePR(fa,precision,recall);
		fp<<time_stamp<<" "<<precision<<" "<<recall<<std::endl;
		fp.close();
	}

	void Scan::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, int frame)
	{
		Transform T;
		if(frame==0) T.setIdentity();
		else if(frame==1) T=T_cg.inv(); // Tgc
		else if(frame==2) T=T_cw.inv(); // Twc

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::transformPointCloud(*point_cloud,*cloud,T.getMatrix4f());

		T.vis(v,time_stamp);
		if (!v->updatePointCloud(cloud,std::to_string(time_stamp)))
			v->addPointCloud(cloud,std::to_string(time_stamp));
	}

	void Scan::visScan(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		vis(v);

		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		int j=0;
		for(iterFeature it=features.begin();it!=features.end();it++,j++)
		{
			it->second->vis(v,red[j%14],grn[j%14],blu[j%14],it->first+std::to_string(time_stamp));
		}
		v->spin();

		if(feature_association==0) return;

		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		vis(v); scan_ref->vis(v);

		for(int i=0;i<feature_association->size();i++)
		{
			Feature* ft_cur=feature_association->getFeature(i);
			Feature* ft_ref=feature_association->getFeatureRef(i);
			ft_cur->vis(v,red[i%14],grn[i%14],blu[i%14],ft_cur->ID()+std::to_string(time_stamp));
//			v->spin();
			ft_ref->vis(v,red[i%14],grn[i%14],blu[i%14],ft_ref->ID()+std::to_string(time_stamp));
//			v->spin();
		}
	}

//	void Scan::visScan(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
//	{
//		v->removeAllPointClouds();
//		v->removeAllCoordinateSystems();
//		vis(v);
//		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
//		int j=0;
//		for(iterFeature it=features.begin();it!=features.end();it++,j++)
//		{
////			for(const_iterLandmark it_lm=map->beginLandmark();it_lm!=map->endLandmark();it_lm++)
////			{
////				Landmark* lm=it_lm->second;
////				for(iterObserv it_obs=lm->beginObserv();it_obs!=lm->endObserv();it_obs++)
////				{
////					if(fabs(it_obs->first-time_stamp)<1e-4)
////					{
////						std::vector<std::string> id=it_obs->second;
////						for(int i=0;i<id.size();i++)
////						{
////							if(id[i]==it->first)
////							{
////								Feature* f=findFeature(id[i]);
////								if(f==0) return;
//////								f->vis(v,red[j%14],grn[j%14],blu[j%14],std::to_string(time_stamp),Tcw());
////								f->vis(v,red[j%14],grn[j%14],blu[j%14],std::to_string(time_stamp));
////							}
////						}
////					}
////				}
////			}
//			it->second->vis(v,red[j%14],grn[j%14],blu[j%14],it->first);
//		}
//	}

	void Scan::loadScan(const double &time, const std::string &file_depth, const std::string &file_rgb)
	{
		time_stamp=time;
		point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;
		// full path of the current rgb and depth image;
//		std::string filename_rgb_full=path+"/"+filename_rgb;
//		std::string filename_depth_full=path+"/"+filename_depth;
		// load the rgb and depth image to cv::Mat;
		// the depth_image is stored as CV_8UC2;
		img_depth=cv::imread(file_depth,cv::IMREAD_UNCHANGED);
		img_rgb=cv::imread(file_rgb,cv::IMREAD_UNCHANGED);
		filename_rgb=file_rgb;
		//cv::imshow("rgb",img_rgb);
		//cv::waitKey(0);
		//cv::imshow("dep",img_depth);
		//cv::waitKey(0);

		// pointer to the Mat data;
		rgb_ptr=img_rgb.data;
		depth_ptr=img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		point_cloud->clear();
		normal_cloud->clear();
		pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<img_depth.rows;i++)
		{
			for(int j=0;j<img_depth.cols;j++)
			{
				// 3 channels for one pixel in rgb image;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
				rgb_ptr++;
				// 2 channels for one pixel in depth image;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				depth_ptr+=2;
//				if(j<300 || j>=800 || i<80 || i>=500)
//					continue;
				point_tmp.z=*depth_tmp_ptr/camera_intrinsic.factor;
				// transformation from pixel coordinate to the camera coordinate;
				// wrong results if considering length of the pixel;
				point_tmp.x=(j-camera_intrinsic.cx)*point_tmp.z/camera_intrinsic.fx;
				point_tmp.y=(i-camera_intrinsic.cy)*point_tmp.z/camera_intrinsic.fy;
				point_cloud->push_back(point_tmp);
			}
		}
		delete depth_tmp_ptr;
		// organize the point_cloud for the normal estimation;
		point_cloud->width=camera_intrinsic.width;//500;//
		point_cloud->height=camera_intrinsic.height;//420;//
		// generate the normal_cloud;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
		normal_estimate_integral.setInputCloud(point_cloud);
		normal_estimate_integral.compute (*normal_cloud);
		// generate the pixel_cloud;
		for(int v=0;v<point_cloud->height;v++)
		{
			for(int u=0;u<point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				pixel_cloud->push_back(tmp_pointxy);
			}
		}
	}

	void Scan::addEdgePoint(int i)
	{
		if(fabs(point_cloud->at(i).z)<1e-4) return;
		EdgePoint* ep=new EdgePoint;
		ep->xyz(0)=point_cloud->at(i).x;
		ep->xyz(1)=point_cloud->at(i).y;
		ep->xyz(2)=point_cloud->at(i).z;
		ep->pixel(0)=pixel_cloud->at(i).x;
		ep->pixel(1)=pixel_cloud->at(i).y;
		ep->idx=i;
		edge_points.push_back(ep);
	}


//	void GlobalMap::addScan(Scan *scan, Map* map)
//	{
//		fp.open("global_map.txt",std::ios::app);
//		if(debug) fp<<"\nGlobalMap::addScan"<<std::endl;
//		// for matches[j].cur;
//		if(map->cameras.size()==0) map->addCamera(Transform(),scan->scan_ref->time_stamp);
//		map->addCamera(map->cameras[map->cameras.size()-1],scan->time_stamp);
//		FeatureAssociation::Iterator iter;
//		for(iter=scan->feature_association->begin();iter!=scan->feature_association->end();iter++)
////		for(size_t j=0;j<scan->plane_matches.size();j++)
//		{ // for each plane observation;
//			if(iter->first->type==Feature::PLANE)
//			{
//				PlaneLM *plane_lm=new PlaneLM(iter->second->plane, Transform(), map->planes.size());
////				PlaneLM *plane_lm=new PlaneLM(scan->plane_matches[j].ref, Transform(), map->planes.size());
//											  //scan->scan_ref->Tcg.inv());
////				plane_lm->id=map->planes.size();
//				plane_lm->indices_cameras.push_back(map->cameras.size()-2);
//				plane_lm->indices_cameras.push_back(map->cameras.size()-1);
//				map->planes.push_back(plane_lm);
//				map->addObservationPlane(iter->second->plane,map->cameras.size()-2,map->planes.size()-1);
//				iter->second->plane->idx_plane=map->planes.size()-1;
//				map->addObservationPlane(iter->first->plane,map->cameras.size()-1,map->planes.size()-1);
//				iter->first->plane->idx_plane=map->planes.size()-1;
////				map->addObservationPlane(scan->plane_matches[j].ref,map->cameras.size()-2,map->planes.size()-1);
////				scan->plane_matches[j].ref->idx_plane=map->planes.size()-1;
////				map->addObservationPlane(scan->plane_matches[j].cur,map->cameras.size()-1,map->planes.size()-1);
////				scan->plane_matches[j].cur->idx_plane=map->planes.size()-1;
//				if(debug) fp<<"\nadd a new plane landmark\n"
//							<<"\tidx_camera\t"<<map->cameras.size()-2<<"\t"<<map->cameras.size()-1
//							<<"\tidx_plane\t" <<map->planes.size()-1
//				            <<"\n\tplane_lm\t"<<plane_lm->pi(0)<<"\t"<<plane_lm->pi(1)<<"\t"<<plane_lm->pi(2)<<"\t"<<plane_lm->pi(3)<<std::endl;
//			}
////		}
////		for(size_t j=0;j<scan->line_matches.size();j++)
////		{ // for each line observation;
//			else // if(iter->first->type==Feature::LINE)
//			{
//				LineLM *line_lm=new LineLM(iter->second->line, Transform(), map->lines.size());
////				LineLM *line_lm=new LineLM(scan->line_matches[j].ref, Transform(), map->lines.size());
//										   //scan->scan_ref->Tcg.inv());
////				line_lm->id=map->lines.size();
//				line_lm->indices_cameras.push_back(map->cameras.size()-2);
//				line_lm->indices_cameras.push_back(map->cameras.size()-1);
//				map->lines.push_back(line_lm);
//				if(debug) fp<<"\nadd a new line landmark and two observations\n"
//							<<"\tidx_camera\t"<<map->cameras.size()-2<<"\t"<<map->cameras.size()-1
//							<<"\tidx_line\t" <<map->lines.size()-1
//				            <<"\n\tline_lm\t"<<line_lm->L(0)<<"\t"<<line_lm->L(1)<<"\t"<<line_lm->L(2)<<"\t"
//											 <<line_lm->L(3)<<"\t"<<line_lm->L(4)<<"\t"<<line_lm->L(5)<<std::endl;
//
//				// add line observation (ref);
//				map->addObservationLine(iter->second->line,map->cameras.size()-2,map->lines.size()-1);
//				iter->second->line->idx_line=map->lines.size()-1;
////				map->addObservationLine(scan->line_matches[j].ref,map->cameras.size()-2,map->lines.size()-1);
////				scan->line_matches[j].ref->idx_line=map->lines.size()-1;
////				for(int i=0;i<scan->plane_matches.size();i++)
////				{
////					map->addObservationShadow(scan->line_matches[j].ref,scan->plane_matches[i].ref,map->cameras.size()-2);
////					int k=map->observ_shadows.size()-1;
////					fp<<std::endl<<"shadow "<<k<<std::endl;
////					fp<<"L "<<map->observ_shadows[k]->L.transpose()<<std::endl;
////					fp<<"sqrt_info "<<std::endl<<map->observ_shadows[k]->sqrt_info<<std::endl;
////					fp<<"cov "<<std::endl<<map->observ_shadows[k]->cov<<std::endl;
////					fp<<"idx_camera "<<map->observ_shadows[k]->idx_camera<<std::endl;
////					fp<<"idx_plane "<<map->observ_shadows[k]->idx_plane<<std::endl;
////					fp<<"idx_line "<<map->observ_shadows[k]->idx_line<<std::endl;
////				}
//
//				// add line observation (cur);
//				map->addObservationLine(iter->first->line,map->cameras.size()-1,map->lines.size()-1);
//				iter->second->line->idx_line=map->lines.size()-1;
////				for(int i=0;i<scan->plane_matches.size();i++)
////				{
////					map->addObservationShadow(scan->line_matches[j].cur,scan->plane_matches[i].cur,map->cameras.size()-1);
////					int k=map->observ_shadows.size()-1;
////					map->observ_shadows[k]->sqrt_info/=double(scan->plane_matches.size());
////					fp<<std::endl<<"shadow "<<k<<std::endl;
////					fp<<"L "<<map->observ_shadows[k]->L.transpose()<<std::endl;
////					fp<<"sqrt_info "<<std::endl<<map->observ_shadows[k]->sqrt_info<<std::endl;
////					fp<<"cov "<<std::endl<<map->observ_shadows[k]->cov<<std::endl;
////					fp<<"idx_camera "<<map->observ_shadows[k]->idx_camera<<std::endl;
////					fp<<"idx_plane "<<map->observ_shadows[k]->idx_plane<<std::endl;
////					fp<<"idx_line "<<map->observ_shadows[k]->idx_line<<std::endl;
////				}
//			}
//		}
//
//		if(debug)
//		{
//			fp<<"\nnum_observ_planes_ = "<<map->observ_planes.size()<<std::endl;
//			fp<<"num_observ_lines_ = "<<map->observ_lines.size()<<std::endl<<std::endl;
//			fp<<"num_cameras_ = "<<map->cameras.size()<<std::endl;
//			fp<<"num_planes_ = "<<map->planes.size()<<std::endl;
//			fp<<"num_lines_ = "<<map->lines.size()<<std::endl<<std::endl;
//
//			fp<<"\nmap->cameras"<<std::endl;
//			for(size_t i=0;i<map->cameras.size();i++)
//			{
//				Eigen::Quaterniond q=map->cameras[i].Quat();
//				fp<<"\t"<<i<<"\t"<<q.w()<<"\t"<<q.vec().transpose()<<"\t"<<map->cameras[i].t.transpose()<<std::endl;
//				fp<<"\t\tplanes";
//				for(int j=0;j<map->indices_planes[i].size();j++)
//				{
//					fp<<"\t"<<map->indices_planes[i][j];
//				}
//				fp<<std::endl;
//				fp<<"\t\tlines";
//				for(int j=0;j<map->indices_lines[i].size();j++)
//				{
//					fp<<"\t"<<map->indices_lines[i][j];
//				}
//				fp<<std::endl;
//			}
//
//			fp<<"\nmap->planes"<<std::endl;
//			for(size_t i=0;i<map->planes.size();i++)
//			{
//				fp<<"\t"<<i<<"\t"<<map->planes[i]->pi.transpose()<<std::endl;
//				fp<<"\t\tobserved in";
//				for(int j=0;j<map->planes[i]->indices_cameras.size();j++)
//				{
//					fp<<"\t"<<map->planes[i]->indices_cameras[j];
//				}
//				fp<<std::endl;
//			}
//
//			fp<<"\nmap->lines"<<std::endl;
//			for(size_t i=0;i<map->lines.size();i++)
//			{
//				fp<<"\t"<<i<<"\t"<<map->lines[i]->L.transpose()<<std::endl;
//				fp<<"\t\tobserved in";
//				for(int j=0;j<map->lines[i]->indices_cameras.size();j++)
//				{
//					fp<<"\t"<<map->lines[i]->indices_cameras[j];
//				}
//				fp<<std::endl;
//			}
//
//			fp<<"\nmap->observ_planes"<<std::endl;
//			for(size_t i=0;i<map->observ_planes.size();i++)
//			{
//				fp<<"\t"<<i<<"\t"<<map->observ_planes[i]->idx_camera<<"\t"<<map->observ_planes[i]->idx_plane<<"\t"<<map->observ_planes[i]->pi.transpose()<<std::endl;
//			}
//
//			fp<<"\nmap->observ_lines"<<std::endl;
//			for(size_t i=0;i<map->observ_lines.size();i++)
//			{
//				fp<<"\t"<<i<<"\t"<<map->observ_lines[i]->idx_camera<<"\t"<<map->observ_lines[i]->idx_line<<"\t"<<map->observ_lines[i]->L.transpose()<<std::endl;
//			}
//
////			fp<<"\nmap->observ_shadows"<<std::endl;
////			for(size_t i=0;i<map->observ_shadows.size();i++)
////			{
////				fp<<"\t"<<i<<"\t"<<map->observ_shadows[i]->idx_camera<<"\t"<<map->observ_shadows[i]->idx_plane<<"\t"<<map->observ_shadows[i]->idx_line<<"\t"<<map->observ_shadows[i]->L.transpose()<<std::endl;
////			}
//		}
//
//		fp.close();
//	}

//	void GlobalMap::computeWeight(Map* map)
//	{
//		std::ofstream fp;
//		fp.open("computeWeight.txt",std::ios::app);
//		// print Psi before weighting;
//		// *****************************************
//		Matrix6d Psi=Matrix6d::Zero();
//		for(int i=0;i<map->observ_planes.size();i++)
//		{ Psi+=map->observ_planes[i]->Psi; }
//		for(int i=0;i<map->observ_lines.size();i++)
//		{ Psi+=map->observ_lines[i]->Psi; }
////		for(int i=0;i<map->observ_shadows.size();i++)
////		{ Psi+=map->observ_shadows[i]->Psi; }
//		Eigen::SelfAdjointEigenSolver<Matrix6d> ess;
//		ess.compute(Psi);
//		Vector6d Lambda=ess.eigenvalues();
//		Matrix6d Q=ess.eigenvectors();
//		Lambda=Lambda.cwiseAbs();
//		Lambda.normalize();
//		fp<<std::endl<<"Psi before weighting\t"
//		  <<Lambda.transpose()<<std::endl;
//		fp<<Q<<std::endl<<std::endl;
//		// *****************************************
//		// end of printing
//
//		Matrix6d Psi_pi=Matrix6d::Zero();
//		for(int i=0;i<map->observ_planes.size();i++)
//		{ Psi_pi+=map->observ_planes[i]->Psi; }
//		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
//		es.compute(Psi_pi);
//		Vector6d Lambda_pi=es.eigenvalues();
//		Matrix6d Q_pi=es.eigenvectors();
//		Lambda_pi=Lambda_pi.cwiseAbs();
//		double sum_Lambda_pi=0;
//		for(int i=0;i<6;i++) sum_Lambda_pi+=Lambda_pi(i);
//		fp<<"Lambda_pi "<<Lambda_pi.transpose()<<std::endl<<std::endl;
//		Lambda_pi.normalize();
//
//		for(int i=0;i<map->observ_lines.size();i++)
//		{
//			Matrix6d Lambda_Lk_mat=Q_pi.transpose()*map->observ_lines[i]->Psi*Q_pi;
//			Vector6d Lambda_Lk=Lambda_Lk_mat.diagonal();
//			Lambda_Lk=Lambda_Lk.cwiseAbs();
//			double sum_Lambda_Lk=0;
//			for(int j=0;j<6;j++) sum_Lambda_Lk+=Lambda_Lk(j);
//			map->observ_lines[i]->scale=(sum_Lambda_pi/sum_Lambda_Lk);
//			fp<<i<<"th Lambda_Lk "<<Lambda_Lk.transpose()<<std::endl;
//			Lambda_Lk.normalize();
//			Vector6d delta=Lambda_pi-Lambda_Lk;
//			Eigen::Matrix<double,1,1> tmp=delta.transpose()*delta;
//			map->observ_lines[i]->weight=0.5*tmp(0,0);
//
//			map->observ_lines[i]->sqrt_info*=sqrt(map->observ_lines[i]->weight*map->observ_lines[i]->scale);
//			fp<<i<<"th line - weight="<<map->observ_lines[i]->weight<<", scale="<<map->observ_lines[i]->scale<<std::endl<<std::endl;
//			Psi_pi+=map->observ_lines[i]->weight*map->observ_lines[i]->Psi;
//		}
//
//		es.compute(Psi_pi);
//		Lambda_pi=es.eigenvalues();
//		Q_pi=es.eigenvectors();
//		Lambda_pi=Lambda_pi.cwiseAbs();
//		Lambda_pi.normalize();
//
////		for(int i=0;i<map->observ_shadows.size();i++)
////		{
////			Matrix6d Lambda_Lk_mat=Q_pi.transpose()*map->observ_shadows[i]->Psi*Q_pi;
////			Vector6d Lambda_Lk=Lambda_Lk_mat.diagonal();
////			Lambda_Lk=Lambda_Lk.cwiseAbs();
////			double sum_Lambda_Lk=0;
////			for(int j=0;j<6;j++) sum_Lambda_Lk+=Lambda_Lk(j);
////			map->observ_shadows[i]->scale=(sum_Lambda_pi/sum_Lambda_Lk);
////			fp<<i<<"th Lambda_Lk "<<Lambda_Lk.transpose()<<std::endl;
////			Lambda_Lk.normalize();
////			Vector6d delta=Lambda_pi-Lambda_Lk;
////			Eigen::Matrix<double,1,1> tmp=delta.transpose()*delta;
////			map->observ_shadows[i]->weight=0.5*tmp(0,0);
////
////			map->observ_shadows[i]->sqrt_info*=sqrt(map->observ_shadows[i]->weight*map->observ_shadows[i]->scale);
////			fp<<i<<"th shadow - weight="<<map->observ_shadows[i]->weight<<", scale="<<map->observ_shadows[i]->scale<<std::endl<<std::endl;
////			Psi_pi+=map->observ_shadows[i]->weight*map->observ_shadows[i]->Psi;
////		}
//
//		// print Psi after weighting;
//		// *****************************************
//		Psi=Matrix6d::Zero();
//		for(int i=0;i<map->observ_planes.size();i++)
//		{ Psi+=map->observ_planes[i]->Psi; }
//		for(int i=0;i<map->observ_lines.size();i++)
//		{ Psi+=map->observ_lines[i]->scale*map->observ_lines[i]->weight*map->observ_lines[i]->Psi; }
//		if(use_shadow)
////		for(int i=0;i<map->observ_shadows.size();i++)
////		{ Psi+=map->observ_shadows[i]->scale*map->observ_shadows[i]->weight*map->observ_shadows[i]->Psi; }
//		ess.compute(Psi);
//		Lambda=ess.eigenvalues();
//		Q=ess.eigenvectors();
//		Lambda=Lambda.cwiseAbs();
//		Lambda.normalize();
//		fp<<"Psi before weighting\t"
//		  <<Lambda.transpose()<<std::endl;
//		fp<<Q<<std::endl<<std::endl;
//		// *****************************************
//		// end of printing
//		fp.close();
//	}

//	void Map::addObservationPlane(Plane *p, int idx_camera, int idx_plane)
//	{
//		observation_plane *plane=new observation_plane;
//		plane->pi.block<3,1>(0,0)=p->n;
//		plane->pi(3)=p->d;
//		plane->sqrt_info=p->sqrt_info;
//		plane->idx_camera=idx_camera;
//		plane->idx_plane=idx_plane;
//		plane->plane_ptr=p;
//		observ_planes.push_back(plane);
//		indices_planes[idx_camera].push_back(idx_plane);
//		computePsiPlane(plane);
//	}
//
//	void Map::computePsiPlane(observation_plane* plane)
//	{
//		// compute plane->Psi;
//		Eigen::Matrix3d R=cameras[plane->idx_camera].R;
//		Eigen::Vector3d t=cameras[plane->idx_camera].t;
//		Eigen::Vector4d pi=planes[plane->idx_plane]->pi;
//		Eigen::Vector3d n=pi.block<3,1>(0,0);
//		n.normalize();
//
//		Eigen::Matrix<double,4,6> e_pi_xi;
//		Eigen::Vector3d Rn=R*n;
//		e_pi_xi.block<3,3>(0,0).setZero();
//		e_pi_xi.block<3,3>(0,3)=Transform::skew_sym(Rn);
//		e_pi_xi.block<1,3>(3,0)=Rn.transpose();
//		e_pi_xi.block<1,3>(3,3)=-t.transpose()*Transform::skew_sym(Rn);
//
//		plane->Psi=e_pi_xi.transpose()*plane->sqrt_info*plane->sqrt_info*e_pi_xi;
//	}
//
//	void Map::addObservationLine(Line *l, int idx_camera, int idx_line)
//	{
//		observation_line *line=new observation_line;
//		line->L.block<3,1>(0,0)=l->u;
//		line->L.block<3,1>(3,0)=l->v;
//		line->sqrt_info=l->sqrt_info;
//		line->idx_camera=idx_camera;
//		line->idx_line=idx_line;
//		line->line_ptr=l;
//
//		Vector6d l_cur=cameras[idx_camera].getLineTransform()*lines[idx_line]->L;
//		double tmp=l_cur.block<3,1>(0,0).transpose()*l->u;
//		if(tmp<0) line->L=-line->L;
//
//		observ_lines.push_back(line);
//		indices_lines[idx_camera].push_back(idx_line);
//
//		computePsiLine(line);
//	}
//
//	void Map::computePsiLine(observation_line* line)
//	{
//		// compute line->Psi;
//		Eigen::Matrix3d R=cameras[line->idx_camera].R;
//		Eigen::Vector3d t=cameras[line->idx_camera].t;
//		Vector6d L=lines[line->idx_line]->L;
//		Eigen::Vector3d u=L.block<3,1>(0,0);
//		Eigen::Vector3d v=L.block<3,1>(3,0);
//		u=u/v.norm();
//		v.normalize();
//
//		Matrix6d e_L_xi;
//		Eigen::Matrix3d Ru=Transform::skew_sym(R*u);
//		Eigen::Matrix3d Rv=Transform::skew_sym(R*v);
//		e_L_xi.block<3,3>(0,0)=Rv;
//		e_L_xi.block<3,3>(0,3)=Ru+Transform::skew_sym(t)*Rv;
//		e_L_xi.block<3,3>(3,0).setZero();
//		e_L_xi.block<3,3>(3,3)=Rv;
//
//		line->Psi=e_L_xi.transpose()*line->sqrt_info*line->sqrt_info*e_L_xi;
//	}

//	void Map::addObservationShadow(Line *l, Plane *p, int idx_camera)
//	{
//		observation_shadow *line=new observation_shadow;
//
//		Eigen::Vector3d u=l->u;
//		Eigen::Vector3d n=p->n;
//		double d=p->d;
//
//		line->L.block<3,1>(0,0)=d*u;
//		line->L.block<3,1>(3,0)=-u.cross(n);
//		line->L/=line->L.block<3,1>(3,0).norm();
//
//		Vector6d l_cur=cameras[idx_camera].getLineTransform()*lines[l->idx_line]->L;
//		double tmp=l_cur.block<3,1>(0,0).transpose()*l->u;
//		if(tmp<0) line->L=-line->L;
//
//		// compute line->cov;
//		Eigen::Matrix3d Cuu=l->cov.block<3,3>(0,0);
//		Eigen::Matrix3d Cnn=p->cov.block<3,3>(0,0);
//		Eigen::Vector3d Cnd=p->cov.block<3,1>(0,3);
//		double Cdd=p->cov(3,3);
//
//		line->cov.block<3,3>(0,0)=d*d*Cuu+Cdd*u*u.transpose();
//		line->cov.block<3,3>(0,3)=d*Cuu*Transform::skew_sym(n).transpose()
//								  +u*Cnd.transpose()*Transform::skew_sym(u);
//		line->cov.block<3,3>(3,0)=line->cov.block<3,3>(0,3).transpose();
//		line->cov.block<3,3>(3,3)=Transform::skew_sym(n)*Cuu*Transform::skew_sym(n).transpose()
//								  +Transform::skew_sym(u)*Cnn*Transform::skew_sym(u).transpose();
//
//		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
//		es.compute(line->cov);
//		Vector6d Lambda=es.eigenvalues();
//		Matrix6d U=es.eigenvectors();
//		Matrix6d sqrt_inv_Lambda=Matrix6d::Zero();
//		for(size_t i=0;i<6;i++)
//		{
//			if(Lambda(i)>0.01)
//			{
//				sqrt_inv_Lambda(i,i)=1.0/sqrt(Lambda(i));
//			}
//		}
//		line->sqrt_info=U*sqrt_inv_Lambda*U.transpose();
//
//		line->idx_camera=idx_camera;
//		line->idx_plane=p->idx_plane;
//		line->idx_line=l->idx_line;
//		observ_shadows.push_back(line);
//
//		computePsiShadow(line);
//	}
//		
//	void Map::computePsiShadow(observation_shadow* line)
//	{
//		// compute shadow->Psi;
//		Eigen::Matrix3d R=cameras[line->idx_camera].R;
//		Eigen::Vector3d t=cameras[line->idx_camera].t;
//		Eigen::Vector4d pi=planes[line->idx_plane]->pi;
//		Eigen::Vector3d n=pi.block<3,1>(0,0);
//		double d=pi(3)/n.norm();
//		n.normalize();
//
//		Vector6d L=lines[line->idx_line]->L;
//		Eigen::Vector3d u=L.block<3,1>(0,0);
//		Eigen::Vector3d v=L.block<3,1>(3,0);
//		u=u/v.norm();
//		v.normalize();
//
//		Eigen::Vector3d t_gc=-R.transpose()*t;
//		Eigen::Vector3d uu=-d*(Transform::skew_sym(v)*t_gc+u)-n*t_gc.transpose()*u;
//		Eigen::Vector3d vv=-Transform::skew_sym(n)*Transform::skew_sym(v)*t_gc+Transform::skew_sym(u)*n;
//		double vv_norm=vv.norm();
//		uu=uu/vv_norm;
//		vv.transpose();
//
//		Eigen::Matrix3d Rv_pi=Transform::skew_sym(R*vv);
//		Eigen::Matrix3d Ru_pi=Transform::skew_sym(R*uu);
//
//		Matrix6d tmp_6d;
//		tmp_6d.block<3,3>(0,0)=R;
//		tmp_6d.block<3,3>(0,3)=Transform::skew_sym(t)*R;
//		tmp_6d.block<3,3>(3,0).setZero();
//		tmp_6d.block<3,3>(3,3)=R;
//
//		Eigen::Matrix<double,6,3> tmp_63;
//		tmp_63.block<3,3>(0,0)=d*Transform::skew_sym(v)+n*u.transpose()
//							   -uu*vv.transpose()*Transform::skew_sym(n)*Transform::skew_sym(v);
//		tmp_63.block<3,3>(3,0)=(Eigen::Matrix3d::Identity()-vv*vv.transpose())
//							   *Transform::skew_sym(n)*Transform::skew_sym(v);
//
//		Eigen::Matrix<double,3,6> tmp_36;
//		tmp_36.block<3,3>(0,0).setIdentity();
//		tmp_36.block<3,3>(0,3)=Transform::skew_sym(t);
//
//		tmp_6d=tmp_6d*tmp_63*R.transpose()*tmp_36;
//		tmp_6d/=-vv_norm;
//
//		Matrix6d e_Lpi_xi;
//		e_Lpi_xi.block<3,3>(0,0)=tmp_6d.block<3,3>(0,0)+Rv_pi;
//		e_Lpi_xi.block<3,3>(0,3)=tmp_6d.block<3,3>(0,3)+Ru_pi+Transform::skew_sym(t)*Rv_pi;
//		e_Lpi_xi.block<3,3>(3,0)=tmp_6d.block<3,3>(3,0);
//		e_Lpi_xi.block<3,3>(3,3)=tmp_6d.block<3,3>(3,3)+Rv_pi;
//
//		line->Psi=e_Lpi_xi.transpose()*line->sqrt_info*line->sqrt_info*e_Lpi_xi;
//
//	}

//	Map::~Map()
//	{
//		for(int i=0;i<planes.size();i++) delete planes[i];
//		for(int i=0;i<lines.size();i++) delete lines[i];
//		for(int i=0;i<observ_planes.size();i++) delete observ_planes[i];
//		for(int i=0;i<observ_lines.size();i++) delete observ_lines[i];
//		for(int i=0;i<observ_shadows.size();i++) delete observ_shadows[i];

//		std::vector<double>().swap(timestamps);
//		std::vector<Transform>().swap(cameras);
//		std::vector<PlaneLM*>().swap(planes);
//		std::vector<LineLM*>().swap(lines);
//		std::vector<Indices>().swap(indices_planes);
//		std::vector<Indices>().swap(indices_lines);
//		std::vector<observation_plane*>().swap(observ_planes);
//		std::vector<observation_line*>().swap(observ_lines);
//		std::vector<observation_shadow*>().swap(observ_shadows);
//	}
}
