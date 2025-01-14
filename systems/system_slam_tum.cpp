/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-07 15:02
#
# Filename:		system_slam_tum.cpp
#
# Description: 
#
************************************************/
#include "systems/system_slam_tum.h"

namespace ulysses 
{
//	CameraIntrinsic camera_intrinsic;

	SystemSLAMTum::SystemSLAMTum(const std::string &settingFile)
	{
		remove("SystemSLAMTum.txt");
		std::string load_folder;

		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["debug"]>>debug;
		settings["loadFolder"]>>load_folder;
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

		loadImages(load_folder);

//		if(plane_method==1) plane_extraction=new PlaneExtraction(settingFile);
//		else if(plane_method==2) 
		plane_segmentation=new PlaneSegmentation(settingFile);
//		
		line_extraction=new LineExtraction(settingFile);
		feature_matching=new GeometricFeatureMatching(settingFile);
//		motion_estimation=new MotionEstimation(settingFile);

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

		if(!debug) rpe=new RelativePoseError(save_folder);
	}

	double SystemSLAMTum::track(const double &time_stamp, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		fp.open("SystemSLAMTum.txt",std::ios::app);
		fp<<std::endl<<"****************************SystemSLAMTum::track()*******************************"<<std::endl;
//		timeval time_start, time_end;
		double time_used=0,time_delta;

		scan_cur=new Scan(time_stamp);
		scan_cur->loadScan(time_stamp,files_depth.find(time_stamp)->second,files_rgb.find(time_stamp)->second);
//		scan_cur->loadFeatures(load_folder);
//		current_time+=time_delta;
//		scan_cur->Tcg.setIdentity();
//		num_scan++;
		bool isFirstFrame=false;

		fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
//		std::cout<<std::fixed<<scan_cur->time()<<"-----------------------"<<std::endl;

		if(scan_ref==0) 
		{
			isFirstFrame=true;
		}
		else 
		{
			scan_cur->ref()=scan_ref;
		}

		// extract features;
		time_delta=plane_segmentation->extractPlanes(scan_cur);
		cout<<"plane segmentation: "<<time_delta<<std::endl;
		time_used+=time_delta;
		if(debug) {plane_segmentation->visPlanes(scan_cur,vis); vis->spin();}

//		line_extraction->vis=vis;
		if (!vis->updatePointCloud (scan_cur->points(),"scan")) vis->addPointCloud (scan_cur->points(),"scan");

		time_delta=line_extraction->extractLines(scan_cur);
		cout<<"line extraction: "<<time_delta<<std::endl;
		time_used+=time_delta;
		if(debug) {line_extraction->visLines(scan_cur,vis); vis->spin();}

		fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);

		if(isFirstFrame)
		{
			map->addScan(scan_cur,isFirstFrame);
//			for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
//			{
//				map->addLandmark(new Landmark(it->second,scan_cur->Tcg()));
//			}
			map->printLandmarks(fp);

			visScan(scan_cur,vis); vis->spin();
		}
		else // if(!isFirstFrame)
		{
			time_delta=feature_matching->match(scan_cur,vis);
			cout<<"feature extraction: "<<time_delta<<std::endl;
			time_used+=time_delta;
			scan_cur->localize();

			fp<<"estimated feature associations"<<std::endl;
			scan_cur->association()->print(fp);
			if(debug) scan_cur->association()->vis(scan_cur,vis);

//			visScan(scan_cur,vis); vis->spin();

//			map->addAssociation(scan_cur->association());
			map->addScan(scan_cur);

			if(!debug) scan_cur->saveAssociations(save_folder);

			delete scan_ref;
		}
//		map->addCamera(scan_cur->Tcg(),scan_cur->time());
//		if(!debug) rpe->evaluate(time_stamp,map_true->camera(time_stamp),scan_cur->Tcg());

		scan_ref=scan_cur;

		fp<<"***************************************************************************************"<<std::endl<<std::endl;
//		std::cout<<"scan_cur="<<scan_cur<<", "<<scan_cur->association_sim()<<std::endl;
		fp.close();
		return time_used;
	}

	void SystemSLAMTum::loadImages(const std::string &folder)
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

	/*
	bool SystemSLAMTum::trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
										boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag)
	{
		timeval time_start, time_end;
		double time_used;

		scan_cur=new Scan(timestamp);
//		num_scan++;
		isFirstFrame=false;

		loadScan(scan_cur,rgb,depth);
		scan_cur->Tcw=Twc.inv();

		if(scan_ref==0) isFirstFrame=true;
		else scan_cur->setRef(scan_ref);

		if(plane_method==1) plane_extraction->extractPlanes(scan_cur,vis);
		else if(plane_method==2) plane_segmentation->extractPlanes(scan_cur,vis);

		edge_extraction->extractEdgePoints(scan_cur,vis);
		line_extraction->extractLines(scan_cur,vis);

		if(!isFirstFrame)
		{
			feature_matching->match(scan_cur);
			vis->spin();

			map=new Map;
			global_map->addScan(scan_cur,map);
			//global_map->computeWeight(map);

			ulysses::Transform Tcr;
			Tcr.setIdentity();
			if(motion_estimation->alignScans(scan_cur,map,Tcr,optimize_frames,vis))
			{
				scan_cur->Tcr=Tcr;
				scan_cur->Tcg=scan_cur->Tcr*scan_ref->Tcg; // Tcg=Tcr*Trg;

				std::cout<<"Tcr "<<std::endl<<Tcr.getMatrix4f()<<std::endl;

				timestamps.push_back(timestamp);
				traj.push_back(scan_cur->Tcg);
				
			}

			delete scan_ref;
			delete map;
		}

		if(flag) vis_addScan(scan_cur,scan_cur->Tcg.inv(),vis);

		scan_ref=scan_cur;
		return true;
	}
	*/

//	void SystemSLAMTum::showScans(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
//										boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc,
//										bool flag)
//	{
//		if(flag)
//		{
//			timeval time_start, time_end;
//			double time_used;
//
//			scan_cur=new Scan(timestamp);
//			num_scan++;
//			isFirstFrame=false;
//
//			loadScan(scan_cur,rgb,depth);
//	//		scan_cur->Tcw=Twc.inv();
//		}
//
//		vis_addScan(scan_cur,Twc,vis,flag);
//	}
//
//	void SystemSLAMTum::saveTraj(const std::string &file)
//	{
//		std::cout<<std::endl<<"saving trajectory to "<<file<<std::endl;
//		std::ofstream fp;
//		fp.open(file.c_str(),std::ios::out);
//		for(int i=0;i<traj.size();i++)
//		{
//			ulysses::Transform Tgc=traj[i].inv();
//			fp<<std::fixed<<timestamps[i]<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
//		}
//		fp.close();
//		std::cout<<"trajectory saved"<<std::endl;
//	}

//	void SystemSLAMTum::loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth)
//	{
//		scan->points()=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		scan->normals()=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
//		scan->pixels()=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);
//
////		scan->time_stamp=timestamp_depth;
//		cv::Mat rgb_image,depth_image;
//		uint8_t *depth_ptr,*rgb_ptr;
//		pcl::PointXYZRGBA point_tmp;
//		unsigned short *depth_tmp_ptr=new unsigned short;
//		pcl::PointXY tmp_pointxy;
//		// full path of the current rgb and depth image;
////		std::string filename_rgb_full=path+"/"+filename_rgb;
////		std::string filename_depth_full=path+"/"+filename_depth;
//		// load the rgb and depth image to cv::Mat;
//		// the depth_image is stored as CV_8UC2;
//		scan->imgRGB()=rgb;//cv::imread(filename_rgb_full);
//		scan->imgDepth()=depth;//cv::imread(filename_depth_full,-1);
//		//cv::imshow("rgb",scan->img_rgb);
//		//cv::waitKey(0);
//		//cv::imshow("dep",scan->img_depth);
//		//cv::waitKey(0);
//
//		// pointer to the Mat data;
//		rgb_ptr=scan->imgRGB().data;
//		depth_ptr=scan->imgDepth().data;
//		// clear the pointcloud;
//		// the allocated memory does not release;
//		// the newly pushed elements cover the old ones;
//		scan->points()->clear();
//		scan->normals()->clear();
//		scan->pixels()->clear();
//		// generate the point_cloud;
//		for(int i=0;i<scan->imgDepth().rows;i++)
//		{
//			for(int j=0;j<scan->imgDepth().cols;j++)
//			{
//				// 3 channels for one pixel in rgb image;
//				point_tmp.b=*rgb_ptr;
//				rgb_ptr++;
//				point_tmp.g=*rgb_ptr;
//				rgb_ptr++;
//				point_tmp.r=*rgb_ptr;
//				rgb_ptr++;
//				// 2 channels for one pixel in depth image;
//				memcpy(depth_tmp_ptr,depth_ptr,2);
//				depth_ptr+=2;
////				if(j<300 || j>=800 || i<80 || i>=500)
////					continue;
//				point_tmp.z=*depth_tmp_ptr/camera_intrinsic.factor;
//				// transformation from pixel coordinate to the camera coordinate;
//				// wrong results if considering length of the pixel;
//				point_tmp.x=(j-camera_intrinsic.cx)*point_tmp.z/camera_intrinsic.fx;
//				point_tmp.y=(i-camera_intrinsic.cy)*point_tmp.z/camera_intrinsic.fy;
//				scan->points()->push_back(point_tmp);
//			}
//		}
//		delete depth_tmp_ptr;
//		// organize the point_cloud for the normal estimation;
//		scan->points()->width=camera_intrinsic.width;//500;//
//		scan->points()->height=camera_intrinsic.height;//420;//
//		// generate the normal_cloud;
//		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
//		normal_estimate_integral.setInputCloud(scan->points());
//		normal_estimate_integral.compute (*scan->normals());
//		// generate the pixel_cloud;
//		for(int v=0;v<scan->points()->height;v++)
//		{
//			for(int u=0;u<scan->points()->width;u++)
//			{
//				tmp_pointxy.x=u;
//				tmp_pointxy.y=v;
//				scan->pixels()->push_back(tmp_pointxy);
//			}
//		}
//
////		Eigen::Quaterniond quat(qw,qx,qy,qz);
////		Eigen::Vector3d vec3d(tx,ty,tz);
////		Transform Tc(quat,vec3d);
////		Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
////		Tcg.inverse();
////		scan->Tcg_gt=Tcg;
////
////		scan->Tcg_gt=Tc.inv()*Tg;
//	}

	void SystemSLAMTum::visScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		Transform Tgc=scan->Tcg().inv();
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

		// origin of the reference frame;
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0.3; pt2.y=0; pt2.z=0; 
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,"x");
		pt2.x=0; pt2.y=0.3; pt2.z=0; 
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,"y");
		pt2.x=0; pt2.y=0; pt2.z=0.3; 
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,"z");

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
