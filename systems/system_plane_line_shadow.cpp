/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-04 19:52
#
# Filename:		system_plane_line_shadow.cpp
#
# Description: 
#
************************************************/
#include "systems/system_plane_line_shadow.h"

namespace ulysses 
{
	CameraIntrinsic camera_intrinsic;

	SystemPlaneLine::SystemPlaneLine(const std::string &seq_path, const std::string &setting_file)
	{
		bool debug;

		std::string settingFile=setting_file;
		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["debug"]>>debug;
		settings["PlaneExtractionMethod"]>>plane_method;
		settings["OptimizeFrames"]>>optimize_frames;
		settings["Camera.fx"]>>camera_intrinsic.fx;
		settings["Camera.fy"]>>camera_intrinsic.fy;
		settings["Camera.cx"]>>camera_intrinsic.cx;
		settings["Camera.cy"]>>camera_intrinsic.cy;
		settings["Camera.width"]>>camera_intrinsic.width;
		settings["Camera.height"]>>camera_intrinsic.height;
		settings["Camera.factor"]>>camera_intrinsic.factor;
		settings.release();

		if(plane_method==1) plane_extraction=new PlaneExtraction(settingFile);
		else if(plane_method==2) plane_segmentation=new PlaneSegmentation(settingFile);
		
		line_extraction=new LineExtraction(settingFile);
		edge_extraction=new EdgePointExtraction(settingFile);
		feature_matching=new GeometricFeatureMatching(settingFile);
		motion_estimation=new MotionEstimation(settingFile);

		global_map=new GlobalMap(settingFile);
		global_map->setDebug(debug);
		scan_cur=0; scan_ref=0; num_scan=0; 
	}

	bool SystemPlaneLine::trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
										boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc, bool flag)
	{
		timeval time_start, time_end;
		double time_used;

		scan_cur=new Scan(timestamp);
		num_scan++;
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

	void SystemPlaneLine::showScans(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
										boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, Transform Twc,
										bool flag)
	{
		if(flag)
		{
			timeval time_start, time_end;
			double time_used;

			scan_cur=new Scan(timestamp);
			num_scan++;
			isFirstFrame=false;

			loadScan(scan_cur,rgb,depth);
	//		scan_cur->Tcw=Twc.inv();
		}

		vis_addScan(scan_cur,Twc,vis,flag);
	}

	void SystemPlaneLine::saveTraj(const std::string &file)
	{
		std::cout<<std::endl<<"saving trajectory to "<<file<<std::endl;
		std::ofstream fp;
		fp.open(file.c_str(),std::ios::out);
		for(int i=0;i<traj.size();i++)
		{
			ulysses::Transform Tgc=traj[i].inv();
			fp<<std::fixed<<timestamps[i]<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
		}
		fp.close();
		std::cout<<"trajectory saved"<<std::endl;
	}

	void SystemPlaneLine::loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth)
	{
		scan->point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan->normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		scan->pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

//		scan->time_stamp=timestamp_depth;
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
		scan->img_rgb=rgb;//cv::imread(filename_rgb_full);
		scan->img_depth=depth;//cv::imread(filename_depth_full,-1);
		//cv::imshow("rgb",scan->img_rgb);
		//cv::waitKey(0);
		//cv::imshow("dep",scan->img_depth);
		//cv::waitKey(0);

		// pointer to the Mat data;
		rgb_ptr=scan->img_rgb.data;
		depth_ptr=scan->img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		scan->point_cloud->clear();
		scan->normal_cloud->clear();
		scan->pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<scan->img_depth.rows;i++)
		{
			for(int j=0;j<scan->img_depth.cols;j++)
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
				scan->point_cloud->push_back(point_tmp);
			}
		}
		delete depth_tmp_ptr;
		// organize the point_cloud for the normal estimation;
		scan->point_cloud->width=camera_intrinsic.width;//500;//
		scan->point_cloud->height=camera_intrinsic.height;//420;//
		// generate the normal_cloud;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
		normal_estimate_integral.setInputCloud(scan->point_cloud);
		normal_estimate_integral.compute (*scan->normal_cloud);
		// generate the pixel_cloud;
		for(int v=0;v<scan->point_cloud->height;v++)
		{
			for(int u=0;u<scan->point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				scan->pixel_cloud->push_back(tmp_pointxy);
			}
		}

//		Eigen::Quaterniond quat(qw,qx,qy,qz);
//		Eigen::Vector3d vec3d(tx,ty,tz);
//		Transform Tc(quat,vec3d);
//		Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
//		Tcg.inverse();
//		scan->Tcg_gt=Tcg;
//
//		scan->Tcg_gt=Tc.inv()*Tg;
	}

	void SystemPlaneLine::shutDown()
	{
		delete scan_cur;
//		delete map;
//		delete intr_cam;
		if(plane_method==1) delete plane_extraction;
		else if(plane_method==2) delete plane_segmentation;
		delete line_extraction;
		delete edge_extraction;
		delete feature_matching;
		delete motion_estimation;
		delete global_map;
	}

	void vis_addScan(Scan *scan, Transform Twc, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis, bool flag)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		Transform Tgt=scan->Tcw.inv();

		if(flag)
		{
			// add point cloud;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

			pcl::transformPointCloud(*scan->point_cloud,*cloud,Twc.getMatrix4f());
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

			sprintf(id,"scan%f", scan->time_stamp);
			if (!vis->updatePointCloud (cloud_filtered,id))
				vis->addPointCloud (cloud_filtered,id);
		}

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
		Transform Tgc=Twc;
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
		sprintf(id,"%fx",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		// y - green
		pt2.x=pt1.x+y(0)*scale;
		pt2.y=pt1.y+y(1)*scale;
		pt2.z=pt1.z+y(2)*scale;
		sprintf(id,"%fy",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		// z - green
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		sprintf(id,"%fz",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);

		// ===================================================
		// add groundtruth;
		Tgc=Tgt;
		x=Tgc.R.block<3,1>(0,0);
		y=Tgc.R.block<3,1>(0,1);
		z=Tgc.R.block<3,1>(0,2);
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		// x - magenta
		pt2.x=pt1.x+x(0)*scale;
		pt2.y=pt1.y+x(1)*scale;
		pt2.z=pt1.z+x(2)*scale;
		sprintf(id,"%fxgt",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,255,id);
		// y - yellow
		pt2.x=pt1.x+y(0)*scale;
		pt2.y=pt1.y+y(1)*scale;
		pt2.z=pt1.z+y(2)*scale;
		sprintf(id,"%fygt",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,0,id);
		// z - cyan
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		sprintf(id,"%fzgt",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,255,id);

		// trajectory
		pt2.x=Twc.t(0);
		pt2.y=Twc.t(1);
		pt2.z=Twc.t(2);
		sprintf(id,"%ftraj",scan->time_stamp);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);

	}
}
