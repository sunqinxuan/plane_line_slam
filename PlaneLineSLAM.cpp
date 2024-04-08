/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-07-24 15:05
#
# Filename:		PlaneLineSLAM.cpp
#
# Description: 
#
===============================================*/

#include <sys/time.h>
#include "systems/system_plane_line_shadow.h"
#include "systems/system_extrinsic_calibration.h"
#include "evaluate/relative_pose_error.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

void loadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, 
				std::vector<ulysses::Transform> &groundtruth, ulysses::Transform calib_sensor_body);
bool flag;
bool flag_calib;

int main(int argc, char *argv[])
{
	flag=true;
	vtkObject::GlobalWarningDisplayOff();
	// argv[1] - sequence path;
	// argv[2] - setting path;
	// argv[3] - settings.yaml;
	// argv[4] - calib_sensor_marker.yaml;
	// argv[5] - calib_pose.yaml;

	std::string seq_path(argv[1]), setting_path(argv[2]);
	std::string setting_file(argv[3]), calib_mocap_file(argv[4]), calib_depth_file(argv[5]);

	// read the setting file;
	cv::FileStorage settings(setting_path+setting_file,cv::FileStorage::READ);
	int debug=(int)settings["debug"];
	int start_frame=(int)settings["start_frame"];
	double time_interval=(double)settings["time_interval"];
	int interval_process=(int)settings["interval_process"];
	int interval_vis=(int)settings["interval_vis"];
	interval_vis*=interval_process;
	flag_calib=(int)settings["flag_calib"];
	int vis_all=(int)settings["vis_all"];

	ulysses::Transform calib_sensor_body;
	cv::Mat rot,trans;
	cv::FileStorage fs;

//	// read kinect-mocap calibration;
//	fs.open(setting_path+calib_mocap_file, cv::FileStorage::READ);
//	fs["rotation"]>>rot;
//	fs["translation"]>>trans;
//	fs.release();
//	cv::cv2eigen(rot,calib_sensor_body.R);
//	cv::cv2eigen(trans,calib_sensor_body.t);
//	std::cout<<"calibration rotation:"<<std::endl<<calib_sensor_body.R<<std::endl;
//	std::cout<<"calibration translation: "<<calib_sensor_body.t.transpose()<<std::endl;

	// read the rgb-depth calibration;
	fs.open(setting_path+calib_depth_file, cv::FileStorage::READ);
	fs["rotation"]>>rot;
	fs["translation"]>>trans;
	fs.release();
	ulysses::Transform T_color_depth;
	cv::cv2eigen(rot,T_color_depth.R);
	cv::cv2eigen(trans,T_color_depth.t);
//	calib_sensor_body=T_color_depth.inv()*calib_sensor_body;
	std::cout<<"T_color_depth: "<<std::endl<<T_color_depth.getMatrix()<<std::endl;

	std::vector<std::string> imgFilenames_depth;
	std::vector<std::string> imgFilenames_rgb;
	std::vector<double> timestamps;
	std::vector<ulysses::Transform> groundtruth;
	std::string file_association=seq_path+"/association.txt";
	loadImages(file_association,imgFilenames_rgb,imgFilenames_depth,timestamps,groundtruth,T_color_depth);

	int num_imgs=imgFilenames_depth.size();
	if(imgFilenames_depth.empty())
	{
		std::cerr<<"No images found."<<std::endl;
		return 1;
	}
	else if(imgFilenames_rgb.size()!=imgFilenames_depth.size())
	{
		std::cerr<<"Different number of rgb and depth images."<<std::endl;
		return 1;
	}

	ulysses::SystemPlaneLine system(seq_path,setting_path+setting_file);
//	ulysses::SystemPlaneLine system2(seq_path,setting_path+setting_file);

//	ulysses::RelativePoseError RPE((double)settings["Evaluate.time_interval"]);
//	ulysses::RelativePoseError RPE2((double)settings["Evaluate.time_interval"]);

	std::vector<double> times_track;
	times_track.resize(num_imgs);

	std::cout<<"Start processing ..."<<std::endl;
	std::cout<<"number of images: "<<num_imgs<<std::endl<<std::endl;
//	if(vis_all) interval_vis=(num_imgs/interval_process-1)*interval_process;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0.1, 0.1, 0.1);
	// vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());

//	boost::shared_ptr<pcl::visualization::PCLPainter2D> fig_error (new pcl::visualization::PCLPainter2D ("Error"));
//	fig_error->setWindowSize (600, 400);
//	fig_error->setPenColor(0,0,0,200);
//	fig_error->addLine(0,200,600,200);

	timeval time_start, time_end;
	double time_used;
	cv::Mat img_rgb,img_depth;
	std::ofstream fp;
	fp.open("frames.txt",std::ios::out);
	double timestamp_pre;
	for(int i=start_frame;i<num_imgs;i+=interval_process)
	{
		if(!flag) break;
		if(i>start_frame && timestamps[i]-timestamp_pre<time_interval) continue;
		timestamp_pre=timestamps[i];
		fp<<std::fixed<<timestamps[i]<<"\t"<<i<<std::endl;
		std::cout<<std::endl<<"***************************************************************************"<<std::endl;
		std::cout<<std::fixed<<timestamps[i]<<"\t"<<i<<std::endl;
//		std::cout<<seq_path+"/"+imgFilenames_depth[i]<<std::endl;
//		std::cout<<seq_path+"/"+imgFilenames_rgb[i]<<std::endl;
		img_depth = cv::imread(seq_path+"/"+imgFilenames_depth[i],CV_LOAD_IMAGE_UNCHANGED);
		img_rgb   = cv::imread(seq_path+"/"+imgFilenames_rgb[i],  CV_LOAD_IMAGE_UNCHANGED);
//		cv::imshow("image",img_rgb);
//		cv::imshow("depth",img_depth);
//		cv::waitKey(0);
		double time_stamp=timestamps[i];
		if(img_depth.empty())
		{
			std::cerr<<"Failed to load image at "<<seq_path<<"/"<<imgFilenames_rgb[i]<<std::endl;
			return 1;
		}

		system.trackCamera(img_rgb,img_depth,time_stamp,vis,groundtruth[i],true);
//		fig_error->setPenColor(255,0,0,200);
//		RPE.evaluateEachFrame(time_stamp,groundtruth[i],system.getCurrentCamera(),fig_error);
//		fig_error->spinOnce();

//		system.showScans(img_rgb,img_depth,time_stamp,vis,groundtruth[i],flag_calib);

//		std::cout<<groundtruth[i].getMatrix()<<std::endl;
		if(i%interval_vis==0 && !vis_all)
		{
			vis->spin();
			vis->removeAllPointClouds();
//			vis->removeAllShapes();
		}
		
//		char ch;
//		std::cout<<"Press enter to continue...\n"<<std::endl;
//		ch=std::cin.get();

//		if(debug==0)
//		{
//			system2.trackCamera(img_rgb,img_depth,time_stamp,vis,groundtruth[i]);
//				fig_error->setPenColor(0,0,255,200);
//				RPE2.evaluateEachFrame(time_stamp,groundtruth[i],system2.getCurrentCamera(),fig_error);
//				fig_error->spinOnce();
//		}
		
	}
	vis->spin();

//	system.saveTraj(std::string("traj.txt"));
//	RPE.saveErrors(std::string("errors.txt"));

//	if(debug==0)
//	{
//		system2.saveTraj(std::string("traj2.txt"));
//		RPE2.saveErrors(std::string("errors2.txt"));
//	}

	system.shutDown();
//	system2.shutDown();
	fp.close();

	return 0;
}

void loadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, 
				std::vector<ulysses::Transform> &groundtruth, ulysses::Transform calib_sensor_body)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            ss >> t;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);

			double tx,ty,tz,qx,qy,qz,qw;
			ss>>t;
			ss>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			Eigen::Quaterniond quat(qw,qx,qy,qz);
			Eigen::Vector3d trans(tx,ty,tz);
			ulysses::Transform Tg(quat,trans);
//			if(flag_calib) Tg=Tg*calib_sensor_body.inv();
			Tg=Tg*calib_sensor_body;
			groundtruth.push_back(Tg);
        }
    }
//	std::ofstream fp;
//	fp.open("axis_rotation.txt",std::ios::out);
	ulysses::Transform T0=groundtruth[0].inv();
	for(int i=0;i<groundtruth.size();i++)
	{
		groundtruth[i]=T0*groundtruth[i];
//		fp<<std::fixed<<vTimestamps[i]<<" "<<groundtruth[i].t.transpose()<<" "<<groundtruth[i].Quaternion().transpose()<<std::endl;
	}
//	fp.close();
}

void saveTimesTrack(const std::vector<double> &times)
{
	std::ofstream fp;
	fp.open("times.txt",std::ios::out);
	for(int i=0;i<times.size();i++)
	{
		fp<<times[i]<<std::endl;
	}
	fp.close();
}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "q" && event.keyDown ())
	{
	   viewer->close();
	}
	if (event.getKeySym () == "c" && event.keyDown ())
	{
		flag=false;
	}
}

