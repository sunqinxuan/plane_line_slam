/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-08-05 08:54
#
# Filename:		system_extrinsic_calibration.cpp
#
# Description: 
#
************************************************/
#include "systems/system_extrinsic_calibration.h"

namespace ulysses 
{
	Transform SystemExtrinsicCalibration::calibrate(const std::string &mode)
	{
		if(mode=="chessLocalize")
		{
			readFile_chess();
			chessLocalization();
			outputPoses();
		}
		else if(mode=="CalibrateStatic")
		{
			Transform T_tag_marker2;
			T_tag_marker2.t(0)=-0.03;
			T_tag_marker2.t(1)=-0.03;
			T_tag_marker2.t(2)=-0.00978;

			std::string path=seq_path+"camera/";
			DIR* dir=opendir(path.c_str());
			dirent* ptr=NULL;
			std::vector<std::string> indices;
			while((ptr=readdir(dir))!=NULL)
			{
				if(ptr->d_name[0]!='.')
				{
//					std::string cur_idx=std::string(ptr->d_name);
//					std::string name=path+std::string(ptr->d_name);
//					std::cout<<name<<std::endl;
					indices.push_back(std::string(ptr->d_name));
				}
			}
			sort(indices.begin(),indices.end());
			for(int i=0;i<indices.size();i++)
			{
				CalibMeasure calib_measure(T_tag_marker2,indices[i]);
				readFile_static(calib_measure);
//				chessLocalization(calib_measure);
				calib_measures.push_back(calib_measure);
//				Transform calib=calib_measure.Ts_tag_sensor[0].pose.inv()
//								*calib_measure.T_tag_marker2
//								*calib_measure.Ts_mocap_marker2[0].pose.inv()
//								*calib_measure.Ts_mocap_marker[0].pose;
//				std::cout<<"calibration matrix: "<<std::endl<<calib.getMatrix()<<std::endl<<std::endl;
//				calib_matrices.push_back(calib);
			}
			calibrate_static();
			storeCalibration();
		}
		else if(mode=="CalibrateDynamic")
		{
			motion_sensor.clear();
			motion_marker.clear();

//			readFile("association0712.txt");
//			generateMotions();
//			readFile("association0716.txt");
//			generateMotions();
//			readFile("association0720.txt");
//			generateMotions();

			readFile("association.txt");
			generateMotions();

			computeRotation();
			computeTranslation();

			storeCalibration();

			for(int i=0;i<T_mocap_marker.size();i++)
			{
				T_mocap_marker[i].pose=T_mocap_marker[i].pose*calib_sensor_marker.inv(); // T_WC
			}

			ATE();

			std::ofstream fp;
			fp.open("rotateAxis.txt",std::ios::out);
			for(int i=0;i<motion_sensor.size();i++)
			{
				Eigen::AngleAxisd angle_s(motion_sensor[i].R);
				Eigen::AngleAxisd angle_m(motion_marker[i].R);
				double weight=motion_weight[i];
				Eigen::Matrix<double,1,1> cos_delta=angle_s.axis().transpose()*calib_sensor_marker.R*angle_m.axis();
				double delta=acos(cos_delta(0,0))*180.0/M_PI;
				fp<<fabs(angle_s.angle()-angle_m.angle())<<" "<<weight<<" "<<delta<<std::endl;
			}
			fp.close();

		}

		return calib_sensor_marker;
	}

	void SystemExtrinsicCalibration::storeCalibration()
	{
//		cv::Mat_<double> rotation = cv::Mat_<double>::eye(3,3);
//		cv::Mat_<double> translation = cv::Mat_<double>::eye(3,1);
		cv::Mat rotation, translation;
		cv::eigen2cv(calib_sensor_marker.R,rotation);
		cv::eigen2cv(calib_sensor_marker.t,translation);

		cv::FileStorage fs;
		fs.open(seq_path+"calib_sensor_marker.yaml", cv::FileStorage::WRITE);
		fs << "rotation" << rotation;
		fs << "translation" << translation;
		fs.release();

//		Eigen::Matrix3d R;
//		Eigen::Vector3d t;
//		cv::Mat rot,trans;
//		fs.open(seq_path+"calib_sensor_marker.yaml", cv::FileStorage::READ);
//		fs["rotation"]>>rot;
//		fs["translation"]>>trans;
//		cv::cv2eigen(rot,R);
//		cv::cv2eigen(trans,t);
//		std::cout<<"stored rotation:"<<std::endl<<R<<std::endl;
//		std::cout<<"stored translation: "<<trans<<std::endl;
	}

	void SystemExtrinsicCalibration::readFile_static(CalibMeasure &calib_measure)
	{
//		// T_tag_sensor 
//		// load the chess board image file names;
//		std::string file_name=seq_path+"camera/"+calib_measure.cur_idx+"/rgb_hd.txt";
//		std::cout<<file_name<<std::endl;
//		std::ifstream fp;
//		fp.open(file_name.c_str(),std::ios::in);
//		while(!fp.eof())
//		{
//			std::string s;
//			getline(fp,s);
//			if(!s.empty())
//			{
//				std::stringstream ss;
//				ss << s;
//				// load rgb filenames;
//				double time_stamp;
//				std::string filename;
//				ss>>time_stamp>>filename;
//				calib_measure.Ts_tag_sensor.push_back(Pose(time_stamp,seq_path+"camera/"+calib_measure.cur_idx+"/"+filename));
//			}
//		}
//		fp.close();

		// T_tag_sensor 
		std::string file_name=seq_path+calib_measure.cur_idx+".txt";
		std::cout<<file_name<<std::endl;
		std::ifstream fp;
		fp.open(file_name.c_str(),std::ios::in);
		while(!fp.eof())
		{
			std::string s;
			getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss;
				ss << s;
				double tx,ty,tz,qx,qy,qz,qw;
				ss>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat1(qw,qx,qy,qz);
				Eigen::Vector3d trans1(tx,ty,tz);
				ulysses::Transform T(quat1,trans1);
				calib_measure.Ts_tag_sensor.push_back(Pose(T));
			}
		}
		fp.close();

		// T_mocap_marker 
		// markers attached to the camera;
		file_name=seq_path+"mocap/markers_camera_"+calib_measure.cur_idx+".txt";
		std::cout<<file_name<<std::endl;
		fp.open(file_name.c_str(),std::ios::in);
		while(!fp.eof())
		{
			std::string s;
			getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss;
				ss << s;
				Transform trans;
				ss>>trans.t(0)>>trans.t(1)>>trans.t(2)
				  >>trans.R(0,0)>>trans.R(1,0)>>trans.R(2,0)
				  >>trans.R(0,1)>>trans.R(1,1)>>trans.R(2,1)
				  >>trans.R(0,2)>>trans.R(1,2)>>trans.R(2,2);
				trans.t=trans.t/1000.0;
				calib_measure.Ts_mocap_marker.push_back(Pose(trans));
			}
		}
		std::cout<<calib_measure.Ts_mocap_marker[0].pose.getMatrix()<<std::endl;
		fp.close();

		// T_mocap_marker2 
		// markers attached to the chess board;
		file_name=seq_path+"mocap/markers_board_"+calib_measure.cur_idx+".txt";
		std::cout<<file_name<<std::endl;
		fp.open(file_name.c_str(),std::ios::in);
		while(!fp.eof())
		{
			std::string s;
			getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss;
				ss << s;
				Eigen::Vector3d p1,p2,p3,p4;
				ss>>p1(0)>>p1(1)>>p1(2)
				  >>p2(0)>>p2(1)>>p2(2)
				  >>p3(0)>>p3(1)>>p3(2)
				  >>p4(0)>>p4(1)>>p4(2);
				Transform trans;
				trans.t=p2/1000.0;
				trans.R.col(0)=p1-p2;
				trans.R.col(0).normalize();
				trans.R.col(1)=p3-p2;
				trans.R.col(1).normalize();
				trans.R.col(2)=trans.R.col(0).cross(trans.R.col(1));
				trans.R.col(2).normalize();
				calib_measure.Ts_mocap_marker2.push_back(Pose(trans));
//				T_mocap_marker2.push_back(Pose(p1,p2,p3,p4));
			}
		}
		std::cout<<calib_measure.Ts_mocap_marker2[0].pose.getMatrix()<<std::endl;
		fp.close();

	}


	void SystemExtrinsicCalibration::readFile_chess()
	{
		std::string association_file=seq_path+"association.txt";
		std::ifstream fp;
		fp.open(association_file.c_str(),std::ios::in);

		double pre_time=0;
		while(!fp.eof())
		{
			std::string s;
			getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss;
				ss << s;

				// load rgb filenames;
				double time_stamp;
				std::string filename;
				ss>>time_stamp>>filename;
//				if(time_stamp-pre_time>3.0)
					T_tag_sensor.push_back(Pose(time_stamp,filename));

				// load groundtruth;
				double tx,ty,tz,qx,qy,qz,qw;
				ss>>time_stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat1(qw,qx,qy,qz);
				Eigen::Vector3d trans1(tx,ty,tz);
				ulysses::Transform Twm(quat1,trans1);
//				if(time_stamp-pre_time>3.0)
					T_mocap_marker.push_back(Pose(time_stamp,Twm));

				pre_time=time_stamp;
			}
		}

		fp.close();
	}

	void SystemExtrinsicCalibration::readFile(const std::string &association_file)
	{
		std::string full_path=seq_path+association_file;
		std::ifstream fp;
		fp.open(full_path.c_str(),std::ios::in);

		T_tag_sensor.clear();
		T_mocap_marker.clear();

		double pre_time=0;
		while(!fp.eof())
		{
			std::string s;
			getline(fp,s);
			if(!s.empty())
			{
				// load calibration
				std::stringstream ss;
				ss << s;
				double time_stamp,tx,ty,tz,qx,qy,qz,qw;
				ss>>time_stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat(qw,qx,qy,qz);
				Eigen::Vector3d trans(tx,ty,tz);
				ulysses::Transform Tca(quat,trans);
//				Tca=Tca.inv();
//				if(time_stamp-pre_time>3.0)
					T_tag_sensor.push_back(Pose(time_stamp,Tca));

				// load groundtruth
				ss>>time_stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat1(qw,qx,qy,qz);
				Eigen::Vector3d trans1(tx,ty,tz);
				ulysses::Transform Twm(quat1,trans1);
//				if(time_stamp-pre_time>3.0)
					T_mocap_marker.push_back(Pose(time_stamp,Twm));

//				// load markers
//				ss>>time_stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
//				Eigen::Quaterniond quat2(qw,qx,qy,qz);
//				Eigen::Vector3d trans2(tx,ty,tz);
//				ulysses::Transform Twm2(quat2,trans2);
////				if(time_stamp-pre_time>3.0)
//					T_mocap_marker2.push_back(Pose(time_stamp,Twm2));
				
				pre_time=time_stamp;
			}
		}

		fp.close();
	}

	void SystemExtrinsicCalibration::calibrate_static()
	{
		calib_sensor_marker.t.setZero();
		calib_matrices.resize(calib_measures.size());
		for(int i=0;i<calib_measures.size();i++)
		{
			calib_measures[i].T_tag_sensor=averageTransform(calib_measures[i].Ts_tag_sensor);
			calib_measures[i].T_mocap_marker=averageTransform(calib_measures[i].Ts_mocap_marker);
			calib_measures[i].T_mocap_marker2=averageTransform(calib_measures[i].Ts_mocap_marker2);
			calib_matrices[i].pose= calib_measures[i].T_tag_sensor.inv()
							       *calib_measures[i].T_tag_marker2
							       *calib_measures[i].T_mocap_marker2.inv()
							       *calib_measures[i].T_mocap_marker;
		}
		calib_sensor_marker=averageTransform(calib_matrices);
		std::cout<<std::endl<<"calib_sensor_marker:"<<std::endl<<calib_sensor_marker.getMatrix()<<std::endl;
	}

	Transform SystemExtrinsicCalibration::averageTransform(const std::vector<Pose> &transforms)
	{
		// compute the average translation;
		Transform avg_trans;
		avg_trans.t.setZero();
		for(int i=0;i<transforms.size();i++)
		{
			avg_trans.t+=transforms[i].pose.t;
		}
		avg_trans.t/=transforms.size();

		// compute the average rotation;
		avg_trans.R.setIdentity();
		Eigen::Matrix3d R0=transforms[0].pose.R;
		Eigen::Vector3d w=Eigen::Vector3d::Zero();
		for(int i=0;i<transforms.size();i++)
		{
			Eigen::Matrix3d w_hat=R0*transforms[i].pose.R.transpose();
			w_hat=w_hat.log();
			w+=Transform::skew_sym_inv(w_hat);
		}
		w/=(transforms.size()-1);
		Eigen::Matrix3d w_hat=Transform::skew_sym(w);
		avg_trans.R=w_hat.exp()*R0;

		return avg_trans;
	}

	void SystemExtrinsicCalibration::simulateCameraPoses()
	{
		Eigen::Quaterniond q(0.876602,0.461966,0.11442,0.071161);
		Eigen::Vector3d t(-0.430554,-0.207695,0.411948);
		Transform T_CM(q,t);
//		Transform T_CM;
		std::cout<<std::endl<<"T_CM:"<<std::endl<<T_CM.getMatrix()<<std::endl;

		Eigen::Quaterniond qq(-0.44491,-0.508,0.622692,0.39528);
		Eigen::Vector3d tt(1.08335,0.991459,-1.04211);
		Transform T_AW(qq,tt);
		std::cout<<std::endl<<"T_AW:"<<std::endl<<T_AW.getMatrix()<<std::endl;

		T_mocap_marker2.resize(T_tag_sensor.size());
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			T_mocap_marker2[i].pose=T_AW*T_mocap_marker[i].pose;
//			T_mocap_marker2[i].pose=T_AW*T_mocap_marker[i].pose*T_CM.inv();
//			T_mocap_marker2[i].pose=T_AW.inv()*T_tag_sensor[i].pose*T_CM;
		}
	}

	void SystemExtrinsicCalibration::outputPoses()
	{
		std::ofstream fp;

		fp.open("outputCalib.txt",std::ios::out);
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			if(T_tag_sensor[i].valid)
			{
				Transform Tca=T_tag_sensor[i].pose;
				fp<<std::fixed<<T_tag_sensor[i].time_stamp<<" "
				  <<Tca.t.transpose()<<" "
				  <<Tca.Quaternion().transpose()<<std::endl;
			}
		}
		fp.close();

		fp.open("outputGT.txt",std::ios::out);
		for(int i=0;i<T_mocap_marker.size();i++)
		{
			if(T_tag_sensor[i].valid)
			{
				fp<<std::fixed<<T_mocap_marker[i].time_stamp<<" "
				  <<T_mocap_marker[i].pose.t.transpose()<<" "
				  <<T_mocap_marker[i].pose.Quaternion().transpose()<<std::endl;
			}
		}
		fp.close();


//		fp.open("motionSensor.txt",std::ios::out);
//		for(int i=0;i<motion_sensor.size();i++)
//		{
//			Transform Tca=motion_sensor[i];
//			fp<<Tca.t.transpose()<<" "
//			  <<Tca.Quaternion().transpose()<<std::endl;
//		}
//		fp.close();
//
//		fp.open("motionMarker.txt",std::ios::out);
//		for(int i=0;i<motion_marker.size();i++)
//		{
//			fp<<motion_marker[i].t.transpose()<<" "
//			  <<motion_marker[i].Quaternion().transpose()<<std::endl;
//		}
//		fp.close();
	}

	void SystemExtrinsicCalibration::generateMotions()
	{
		cv::Mat rot,trans;
		cv::FileStorage fs;
		fs.open(seq_path+"calib_sensor_marker.yaml", cv::FileStorage::READ);
		fs["rotation"]>>rot;
		fs["translation"]>>trans;
		fs.release();
		cv::cv2eigen(rot,calib_sensor_marker.R);
		cv::cv2eigen(trans,calib_sensor_marker.t);

		std::ofstream fp;
		fp.open("generateMotions.txt",std::ios::out);
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			for(int j=i+1;j<T_tag_sensor.size();j++)
			{
//				if(fabs(T_tag_sensor[i].time_stamp-T_tag_sensor[j].time_stamp)<3.0) continue;
				Transform motion_s=T_tag_sensor[i].pose.inv()*T_tag_sensor[j].pose;
				Eigen::AngleAxisd angle_s(motion_s.R);
				Transform motion_m=T_mocap_marker[i].pose.inv()*T_mocap_marker[j].pose;
				Eigen::AngleAxisd angle_m(motion_m.R);

				if(angle_s.angle()<0.1 || angle_m.angle()<0.1) continue;
//				double delta=fabs(angle_s.angle()-angle_m.angle());
//				double weight=1;
//				if(delta>0.001) weight=0.001/delta;

				Eigen::Matrix<double,1,1> cos_delta=angle_s.axis().transpose()*calib_sensor_marker.R*angle_m.axis();
				double delta=acos(cos_delta(0,0))*180.0/M_PI;

				Eigen::Matrix3d R=calib_sensor_marker.R.transpose()*motion_s.R.transpose()*calib_sensor_marker.R*motion_m.R;
				double r=acos((R.trace()-1.0)/2.0);
				double weight=1;
				if(r>0.001) weight=0.001/r;

				fp<<fabs(angle_s.angle()-angle_m.angle())<<" "<<r<<" "<<weight<<std::endl;

//				if(weight<0.9) continue;

//				fp<<angle_s.angle()<<" "<<angle_m.angle()<<" "<<fabs(angle_s.angle()-angle_m.angle())<<" "<<weight<<std::endl;
//				fp<<fabs(angle_s.angle()-angle_m.angle())<<" "<<weight<<" "<<angle_s.axis().transpose()<<" "<<angle_m.axis().transpose()<<std::endl;

				motion_sensor.push_back(motion_s);
				motion_marker.push_back(motion_m);
				motion_weight.push_back(weight);
			}
		}
		fp.close();
	}

	void SystemExtrinsicCalibration::computeRotation()
	{
		std::ofstream fp;
		fp.open("computeRotation.txt",std::ios::out);

		const int N=motion_sensor.size();
		Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N*4,4);
		int count=0;
		for(int i=0;i<N;i++)
		{
			// q=[x,y,z,w]=[v,w];
			Eigen::Quaterniond q_sensor=motion_sensor[i].Quat();
			Eigen::Vector3d v=q_sensor.vec();
			double w=q_sensor.w();
			Eigen::Matrix4d Q_sensor;
			Q_sensor.setZero();
			Q_sensor.block<1,3>(0,1)=-v.transpose();
			Q_sensor.block<3,1>(1,0)=v;
			Q_sensor.block<3,3>(1,1)=Transform::skew_sym(v);
			Q_sensor+=w*Eigen::Matrix4d::Identity();

			Eigen::Quaterniond q_body=motion_marker[i].Quat();
			v=q_body.vec();
			w=q_body.w();
			Eigen::Matrix4d Q_body_bar;
			Q_body_bar.setZero();
			Q_body_bar.block<1,3>(0,1)=-v.transpose();
			Q_body_bar.block<3,1>(1,0)=v;
			Q_body_bar.block<3,3>(1,1)=-Transform::skew_sym(v);
			Q_body_bar+=w*Eigen::Matrix4d::Identity();

			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q_body_bar-Q_sensor, Eigen::ComputeFullU | Eigen::ComputeFullV);
//			if(svd.singularValues()(3)>0.005) continue;
//			std::cout<<std::endl<<"ratio: "<<svd.singularValues()(3)/svd.singularValues()(0)<<std::endl;
//			if(svd.singularValues()(3)/svd.singularValues()(0)>0.01) continue;

//			Eigen::Vector4d q=svd.matrixV().col(3);
//			Eigen::Quaterniond q_xyz_w(q);
			std::cout<<std::endl<<i<<" computeSVD"<<std::endl;
			std::cout<<"q_sensor: "<<q_sensor.w()<<" "<<q_sensor.vec().transpose()<<std::endl;
			std::cout<<"q_body: "<<q_body.w()<<" "<<q_body.vec().transpose()<<std::endl;
//			std::cout<<"angularDistance: "<<q_sensor.angularDistance(q_body)<<std::endl;
			std::cout<<"angularDistance: "<<fabs(q_sensor.w()-q_body.w())<<std::endl;
			std::cout<<"Q singularValues "<<svd.singularValues().transpose()<<std::endl;
			std::cout<<"Q V: "<<std::endl<<svd.matrixV()<<std::endl;
//			std::cout<<"q_xyz_w="<<q.transpose()<<std::endl;

//			if(fabs(q_sensor.w()-q_body.w())>0.001) continue;
			Q.block<4,4>(i*4,0)=(Q_body_bar-Q_sensor)*motion_weight[i];
			count++;

		}

		std::cout<<"selected data: "<<count<<std::endl;

		Eigen::Matrix4d QTQ=Q.transpose()*Q;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es;
		es.compute(QTQ);
		Eigen::Vector4d eig_vals=es.eigenvalues();
		Eigen::Matrix4d eig_vecs=es.eigenvectors();

		std::cout<<std::endl<<"computeRotation"<<std::endl;
		std::cout<<"QTQ eigen values "<<eig_vals.transpose()<<std::endl;
		std::cout<<"QTQ eigen vectors "<<std::endl<<eig_vecs<<std::endl;

		double min=DBL_MAX;
		int idx;
		for(int i=0;i<4;i++)
		{
			if(eig_vals(i)<min)
			{
				min=eig_vals(i);
				idx=i;
			}
		}
		Eigen::Vector4d q_w_xyz=eig_vecs.block<4,1>(0,idx);
		std::cout<<"q_w_xyz="<<q_w_xyz.transpose()<<std::endl;

		Eigen::Quaterniond quat(q_w_xyz(0),q_w_xyz(1),q_w_xyz(2),q_w_xyz(3));
		calib_sensor_marker.R=quat.toRotationMatrix();
		std::cout<<"calibration rotation:"<<std::endl<<calib_sensor_marker.R<<std::endl;
		
		fp.close();
	}

	void SystemExtrinsicCalibration::computeTranslation()
	{
		const int N=motion_sensor.size();
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N*3,3);
		Eigen::VectorXd b = Eigen::VectorXd::Zero(N*3);
		Eigen::Matrix3d R_calib=calib_sensor_marker.R;
		for(int i=0;i<N;i++)
		{
			Eigen::Matrix3d R_sensor=motion_sensor[i].R;
			Eigen::Vector3d t_sensor=motion_sensor[i].t;
			Eigen::Vector3d t_body=motion_marker[i].t;
			
			A.block<3,3>(i*3,0)=motion_weight[i]*(R_sensor-Eigen::Matrix3d::Identity());
			b.block<3,1>(i*3,0)=motion_weight[i]*(R_calib*t_body-t_sensor);
		}

		Eigen::Matrix3d ATA=A.transpose()*A;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
		es.compute(ATA);
		Eigen::Vector3d eig_vals=es.eigenvalues();
		Eigen::Matrix3d eig_vecs=es.eigenvectors();

		std::cout<<std::endl<<"computeTranslation"<<std::endl;
		std::cout<<"ATA eigen values "<<eig_vals.transpose()<<std::endl;
		std::cout<<"ATA eigen vectors "<<std::endl<<eig_vecs<<std::endl;

		Eigen::Matrix3d eig_vals_mat=eig_vals.asDiagonal();
		for(int i=0;i<3;i++)
		{
			eig_vals_mat(i,i)=1.0/eig_vals_mat(i,i);
		}
		Eigen::Matrix3d ATA_inv=eig_vecs*eig_vals_mat*eig_vecs.transpose();
		std::cout<<"ATA_inv="<<std::endl<<ATA_inv<<std::endl;
		std::cout<<"ATA*ATA_inv="<<std::endl<<ATA*ATA_inv<<std::endl;

		calib_sensor_marker.t=ATA_inv*A.transpose()*b;
		std::cout<<"calibration translation "<<calib_sensor_marker.t.transpose()<<std::endl;

		Eigen::VectorXd error=b;
		error=A*calib_sensor_marker.t-b;
//		std::cout<<"error: "<<error.transpose()<<std::endl;
		Eigen::Matrix<double,1,1> errorTerror=error.transpose()*error;
		std::cout<<"error norm: "<<sqrt(errorTerror(0,0)/N)<<std::endl;
	}

	void SystemExtrinsicCalibration::ATE()
	{
		// compute R,t such that t_tag_sensor=R*t_mocap_marker+t;
		Eigen::Vector3d centroid_sensor=Eigen::Vector3d::Zero();
		Eigen::Vector3d centroid_marker=Eigen::Vector3d::Zero();
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			centroid_sensor+=T_tag_sensor[i].pose.t;
			centroid_marker+=T_mocap_marker[i].pose.t;
		}
		centroid_sensor/=T_tag_sensor.size();
		centroid_marker/=T_mocap_marker.size();

		std::vector<Eigen::Vector3d> p_sensor, p_marker;
		Eigen::Matrix3d H=Eigen::Matrix3d::Zero();
		p_sensor.resize(T_tag_sensor.size());
		p_marker.resize(T_mocap_marker.size());
		// p_sensor[i]=R*p_marker[i];
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			p_sensor[i]=T_tag_sensor[i].pose.t-centroid_sensor;
			p_marker[i]=T_mocap_marker[i].pose.t-centroid_marker;
//			std::cout<<i<<"th pair: "<<p_sensor[i].transpose()<<"; "<<p_marker[i].transpose()<<std::endl;
			H+=p_marker[i]*p_sensor[i].transpose();
		}
		std::cout<<"H:"<<std::endl<<H<<std::endl;
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d H_svd_U=svd.matrixU();
		Eigen::Matrix3d H_svd_V=svd.matrixV();
		Eigen::Vector3d H_singularValues=svd.singularValues();
		std::cout<<"H_singularValues: "<<H_singularValues.transpose()<<std::endl;

		Eigen::Matrix3d R=H_svd_V*H_svd_U.transpose();
//		std::cout<<"rotation aligning trajs "<<std::endl<<R<<std::endl;
		std::cout<<"R.det()="<<R.determinant()<<std::endl;
		if(R.determinant()<0) 
		{
			std::cerr<<"aligning failure !"<<std::endl;
			H_svd_V.col(2)=-H_svd_V.col(2);
			R=H_svd_V*H_svd_U.transpose();
			std::cout<<"R.det()="<<R.determinant()<<std::endl;
		}

		Eigen::Vector3d t=centroid_sensor-R*centroid_marker;
//		std::cout<<"translation aligning trajs "<<std::endl<<t.transpose()<<std::endl;
		
		T_align.R=R;
		T_align.t=t;
		std::cout<<"aligning trajs:"<<std::endl<<T_align.getMatrix()<<std::endl;
		std::cout<<"Quaternion:"<<T_align.Quat().w()<<","<<T_align.Quat().vec().transpose()<<std::endl;
		std::cout<<"translation:"<<T_align.t.transpose()<<std::endl;

		double ate=0;
		for(int i=0;i<T_mocap_marker.size();i++)
		{
			T_mocap_marker[i].pose=T_align*T_mocap_marker[i].pose;
//			T_mocap_marker[i].pose.t=T_align.R*T_mocap_marker[i].pose.t+T_align.t;
			Eigen::Vector3d delta=T_mocap_marker[i].pose.t-T_tag_sensor[i].pose.t;
			ate+=delta.transpose()*delta;
		}
		ate=sqrt(ate);
		ate/=T_mocap_marker.size();
		std::cout<<"ate="<<ate<<std::endl;
	}

	void SystemExtrinsicCalibration::RPE()
	{
		double rot_error=0, trans_error=0;
		for(int i=23;i<T_tag_sensor.size();i++)
		{
			Transform delta_sensor=T_tag_sensor[i].pose*T_tag_sensor[0].pose.inv();
			Transform delta_marker=T_mocap_marker[i].pose*T_mocap_marker[0].pose.inv();
			Transform delta=delta_sensor*delta_marker.inv();
			double rot=acos((delta.R.trace()-1.0)/2.0);
			double trans=delta.t.norm();
			std::cout<<i<<"th rot="<<rot<<", trans="<<trans<<std::endl;
			rot_error+=rot;
			trans_error+=trans;
		}
		rot_error/=(T_tag_sensor.size()-23);
		trans_error/=(T_tag_sensor.size()-23);
		
		std::cout<<"rotational error="<<rot_error<<std::endl;
		std::cout<<"translational error="<<trans_error<<std::endl;
	}

	void SystemExtrinsicCalibration::chessLocalization()
	{
		cv::Size boardDims=cv::Size(5,7);
		double boardSize=0.03;
		std::vector<cv::Point3f> board;
		std::vector<std::vector<cv::Point3f> > pointsBoard;
		std::vector<std::vector<cv::Point2f> > pointsColor;
		
		board.resize(boardDims.width * boardDims.height);
		for (int r = 0, i = 0; r < (int)boardDims.height; ++r)
		{
			for (int c = 0; c < (int)boardDims.width; ++c, ++i)
			{
				board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
			}
		}

		pointsColor.resize(T_tag_sensor.size());
		pointsBoard.resize(T_tag_sensor.size(),board);

		cv::Mat color,colorDisp;
		const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			std::cout<<"filename: "<<seq_path+T_tag_sensor[i].filename<<std::endl;
			cv::cvtColor(cv::imread(seq_path+T_tag_sensor[i].filename,CV_LOAD_IMAGE_UNCHANGED),color,CV_BGR2GRAY);
			bool foundColor=cv::findChessboardCorners(color, boardDims, pointsColor[i]);//, cv::CALIB_CB_FAST_CHECK);
//			cv::imshow("color",color);
//			cv::waitKey(0);
			if(foundColor) cv::cornerSubPix(color,pointsColor[i],cv::Size(11,11),cv::Size(-1,-1),termCriteria);
			std::cout<<"findChessboardCorners: "<<i<<std::endl;
			if(pointsColor[i].size()==0)
			{
				T_tag_sensor[i].valid=false;
				continue;
			}
			std::cout<<"("<<pointsColor[i][0].x<<", "<<pointsColor[i][0].y<<")"<<std::endl;
			std::cout<<"("<<pointsColor[i][34].x<<", "<<pointsColor[i][34].y<<")"<<std::endl;
			if(pointsColor[i][0].x<pointsColor[i][34].x && pointsColor[i][0].y>pointsColor[i][34].y)
			{
				std::reverse(pointsColor[i].begin(),pointsColor[i].end());
			}

			cv::cvtColor(color,colorDisp,CV_GRAY2BGR);
			cv::drawChessboardCorners(colorDisp,boardDims,pointsColor[i],foundColor);
			cv::imshow("color",colorDisp);
			cv::waitKey(10);
		}

		cv::Size size=cv::Size(1920,1080);
		cv::Mat cameraMatrix, distortion;

		cv::FileStorage fs_intrinsic;
		fs_intrinsic.open(setting_path+"calib_color.yaml",cv::FileStorage::READ);
		fs_intrinsic["cameraMatrix"] >> cameraMatrix;
		fs_intrinsic["distortionCoefficients"] >> distortion;
		fs_intrinsic.release();
//		calibrateIntrinsics(sizeColor, pointsBoard, pointsColor, cameraMatrixColor, distortionColor, rotationColor, projectionColor, rvecsColor, tvecsColor);


//		const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
//		double error;
//
//		std::cout<<"calibrating intrinsics..."<<std::endl;
//		error = cv::calibrateCamera(pointsBoard, pointsColor, size, cameraMatrix, distortion, rvecs, tvecs, 0, termCriteria);
//		std::cout<<"re-projection error: " << error << std::endl;

		std::cout<<"Camera Matrix:" << std::endl << cameraMatrix<<std::endl;
		std::cout<<"Distortion Coeeficients:" << std::endl << distortion << std::endl;
//		std::vector<cv::Mat> R_matrices;
//		R_matrices.resize(rvecs.size());
		cv::Mat rvec,tvec;
//		std::string file_name=seq_path+calib_measure.cur_idx+".txt";
//		std::ofstream fp;
//		fp.open(file_name,std::ios::out);
		for(int i=0;i<T_tag_sensor.size();i++)
		{
			std::cout<<"pointsColor["<<i<<"].size()="<<pointsColor[i].size()<<std::endl;
			if(T_tag_sensor[i].valid==false) continue;
			cv::solvePnP(pointsBoard[i],pointsColor[i],cameraMatrix,distortion,rvec,tvec);
//			std::cout<<std::endl<<"rotation:"<<rvecs[i].t()<<std::endl;
//			std::cout<<"translation:"<<tvecs[i].t()<<std::endl;
//			cv::Rodrigues(rvecs[i],R_matrices[i]);
//			std::cout<<"rotation matrix:"<<std::endl<<R_matrices[i]<<std::endl;

			T_tag_sensor[i].pose=Mat2Trans(rvec,tvec);
			std::cout<<"transform:"<<std::endl<<T_tag_sensor[i].pose.getMatrix()<<std::endl;
			std::cout<<"quat:"<<T_tag_sensor[i].pose.Quat().w()<<" "<<T_tag_sensor[i].pose.Quat().vec().transpose()<<std::endl;
//			fp<<T_tag_sensor[i].pose.t.transpose()<<" "<<T_tag_sensor[i].pose.Quaternion().transpose()<<std::endl;
		}
//		fp.close();

	}

	void SystemExtrinsicCalibration::chessLocalization(CalibMeasure &calib_measure)
	{
		cv::Size boardDims=cv::Size(5,7);
		double boardSize=0.03;
		std::vector<cv::Point3f> board;
		std::vector<std::vector<cv::Point3f> > pointsBoard;
		std::vector<std::vector<cv::Point2f> > pointsColor;
		
		board.resize(boardDims.width * boardDims.height);
		for (int r = 0, i = 0; r < (int)boardDims.height; ++r)
		{
			for (int c = 0; c < (int)boardDims.width; ++c, ++i)
			{
				board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
			}
		}

		pointsColor.resize(calib_measure.Ts_tag_sensor.size());
		pointsBoard.resize(calib_measure.Ts_tag_sensor.size(),board);

		cv::Mat color,colorDisp;
		const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);
		for(int i=0;i<calib_measure.Ts_tag_sensor.size();i++)
		{
			std::cout<<"filename: "<<calib_measure.Ts_tag_sensor[i].filename<<std::endl;
			cv::cvtColor(cv::imread(calib_measure.Ts_tag_sensor[i].filename,CV_LOAD_IMAGE_UNCHANGED),color,CV_BGR2GRAY);
			bool foundColor=cv::findChessboardCorners(color, boardDims, pointsColor[i]);//, cv::CALIB_CB_FAST_CHECK);
//			cv::imshow("color",color);
//			cv::waitKey(0);
			if(foundColor) cv::cornerSubPix(color,pointsColor[i],cv::Size(11,11),cv::Size(-1,-1),termCriteria);
			std::cout<<"findChessboardCorners: "<<i<<std::endl;

			cv::cvtColor(color,colorDisp,CV_GRAY2BGR);
			cv::drawChessboardCorners(colorDisp,boardDims,pointsColor[i],foundColor);
			cv::imshow("color",colorDisp);
			cv::waitKey(10);
		}

		cv::Size size=cv::Size(1920,1080);
		cv::Mat cameraMatrix, distortion;

		cv::FileStorage fs_intrinsic;
		fs_intrinsic.open(setting_path+"calib_color.yaml",cv::FileStorage::READ);
		fs_intrinsic["cameraMatrix"] >> cameraMatrix;
		fs_intrinsic["distortionCoefficients"] >> distortion;
		fs_intrinsic.release();
//		calibrateIntrinsics(sizeColor, pointsBoard, pointsColor, cameraMatrixColor, distortionColor, rotationColor, projectionColor, rvecsColor, tvecsColor);


//		const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
//		double error;
//
//		std::cout<<"calibrating intrinsics..."<<std::endl;
//		error = cv::calibrateCamera(pointsBoard, pointsColor, size, cameraMatrix, distortion, rvecs, tvecs, 0, termCriteria);
//		std::cout<<"re-projection error: " << error << std::endl;

		std::cout<<"Camera Matrix:" << std::endl << cameraMatrix<<std::endl;
		std::cout<<"Distortion Coeeficients:" << std::endl << distortion << std::endl;
//		std::vector<cv::Mat> R_matrices;
//		R_matrices.resize(rvecs.size());
		cv::Mat rvec,tvec;
		std::string file_name=seq_path+calib_measure.cur_idx+".txt";
		std::ofstream fp;
		fp.open(file_name,std::ios::out);
		for(int i=0;i<calib_measure.Ts_tag_sensor.size();i++)
		{
			cv::solvePnP(pointsBoard[i],pointsColor[i],cameraMatrix,distortion,rvec,tvec);
//			std::cout<<std::endl<<"rotation:"<<rvecs[i].t()<<std::endl;
//			std::cout<<"translation:"<<tvecs[i].t()<<std::endl;
//			cv::Rodrigues(rvecs[i],R_matrices[i]);
//			std::cout<<"rotation matrix:"<<std::endl<<R_matrices[i]<<std::endl;

			calib_measure.Ts_tag_sensor[i].pose=Mat2Trans(rvec,tvec);
			std::cout<<"transform:"<<std::endl<<calib_measure.Ts_tag_sensor[i].pose.getMatrix()<<std::endl;
			std::cout<<"quat:"<<calib_measure.Ts_tag_sensor[i].pose.Quat().w()<<" "<<calib_measure.Ts_tag_sensor[i].pose.Quat().vec().transpose()<<std::endl;
			fp<<calib_measure.Ts_tag_sensor[i].pose.t.transpose()<<" "<<calib_measure.Ts_tag_sensor[i].pose.Quaternion().transpose()<<std::endl;
		}
		fp.close();

	}
	
	Transform SystemExtrinsicCalibration::Mat2Trans(cv::Mat r, cv::Mat t)
	{
		cv::Mat R;
		cv::Rodrigues(r,R);
		Transform T;
		for(int i=0;i<3;i++)
		{
			T.t(i)=t.at<double>(i);
			for(int j=0;j<3;j++)
			{
				T.R(i,j)=R.at<double>(i,j);
			}
		}
		return T.inv();
	}

	void SystemExtrinsicCalibration::shutDown()
	{
	}

	void SystemExtrinsicCalibration::visTraj(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointXYZRGBA pt1,pt2;

		// add occluding points;
		std::cout<<"sensor trajectory - red"<<std::endl;
		std::cout<<"marker trajectory - green"<<std::endl;
		cloud->resize(T_tag_sensor.size()+T_mocap_marker.size());

		for(int i=0;i<T_tag_sensor.size();i++)
		{
			cloud->at(i).x=T_tag_sensor[i].pose.t(0);
			cloud->at(i).y=T_tag_sensor[i].pose.t(1);
			cloud->at(i).z=T_tag_sensor[i].pose.t(2);
			cloud->at(i).r=255;
			cloud->at(i).g=0;
			cloud->at(i).b=0;
			if(i>0)
			{
				pt1.x=T_tag_sensor[i].pose.t(0);
				pt1.y=T_tag_sensor[i].pose.t(1);
				pt1.z=T_tag_sensor[i].pose.t(2);
				pt2.x=T_tag_sensor[i-1].pose.t(0);
				pt2.y=T_tag_sensor[i-1].pose.t(1);
				pt2.z=T_tag_sensor[i-1].pose.t(2);
				sprintf(id,"%dline",i);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
			}
		}

		for(int i=0;i<T_mocap_marker.size();i++)
		{
			int j=T_tag_sensor.size()+i;
			Eigen::Vector3d t =T_mocap_marker[i].pose.t;
			Eigen::Vector3d t0=T_mocap_marker[i-1].pose.t;
			cloud->at(j).x=t(0);
			cloud->at(j).y=t(1);
			cloud->at(j).z=t(2);
			cloud->at(j).r=0;
			cloud->at(j).g=255;
			cloud->at(j).b=0;
				pt1.x=T_tag_sensor[i].pose.t(0);
				pt1.y=T_tag_sensor[i].pose.t(1);
				pt1.z=T_tag_sensor[i].pose.t(2);
				pt2.x=t(0);
				pt2.y=t(1);
				pt2.z=t(2);
				sprintf(id,"%dedge",i);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);
			if(i>0)
			{
				pt1.x=t(0);
				pt1.y=t(1);
				pt1.z=t(2);
				pt2.x=t0(0);
				pt2.y=t0(1);
				pt2.z=t0(2);
				sprintf(id,"%dline",j);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
			}
		}

		sprintf(id,"traj");
		if (!vis->updatePointCloud (cloud, id))
			vis->addPointCloud (cloud, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);
		vis->spin();
	}

	void SystemExtrinsicCalibration::visChessCalib(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		pcl::PointXYZRGBA pt1,pt2;

		std::cout<<"x axis - red"<<std::endl;
		std::cout<<"y axis - greem"<<std::endl;
		std::cout<<"z axis - blue"<<std::endl;

		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0.2; pt2.y=0; pt2.z=0;
		sprintf(id,"chess_x");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0.2; pt2.z=0;
		sprintf(id,"chess_y");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0; pt2.z=0.2;
		sprintf(id,"chess_z");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
		

		for(int i=0;i<T_tag_sensor.size();i++)
		{
			pt1.x=T_tag_sensor[i].pose.t(0);
			pt1.y=T_tag_sensor[i].pose.t(1);
			pt1.z=T_tag_sensor[i].pose.t(2);
			pt2.x=pt1.x+T_tag_sensor[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+T_tag_sensor[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+T_tag_sensor[i].pose.R(2,0)*0.1;
			sprintf(id,"%dline_x",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id); // red

			pt2.x=pt1.x+T_tag_sensor[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+T_tag_sensor[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+T_tag_sensor[i].pose.R(2,1)*0.1;
			sprintf(id,"%dline_y",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id); // green 

			pt2.x=pt1.x+T_tag_sensor[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+T_tag_sensor[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+T_tag_sensor[i].pose.R(2,2)*0.1;
			sprintf(id,"%dline_z",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id); // blue
		}

		for(int i=0;i<T_mocap_marker.size();i++)
		{
			pt1.x=T_mocap_marker[i].pose.t(0);
			pt1.y=T_mocap_marker[i].pose.t(1);
			pt1.z=T_mocap_marker[i].pose.t(2);
			pt2.x=pt1.x+T_mocap_marker[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+T_mocap_marker[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+T_mocap_marker[i].pose.R(2,0)*0.1;
			sprintf(id,"%dmline_x",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,255,id); // magenta

			pt2.x=pt1.x+T_mocap_marker[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+T_mocap_marker[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+T_mocap_marker[i].pose.R(2,1)*0.1;
			sprintf(id,"%dmline_y",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,0,id); // yellow

			pt2.x=pt1.x+T_mocap_marker[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+T_mocap_marker[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+T_mocap_marker[i].pose.R(2,2)*0.1;
			sprintf(id,"%dmline_z",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,255,id); // cyan
		}

		for(int i=0;i<T_mocap_marker2.size();i++)
		{
			pt1.x=T_mocap_marker2[i].pose.t(0);
			pt1.y=T_mocap_marker2[i].pose.t(1);
			pt1.z=T_mocap_marker2[i].pose.t(2);
			pt2.x=pt1.x+T_mocap_marker2[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+T_mocap_marker2[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+T_mocap_marker2[i].pose.R(2,0)*0.1;
			sprintf(id,"%dm2line_x",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,200,0,100,id);

			pt2.x=pt1.x+T_mocap_marker2[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+T_mocap_marker2[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+T_mocap_marker2[i].pose.R(2,1)*0.1;
			sprintf(id,"%dm2line_y",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,200,200,0,id);

			pt2.x=pt1.x+T_mocap_marker2[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+T_mocap_marker2[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+T_mocap_marker2[i].pose.R(2,2)*0.1;
			sprintf(id,"%dm2line_z",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,200,200,id);
		}

		vis->spin();
	}

	void SystemExtrinsicCalibration::visStatic(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		pcl::PointXYZRGBA pt1,pt2;

		std::cout<<"x axis - red"<<std::endl;
		std::cout<<"y axis - greem"<<std::endl;
		std::cout<<"z axis - blue"<<std::endl;

		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0.2; pt2.y=0; pt2.z=0;
		sprintf(id,"chess_x");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0.2; pt2.z=0;
		sprintf(id,"chess_y");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		pt1.x=0; pt1.y=0; pt1.z=0;
		pt2.x=0; pt2.y=0; pt2.z=0.2;
		sprintf(id,"chess_z");
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
		

		int i=0;
		for(int j=0;j<calib_measures.size();j++)
		{
			// T_tag_camera
			pt1.x=calib_measures[j].Ts_tag_sensor[i].pose.t(0);
			pt1.y=calib_measures[j].Ts_tag_sensor[i].pose.t(1);
			pt1.z=calib_measures[j].Ts_tag_sensor[i].pose.t(2);
			pt2.x=pt1.x+calib_measures[j].Ts_tag_sensor[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_tag_sensor[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_tag_sensor[i].pose.R(2,0)*0.1;
			sprintf(id,"%dline_x",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id); // red

			pt2.x=pt1.x+calib_measures[j].Ts_tag_sensor[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_tag_sensor[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_tag_sensor[i].pose.R(2,1)*0.1;
			sprintf(id,"%dline_y",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id); // green 

			pt2.x=pt1.x+calib_measures[j].Ts_tag_sensor[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_tag_sensor[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_tag_sensor[i].pose.R(2,2)*0.1;
			sprintf(id,"%dline_z",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id); // blue

			// T_mocap_marker
			pt1.x=calib_measures[j].Ts_mocap_marker[i].pose.t(0);
			pt1.y=calib_measures[j].Ts_mocap_marker[i].pose.t(1);
			pt1.z=calib_measures[j].Ts_mocap_marker[i].pose.t(2);
			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker[i].pose.R(2,0)*0.1;
			sprintf(id,"%dmline_x",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,255,id); // magenta

			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker[i].pose.R(2,1)*0.1;
			sprintf(id,"%dmline_y",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,0,id); // yellow

			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker[i].pose.R(2,2)*0.1;
			sprintf(id,"%dmline_z",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,255,id); // cyan

			// T_mocap_marker2
			pt1.x=calib_measures[j].Ts_mocap_marker2[i].pose.t(0);
			pt1.y=calib_measures[j].Ts_mocap_marker2[i].pose.t(1);
			pt1.z=calib_measures[j].Ts_mocap_marker2[i].pose.t(2);
			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker2[i].pose.R(0,0)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker2[i].pose.R(1,0)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker2[i].pose.R(2,0)*0.1;
			sprintf(id,"%dm2line_x",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,200,0,100,id);

			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker2[i].pose.R(0,1)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker2[i].pose.R(1,1)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker2[i].pose.R(2,1)*0.1;
			sprintf(id,"%dm2line_y",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,200,200,0,id);

			pt2.x=pt1.x+calib_measures[j].Ts_mocap_marker2[i].pose.R(0,2)*0.1;
			pt2.y=pt1.y+calib_measures[j].Ts_mocap_marker2[i].pose.R(1,2)*0.1;
			pt2.z=pt1.z+calib_measures[j].Ts_mocap_marker2[i].pose.R(2,2)*0.1;
			sprintf(id,"%dm2line_z",j);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,200,200,id);
		}

		vis->spin();
	}
}
