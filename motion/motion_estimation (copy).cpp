/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-21 10:41
#
# Filename:		motion_estimation.cpp
#
# Description: 
#
===============================================*/

#include "motion/motion_estimation.h"

namespace ulysses
{
	using namespace std;
	PlaneLineBA::PlaneLineBA(Map *map)//, int num_frames_, bool use_plane_, bool use_line_, bool use_shadow_)
	{
//		std::ofstream fp;
//		fp.open("plane_line_ba.txt",std::ios::out);
//		fp<<std::endl<<std::fixed<<map->timestamps[map->timestamps.size()-1]<<std::endl;
		
//		const int idx_camera_initial=map->cameras.size()-num_frames_;
//
//		num_cameras_=num_frames_;
//		num_observ_planes_=0;
//		num_observ_lines_=0;
////		num_observ_shadows_=0;
//		num_planes_=0;
//		num_lines_=0;
//
//		// if(plane_mask[i]) then map->planes[i] is included;
//		std::vector<int> plane_mask;
//		plane_mask.resize(map->planes.size());
//		for(int i=0;i<plane_mask.size();i++) plane_mask[i]=-1;
//		std::vector<int> line_mask;
//		line_mask.resize(map->lines.size());
//		for(int i=0;i<line_mask.size();i++) line_mask[i]=-1;
//
//		for(int i=idx_camera_initial;i<map->cameras.size();i++)
//		{
//			if(use_plane_) 
//			for(size_t j=0;j<map->observ_planes.size();j++)
//			{
//				if(map->observ_planes[j]->idx_camera==i)
//				{
//					num_observ_planes_++;
//					plane_mask[map->observ_planes[j]->idx_plane]=0;
//				}
//			}
//			if(use_line_) 
//			for(size_t j=0;j<map->observ_lines.size();j++)
//			{
//				if(map->observ_lines[j]->idx_camera==i)
//				{
//					num_observ_lines_++;
//					line_mask[map->observ_lines[j]->idx_line]=0;
//				}
//			}
////			if(use_shadow_)
////			for(size_t j=0;j<map->observ_shadows.size();j++)
////			{
////				if(map->observ_shadows[j]->idx_camera==i)
////				{
////					num_observ_shadows_++;
////				}
////			}
//		}

//		num_observations_=num_observ_planes_+num_observ_lines_;
//		if(use_plane_) for(int i=0;i<plane_mask.size();i++) if(plane_mask[i]>=0) num_planes_++;
//		if(use_line_)  for(int i=0;i<line_mask.size();i++)  if(line_mask[i]>=0)  num_lines_++;
//
//		num_parameters_=7*num_cameras_+4*num_planes_+6*num_lines_;
//		parameters_=new double[num_parameters_];
//
//		camera_index_=new int[num_observations_];
//		feature_index_=new int[num_observations_];
//		observations_=new double[4*num_observ_planes_+6*num_observ_lines_];
//		sqrt_information_=new double[16*num_observ_planes_+36*num_observ_lines_];
//		feature_index_map_=new int[num_planes_+num_lines_];

//		shadow_camera_index_=new int[num_observ_shadows_];
//		shadow_plane_index_=new int[num_observ_shadows_];
//		shadow_line_index_=new int[num_observ_shadows_];
//		observ_shadows_=new double[6*num_observ_shadows_];
//		sqrt_information_shadows_=new double[36*num_observ_shadows_];

//		int* cursor_camera=camera_index_;
//		int* cursor_feature =feature_index_;
//		double* cursor=observations_;
//		double* cursor_info=sqrt_information_;
//		double* cursor_param=parameters_;
//		int* cursor_feature_map =feature_index_map_;

//		int* cursor_shadow_camera_index=shadow_camera_index_;
//		int* cursor_shadow_plane_index=shadow_plane_index_;
//		int* cursor_shadow_line_index=shadow_line_index_;
//		double* cursor_observ_shadows=observ_shadows_;
//		double* cursor_info_shadows=sqrt_information_shadows_;

		// parameters_ - cameras
//		for(int i=idx_camera_initial;i<map->cameras.size();i++)
//		fp<<"parameters - camera"<<endl;
		num_cameras_=0;
		std::map<double,int> camera_indices; int counter=0;
		for(const_iterCamera it=map->beginCamera();it!=map->endCamera();it++)
		{
			Transform T=it->second;
			Eigen::Quaterniond quat=T.Quat();
			parameters_.push_back(quat.w());
			parameters_.push_back(quat.x());
			parameters_.push_back(quat.y());
			parameters_.push_back(quat.z());
			parameters_.push_back(T.t(0));
			parameters_.push_back(T.t(1));
			parameters_.push_back(T.t(2));
			timestamps_.push_back(it->first);
//			fp<<counter<<endl;
//			fp<<fixed<<it->first<<" "<<T<<endl;
			camera_indices[it->first]=counter++;
			num_cameras_++;
		}

//		int counter_planes=0;
//		int counter_lines=0;
		// parameters_ - features;
//		fp<<"parameters - features"<<endl;
		num_planes_=0; num_lines_=0; counter=0;
		num_observ_planes_=0; num_observ_lines_=0;
		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
		{
			Landmark* lm=it->second;
//			fp<<counter<<endl;
//			fp<<lm<<endl;
			if(lm->Type()==PLANE)
			{
				PlaneLM* p=lm->planelm();
				parameters_.push_back(p->pi(0));
				parameters_.push_back(p->pi(1));
				parameters_.push_back(p->pi(2));
				parameters_.push_back(p->pi(3));
				num_planes_++;
			}
			else if(lm->Type()==LINE)
			{
				LineLM* l=lm->linelm();
				parameters_.push_back(l->L(0));
				parameters_.push_back(l->L(1));
				parameters_.push_back(l->L(2));
				parameters_.push_back(l->L(3));
				parameters_.push_back(l->L(4));
				parameters_.push_back(l->L(5));
				num_lines_++;
			}
			features_id_.push_back(it->first);
			for(iterObserv it_obs=lm->beginObserv();it_obs!=lm->endObserv();it_obs++)
			{
				Scan* s=map->scan(it_obs->first);
				for(int j=0;j<it_obs->second.size();j++)
				{
					Feature* f=s->findFeature(it_obs->second[j]);
//					fp<<"\t"<<fixed<<it_obs->first<<" "<<f<<endl;
					if(f->Type()==PLANE)
					{
						observations_.push_back(f->plane()->n(0));
						observations_.push_back(f->plane()->n(1));
						observations_.push_back(f->plane()->n(2));
						observations_.push_back(f->plane()->d);
						for(int i=0;i<16;i++) sqrt_information_.push_back(f->plane()->sqrt_info.data()[i]);
						num_observ_planes_++;
						
					}
					else if(f->Type()==LINE)
					{
						observations_.push_back(f->line()->u(0));
						observations_.push_back(f->line()->u(1));
						observations_.push_back(f->line()->u(2));
						observations_.push_back(f->line()->v(0));
						observations_.push_back(f->line()->v(1));
						observations_.push_back(f->line()->v(2));
						for(int i=0;i<36;i++) sqrt_information_.push_back(f->line()->sqrt_info.data()[i]);
						num_observ_lines_++;
					}
					camera_index_.push_back(camera_indices.find(it_obs->first)->second);
					feature_index_.push_back(counter);
				}
			}
			counter++;
		}
		num_observations_=num_observ_planes_+num_observ_lines_;
		num_parameters_=7*num_cameras_+4*num_planes_+6*num_lines_;
//		for(int i=0;i<plane_mask.size();i++)
//		{
//			if(plane_mask[i]>=0)
//			{
//				memcpy(cursor_param,map->planes[i]->pi.data(),4*sizeof(double));
//				cursor_param+=4;
//				*cursor_feature_map++=i;
//				plane_mask[i]=counter_planes++;
//			}
//		}
//		// parameters_ - lines
//		for(int i=0;i<line_mask.size();i++)
//		{
//			if(line_mask[i]>=0)
//			{
//				memcpy(cursor_param,map->lines[i]->L.data(),6*sizeof(double));
//				cursor_param+=6;
//				*cursor_feature_map++=i;
//				line_mask[i]=counter_lines++;
//			}
//		}

//		int counter=0;
//		if(use_plane_)
//		for(int i=idx_camera_initial;i<map->cameras.size();i++)
//		{
//			for(size_t j=0;j<map->observ_planes.size();j++)
//			{
//				if(map->observ_planes[j]->idx_camera==i)
//				{
//					*cursor_feature++=plane_mask[map->observ_planes[j]->idx_plane];
//					*cursor_camera++=map->observ_planes[j]->idx_camera-idx_camera_initial;
//					memcpy(cursor,map->observ_planes[j]->pi.data(),4*sizeof(double));
//					cursor+=4;
//					memcpy(cursor_info,map->observ_planes[j]->sqrt_info.data(),16*sizeof(double));
//					cursor_info+=16;
////					for(int k=0;k<map->observ_planes[j]->plane_ptr->points.size();k++)
////					{
////						Eigen::Vector3d pt;
////						pt(0)=map->observ_planes[j]->plane_ptr->points[k].xyz(0);
////						pt(1)=map->observ_planes[j]->plane_ptr->points[k].xyz(1);
////						pt(2)=map->observ_planes[j]->plane_ptr->points[k].xyz(2);
////						points_plane.push_back(pt);
////					}
//				}
//			}
//		}
//
//		counter=0;
//		if(use_line_)
//		for(int i=idx_camera_initial;i<map->cameras.size();i++)
//		{
//			for(size_t j=0;j<map->observ_lines.size();j++)
//			{
//				if(map->observ_lines[j]->idx_camera==i)
//				{
////					*cursor_feature++=map->observ_lines[j]->idx_line;
//					*cursor_feature++=line_mask[map->observ_lines[j]->idx_line];
//					*cursor_camera++=map->observ_lines[j]->idx_camera-idx_camera_initial;
//					memcpy(cursor,map->observ_lines[j]->L.data(),6*sizeof(double));
//					cursor+=6;
//					memcpy(cursor_info,map->observ_lines[j]->sqrt_info.data(),36*sizeof(double));
//					cursor_info+=36;
////					for(std::list<EdgePoint*>::iterator it=map->observ_lines[j]->line_ptr->points.begin();
////					                                    it!=map->observ_lines[j]->line_ptr->points.end();it++)
////					{
////						Eigen::Vector3d pt;
////						pt(0)=(*it)->xyz(0);
////						pt(1)=(*it)->xyz(1);
////						pt(2)=(*it)->xyz(2);
////						points_line.push_back(pt);
////					}
//				}
//			}
//		}

//		counter=0;
//		if(use_shadow_)
//		for(int i=idx_camera_initial;i<map->cameras.size();i++)
//		{
//			for(size_t j=0;j<map->observ_shadows.size();j++)
//			{
//				if(map->observ_shadows[j]->idx_camera==i)
//				{
//					*cursor_shadow_camera_index++=map->observ_shadows[j]->idx_camera-idx_camera_initial;
//					*cursor_shadow_plane_index++=plane_mask[map->observ_shadows[j]->idx_plane];
//					*cursor_shadow_line_index++=line_mask[map->observ_shadows[j]->idx_line];
//					memcpy(cursor_observ_shadows,map->observ_shadows[j]->L.data(),6*sizeof(double));
//					cursor_observ_shadows+=6;
//					memcpy(cursor_info_shadows,map->observ_shadows[j]->sqrt_info.data(),36*sizeof(double));
//					cursor_info_shadows+=36;
////					for(std::list<EdgePoint*>::iterator it=map->observ_shadows[j]->line_ptr->points.begin();
////					                                    it!=map->observ_shadows[j]->line_ptr->points.end();it++)
////					{
////						Eigen::Vector3d pt;
////						pt(0)=(*it)->xyz(0);
////						pt(1)=(*it)->xyz(1);
////						pt(2)=(*it)->xyz(2);
////						points_shadow.push_back(pt);
////					}
//				}
//			}
//		}

//		if(true)
//		{
//			fp<<"num_observations_ = "<<num_observations_<<std::endl;
//			fp<<"num_observ_planes_ = "<<num_observ_planes_<<std::endl;
//			fp<<"num_observ_lines_ = "<<num_observ_lines_<<std::endl<<std::endl;
////			fp<<"num_observ_shadows_ = "<<num_observ_shadows_<<std::endl<<std::endl;
//			fp<<"num_parameters_ = "<<num_parameters_<<std::endl;
//			fp<<"num_cameras_ = "<<num_cameras_<<std::endl;
//			fp<<"num_planes_ = "<<num_planes_<<std::endl;
//			fp<<"num_lines_ = "<<num_lines_<<std::endl<<std::endl;
//
//			cursor=observations_;
//			cursor_info=sqrt_information_;
//			fp<<"\nplane observations_ "<<std::endl;
//			for(size_t i=0;i<num_observ_planes_;i++)
//			{
//				fp<<i<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<std::endl;
//				for(size_t j=0;j<16;j++)
//				{
//					fp<<"\t"<<*cursor_info++;
//				}
//				fp<<std::endl;
//			}
//			fp<<"\nline observations_ "<<std::endl;
//			for(size_t i=0;i<num_observ_lines_;i++)
//			{
//				fp<<i<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<std::endl;
//				for(size_t j=0;j<36;j++)
//				{
//					fp<<"\t"<<*cursor_info++;
//				}
//				fp<<std::endl;
//			}
////			fp<<"\nshadow observations_ "<<std::endl;
////			cursor=observ_shadows_;
////			cursor_info=sqrt_information_shadows_;
////			for(size_t i=0;i<num_observ_shadows_;i++)
////			{
////				fp<<i<<"\t";
////				fp<<*cursor++<<"\t";
////				fp<<*cursor++<<"\t";
////				fp<<*cursor++<<"\t";
////				fp<<*cursor++<<"\t";
////				fp<<*cursor++<<"\t";
////				fp<<*cursor++<<std::endl;
////				for(size_t j=0;j<36;j++)
////				{
////					fp<<"\t"<<*cursor_info++;
////				}
////				fp<<std::endl;
////			}
//
//			cursor_camera=camera_index_;
//			cursor_feature=feature_index_;
//			fp<<"\nfeature_index_"<<std::endl;
//			for(size_t i=0;i<num_observ_planes_;i++)
//			{
//				fp<<i<<"\t"<<*cursor_feature++<<std::endl;
//			}
//			for(size_t i=0;i<num_observ_lines_;i++)
//			{
//				fp<<i+num_observ_planes_<<"\t"<<*cursor_feature++<<std::endl;
//			}
//
//			fp<<"\ncamera_index_"<<std::endl;
//			for(size_t i=0;i<num_observations_;i++)
//			{
//				fp<<i<<"\t"<<*cursor_camera++<<std::endl;
//			}
//
////			cursor_shadow_camera_index=shadow_camera_index_;
////			cursor_shadow_plane_index=shadow_plane_index_;
////			cursor_shadow_line_index=shadow_line_index_;
////			fp<<"\nshadow_index_"<<std::endl;
////			for(size_t i=0;i<num_observ_shadows_;i++)
////			{
////				fp<<i<<"\t"<<*cursor_shadow_camera_index++
////					 <<"\t"<<*cursor_shadow_plane_index++
////					 <<"\t"<<*cursor_shadow_line_index++<<std::endl;
////			}
//
//			cursor=parameters_;
//			fp<<"\nparameters_"<<std::endl;
//			for(size_t i=0;i<num_cameras_;i++)
//			{
//				fp<<i<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<std::endl;
//			}
//			for(size_t i=0;i<num_planes_;i++)
//			{
//				fp<<i<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<std::endl;
//			}
//			for(size_t i=0;i<num_lines_;i++)
//			{
//				fp<<i<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<"\t";
//				fp<<*cursor++<<std::endl;
//			}
//		}

//		fp.close();
	}

//	void PlaneLineBA::computePsi(const int &idx_camera, Matrix6d &Psi)
//	{
//		Psi.setZero();
//
//		const int camera_block = camera_block_size();
//		const int plane_block  = plane_block_size();
//		const int line_block   = line_block_size();
//		double* cameras = mutable_cameras();
//		double* planes  = mutable_planes();
//		double* lines   = mutable_lines();
//
//		double* camera=cameras+camera_block*idx_camera;
//		Eigen::Quaterniond quat(camera[0],camera[1],camera[2],camera[3]);
//		Eigen::Vector3d t(camera[4],camera[5],camera[6]);
//		Transform Tcg(quat,t);
//		Eigen::Matrix3d R=Tcg.R;
//
//		double* cursor_observations=observations();
//		double* cursor_sqrt_information=sqrt_information();
//		for(int i=0;i<num_observ_planes();i++)
//		{
//			if(camera_index()[i]==idx_camera)
//			{
//				Eigen::Vector4d z_pi(cursor_observations);
//				cursor_observations+=plane_block;
//				Eigen::Matrix4d information(cursor_sqrt_information);
//				cursor_sqrt_information+=plane_block*plane_block;
//
//				double* plane=planes+plane_block*feature_index()[i];
//				Eigen::Vector4d pi(plane);
//				Eigen::Vector3d n=pi.block<3,1>(0,0);
//				n.normalize();
//
//				Eigen::Matrix<double,4,6> e_pi_xi;
//				Eigen::Vector3d Rn=R*n;
//				e_pi_xi.block<3,3>(0,0).setZero();
//				e_pi_xi.block<3,3>(0,3)=Transform::skew_sym(Rn);
//				e_pi_xi.block<1,3>(3,0)=Rn.transpose();
//				e_pi_xi.block<1,3>(3,3)=-t.transpose()*Transform::skew_sym(Rn);
//
//				Psi+=e_pi_xi.transpose()*information*information*e_pi_xi;
//			}
//		}
//
//		for(size_t i=0;i<num_observ_lines();i++)
//		{
//			if(camera_index()[i+num_observ_planes()]==idx_camera)
//			{
//				Vector6d z_L(cursor_observations);
//				cursor_observations+=line_block;
//				Matrix6d information(cursor_sqrt_information);
//				cursor_sqrt_information+=line_block*line_block;
//
//				double* line=lines+line_block*feature_index()[i+num_observ_planes()];
//				Vector6d L(line);
//				Eigen::Vector3d u=L.block<3,1>(0,0);
//				Eigen::Vector3d v=L.block<3,1>(3,0);
//				u=u/v.norm();
//				v.normalize();
//
//				Matrix6d e_L_xi;
//				Eigen::Vector3d Ru=R*u;
//				Eigen::Vector3d Rv=R*v;
//				e_L_xi.block<3,3>(0,0)=Transform::skew_sym(Rv);
//				e_L_xi.block<3,3>(0,3)=Transform::skew_sym(Ru)+Transform::skew_sym(t)*Transform::skew_sym(Rv);
//				e_L_xi.block<3,3>(3,0).setZero();
//				e_L_xi.block<3,3>(3,3)=Transform::skew_sym(Rv);
//
//				Psi+=e_L_xi.transpose()*information*information*e_L_xi;
//			}
//		}
//
////		double* cursor_observ_shadows=observ_shadows();
////		double* cursor_sqrt_information_shadows=sqrt_information_shadows();
////		for(size_t i=0;i<num_observ_shadows();i++)
////		{
////			if(shadow_camera_index()[i]==idx_camera)
////			{
////				Vector6d z_Lpi(cursor_observ_shadows);
////				cursor_observ_shadows+=line_block;
////				Matrix6d information(cursor_sqrt_information_shadows);
////				cursor_sqrt_information_shadows+=line_block*line_block;
////
////				double* plane=planes+plane_block*shadow_plane_index()[i];
////				Eigen::Vector4d pi(plane);
////				Eigen::Vector3d n=pi.block<3,1>(0,0);
////				double d=pi(3)/n.norm();
////				n.normalize();
////
////				double* line=lines+line_block*shadow_line_index()[i];
////				Vector6d L(line);
////				Eigen::Vector3d u=L.block<3,1>(0,0);
////				Eigen::Vector3d v=L.block<3,1>(3,0);
////				u=u/v.norm();
////				v.normalize();
////
////				Eigen::Vector3d t_gc=-R.transpose()*t;
////				Eigen::Vector3d uu=-d*(Transform::skew_sym(v)*t_gc+u)-n*t_gc.transpose()*u;
////				Eigen::Vector3d vv=-Transform::skew_sym(n)*Transform::skew_sym(v)*t_gc+Transform::skew_sym(u)*n;
////				double vv_norm=vv.norm();
////				uu=uu/vv_norm;
////				vv.transpose();
////
////				Eigen::Matrix3d Rv_pi=Transform::skew_sym(R*vv);
////				Eigen::Matrix3d Ru_pi=Transform::skew_sym(R*uu);
////
////				Matrix6d tmp_6d;
////				tmp_6d.block<3,3>(0,0)=R;
////				tmp_6d.block<3,3>(0,3)=Transform::skew_sym(t)*R;
////				tmp_6d.block<3,3>(3,0).setZero();
////				tmp_6d.block<3,3>(3,3)=R;
////
////				Eigen::Matrix<double,6,3> tmp_63;
////				tmp_63.block<3,3>(0,0)=d*Transform::skew_sym(v)+n*u.transpose()
////									   -uu*vv.transpose()*Transform::skew_sym(n)*Transform::skew_sym(v);
////				tmp_63.block<3,3>(3,0)=(Eigen::Matrix3d::Identity()-vv*vv.transpose())
////									   *Transform::skew_sym(n)*Transform::skew_sym(v);
////
////				Eigen::Matrix<double,3,6> tmp_36;
////				tmp_36.block<3,3>(0,0).setIdentity();
////				tmp_36.block<3,3>(0,3)=Transform::skew_sym(t);
////
////				tmp_6d=tmp_6d*tmp_63*R.transpose()*tmp_36;
////				tmp_6d/=-vv_norm;
////
////				Matrix6d e_Lpi_xi;
////				e_Lpi_xi.block<3,3>(0,0)=tmp_6d.block<3,3>(0,0)+Rv_pi;
////				e_Lpi_xi.block<3,3>(0,3)=tmp_6d.block<3,3>(0,3)+Ru_pi+Transform::skew_sym(t)*Rv_pi;
////				e_Lpi_xi.block<3,3>(3,0)=tmp_6d.block<3,3>(3,0);
////				e_Lpi_xi.block<3,3>(3,3)=tmp_6d.block<3,3>(3,3)+Rv_pi;
////
////				Psi+=e_Lpi_xi.transpose()*information*information*e_Lpi_xi;
////			}
////		}
//	}

	/*
	PlaneLineBA::PlaneLineBA(std::vector<Scan*> scans, bool use_plane_, bool use_line_)
	{
		std::ofstream fp;
		fp.open("ba_debug.txt",std::ios::out);

		num_cameras_=scans.size()+1; // all the scans and scans[0]->scan_ref;
		num_observations_=0;
		num_observ_planes_=0;
		num_observ_lines_=0;
		for(size_t i=0;i<scans.size();i++)
		{
			if(i==0)
			{
				if(use_plane_)
				{
					num_observations_+=scans[i]->plane_matches.size()*2;
					num_observ_planes_+=scans[i]->plane_matches.size()*2;
				}
				if(use_line_)
				{
					num_observations_+=scans[i]->line_matches.size()*2;
					num_observ_lines_ +=scans[i]->line_matches.size()*2;
				}
			}
			else
			{
				if(use_plane_)
				{
					num_observations_+=scans[i]->plane_matches.size();
					num_observ_planes_+=scans[i]->plane_matches.size();
				}
				if(use_line_)
				{
					num_observations_+=scans[i]->line_matches.size();
					num_observ_lines_ +=scans[i]->line_matches.size();
				}
			}
		}

		feature_index_=new int[num_observations_];
		camera_index_=new int[num_observations_];
		observations_=new double[4*num_observ_planes_+6*num_observ_lines_];
		sqrt_information_=new double[16*num_observ_planes_+36*num_observ_lines_];

		// construct the plane_map_;
		int* cursor_camera=camera_index_;
		int* cursor_feature =feature_index_;
		double* cursor=observations_;
		double* cursor_info=sqrt_information_;
		// add scan_ref to the plane_map_;
		if(use_plane_)
		for(size_t j=0;j<scans[0]->plane_matches.size();j++)
		{
			fp<<"\nadd a new plane landmark (ref)\tscan_id "<<scans[0]->scan_ref->id<<"\tplane_id "<<scans[0]->plane_matches[j].ref->id<<std::endl;
//			fp<<scans[0]->plane_matches[j].ref->n.transpose()<<"\t"<<scans[0]->plane_matches[j].ref->d<<std::endl;
//			fp<<scans[0]->scan_ref->Tcg.inv().getMatrix()<<std::endl;
			PlaneLM *plane_lm=new PlaneLM(scans[0]->plane_matches[j].ref,
										  scans[0]->scan_ref->Tcg.inv());
			fp<<"\tplane_lm  "<<plane_lm->pi(0)<<"\t"<<plane_lm->pi(1)<<"\t"<<plane_lm->pi(2)<<"\t"<<plane_lm->pi(3)<<std::endl;
			plane_lm->addObservation(scans[0]->plane_matches[j].ref,scans[0]->scan_ref->Tcg.inv());
			fp<<"\tobserv_cameras "<<plane_lm->cameras[0].t.transpose()<<std::endl;
			fp<<"\tobserv_planes  "<<plane_lm->planes[0]->n.transpose()<<std::endl;
			plane_lm->id=plane_map_.size();
			plane_map_.push_back(plane_lm);

			*cursor_feature++=plane_lm->id;
			*cursor_camera++=0;
			*cursor++=scans[0]->plane_matches[j].ref->n(0);
			*cursor++=scans[0]->plane_matches[j].ref->n(1);
			*cursor++=scans[0]->plane_matches[j].ref->n(2);
			*cursor++=scans[0]->plane_matches[j].ref->d;
//			fp<<scans[0]->plane_matches[j].ref->sqrt_info<<std::endl;
			memcpy(cursor_info,scans[0]->plane_matches[j].ref->sqrt_info.data(),16*sizeof(double));
//			fp<<"\t"<<"sqrt_information "<<std::endl
//			  <<"\t"<<cursor_info[0]<<"\t" <<cursor_info[1]<<"\t" <<cursor_info[2]<<"\t" <<cursor_info[3]<<std::endl    
//			  <<"\t"<<cursor_info[4]<<"\t" <<cursor_info[5]<<"\t" <<cursor_info[6]<<"\t" <<cursor_info[7]<<std::endl    
//			  <<"\t"<<cursor_info[8]<<"\t" <<cursor_info[9]<<"\t" <<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl  
//			  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]<<"\t"<<cursor_info[15]<<std::endl
//			  <<std::endl;
			cursor_info+=16;
		}
		if(use_plane_)
		for(size_t i=0;i<scans.size();i++)
		{
			for(size_t j=0;j<scans[i]->plane_matches.size();j++)
			{ // for each plane observation;
				bool not_observed=true;
				for(size_t k=0;k<plane_map_.size();k++)
				{
					if(plane_map_[k]->isObserved(scans[i]->plane_matches[j].ref))
					{
						fp<<"\nobserved in plane_map_["<<k<<"] with "<<plane_map_[k]->planes.size()<<" observations"<<std::endl;
						fp<<"\t"<<"scan_id "<<scans[i]->id<<"\tplane_id "<<scans[i]->plane_matches[j].cur->id<<std::endl;
						plane_map_[k]->addObservation(scans[i]->plane_matches[j].cur,scans[i]->Tcg.inv());
						*cursor_feature++=plane_map_[k]->id;
						*cursor_camera++=i+1;
						*cursor++=scans[i]->plane_matches[j].cur->n(0);
						*cursor++=scans[i]->plane_matches[j].cur->n(1);
						*cursor++=scans[i]->plane_matches[j].cur->n(2);
						*cursor++=scans[i]->plane_matches[j].cur->d;
//						fp<<scans[i]->plane_matches[j].cur->sqrt_info<<std::endl;
						memcpy(cursor_info,scans[i]->plane_matches[j].cur->sqrt_info.data(),16*sizeof(double));
//						fp<<"\t"<<"sqrt_information "<<std::endl
//						  <<"\t"<<cursor_info[0]<<"\t" <<cursor_info[1]<<"\t" <<cursor_info[2]<<"\t" <<cursor_info[3]<<std::endl    
//						  <<"\t"<<cursor_info[4]<<"\t" <<cursor_info[5]<<"\t" <<cursor_info[6]<<"\t" <<cursor_info[7]<<std::endl    
//						  <<"\t"<<cursor_info[8]<<"\t" <<cursor_info[9]<<"\t" <<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl  
//						  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]<<"\t"<<cursor_info[15]<<std::endl
//						  <<std::endl;
						cursor_info+=16;
						not_observed=false;
						break;
					}
				}
				if(not_observed)
				{
					fp<<"\nadd a new plane landmark\tscan_id "<<scans[i]->id<<"\tplane_id "<<scans[i]->plane_matches[j].cur->id<<std::endl;
					PlaneLM *plane_lm=new PlaneLM(scans[i]->plane_matches[j].cur,
												  scans[i]->Tcg.inv());
					plane_lm->addObservation(scans[i]->plane_matches[j].cur,scans[i]->Tcg.inv());
					plane_lm->id=plane_map_.size();
					plane_map_.push_back(plane_lm);
//						fp<<"\t"<<plane_lm->id<<"\t"<<plane_lm->pi.transpose()<<std::endl;

					*cursor_feature++=plane_lm->id;
					*cursor_camera++=i+1;
					*cursor++=scans[i]->plane_matches[j].cur->n(0);
					*cursor++=scans[i]->plane_matches[j].cur->n(1);
					*cursor++=scans[i]->plane_matches[j].cur->n(2);
					*cursor++=scans[i]->plane_matches[j].cur->d;
//					fp<<scans[i]->plane_matches[j].cur->sqrt_info<<std::endl;
					memcpy(cursor_info,scans[i]->plane_matches[j].cur->sqrt_info.data(),16*sizeof(double));
//					fp<<"\t"<<"sqrt_information "<<std::endl
//					  <<"\t"<<cursor_info[0]<<"\t" <<cursor_info[1]<<"\t" <<cursor_info[2]<<"\t" <<cursor_info[3]<<std::endl    
//					  <<"\t"<<cursor_info[4]<<"\t" <<cursor_info[5]<<"\t" <<cursor_info[6]<<"\t" <<cursor_info[7]<<std::endl    
//					  <<"\t"<<cursor_info[8]<<"\t" <<cursor_info[9]<<"\t" <<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl  
//					  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]<<"\t"<<cursor_info[15]<<std::endl
//					  <<std::endl;
					cursor_info+=16;
				}
			}
		}

		// add scan_ref to the line_map_;
		if(use_line_)
		for(size_t j=0;j<scans[0]->line_matches.size();j++)
		{
			fp<<"\nadd a new line landmark (ref)\tscan_id "<<scans[0]->scan_ref->id<<"\tline_id "<<scans[0]->line_matches[j].ref->id<<std::endl;
			LineLM *line_lm=new LineLM(scans[0]->line_matches[j].ref,
									   scans[0]->scan_ref->Tcg.inv());
			fp<<"\tline_lm "<<line_lm->L(0)<<"\t"<<line_lm->L(1)<<"\t"<<line_lm->L(2)<<"\t"<<line_lm->L(3)<<"\t"<<line_lm->L(4)<<"\t"<<line_lm->L(5)<<std::endl;
			line_lm->addObservation(scans[0]->line_matches[j].ref,scans[0]->scan_ref->Tcg.inv());
			fp<<"\tobserv_camera "<<line_lm->cameras[0].t.transpose()<<std::endl;
			fp<<"\tobserv_line   "<<line_lm->lines[0]->u.transpose()<<"\t"<<line_lm->lines[0]->v.transpose()<<std::endl;
			line_lm->id=line_map_.size();
			line_map_.push_back(line_lm);

			*cursor_feature++=line_lm->id;
			*cursor_camera++=0;
			*cursor++=scans[0]->line_matches[j].ref->u(0);
			*cursor++=scans[0]->line_matches[j].ref->u(1);
			*cursor++=scans[0]->line_matches[j].ref->u(2);
			*cursor++=scans[0]->line_matches[j].ref->v(0);
			*cursor++=scans[0]->line_matches[j].ref->v(1);
			*cursor++=scans[0]->line_matches[j].ref->v(2);
//			fp<<scans[0]->line_matches[j].ref->sqrt_info<<std::endl;
			memcpy(cursor_info,scans[0]->line_matches[j].ref->sqrt_info.data(),36*sizeof(double));
//			fp<<"\t"<<"sqrt_information "<<std::endl
//			  <<"\t"<<cursor_info[0]<<"\t"<<cursor_info[1]<<"\t"<<cursor_info[2]
//			  <<"\t"<<cursor_info[3]<<"\t"<<cursor_info[4]<<"\t"<<cursor_info[5]<<std::endl    
//			  <<"\t"<<cursor_info[6]<<"\t"<<cursor_info[7]<<"\t" <<cursor_info[8]
//			  <<"\t"<<cursor_info[9]<<"\t"<<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl    
//			  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]
//			  <<"\t"<<cursor_info[15]<<"\t"<<cursor_info[16]<<"\t"<<cursor_info[17]<<std::endl    
//			  <<"\t"<<cursor_info[18]<<"\t"<<cursor_info[19]<<"\t"<<cursor_info[20]
//			  <<"\t"<<cursor_info[21]<<"\t"<<cursor_info[22]<<"\t"<<cursor_info[23]<<std::endl    
//			  <<"\t"<<cursor_info[24]<<"\t"<<cursor_info[25]<<"\t"<<cursor_info[26]
//			  <<"\t"<<cursor_info[27]<<"\t"<<cursor_info[28]<<"\t"<<cursor_info[29]<<std::endl    
//			  <<"\t"<<cursor_info[30]<<"\t"<<cursor_info[31]<<"\t"<<cursor_info[32]
//			  <<"\t"<<cursor_info[33]<<"\t"<<cursor_info[34]<<"\t"<<cursor_info[35]<<std::endl    
//			  <<std::endl;
			cursor_info+=36;
		}
		if(use_line_)
		for(size_t i=0;i<scans.size();i++)
		{
			for(size_t j=0;j<scans[i]->line_matches.size();j++)
			{ // for each line observation;
				bool not_observed=true;
				for(size_t k=0;k<line_map_.size();k++)
				{
					if(line_map_[k]->isObserved(scans[i]->line_matches[j].ref))
					{
						fp<<"\nobserved in line_map_["<<k<<"] with "<<line_map_[k]->lines.size()<<" observations"<<std::endl;
						fp<<"\t"<<"scan_id "<<scans[i]->id<<"\tline_id "<<scans[i]->line_matches[j].cur->id<<std::endl;
						line_map_[k]->addObservation(scans[i]->line_matches[j].cur,scans[i]->Tcg.inv());
						*cursor_feature++=line_map_[k]->id;
						*cursor_camera++=i+1;
						*cursor++=scans[i]->line_matches[j].cur->u(0);
						*cursor++=scans[i]->line_matches[j].cur->u(1);
						*cursor++=scans[i]->line_matches[j].cur->u(2);
						*cursor++=scans[i]->line_matches[j].cur->v(0);
						*cursor++=scans[i]->line_matches[j].cur->v(1);
						*cursor++=scans[i]->line_matches[j].cur->v(2);
//						fp<<scans[i]->line_matches[j].cur->sqrt_info<<std::endl;
						memcpy(cursor_info,scans[i]->line_matches[j].cur->sqrt_info.data(),36*sizeof(double));
//						fp<<"\t"<<"sqrt_information "<<std::endl
//						  <<"\t"<<cursor_info[0]<<"\t"<<cursor_info[1]<<"\t"<<cursor_info[2]
//						  <<"\t"<<cursor_info[3]<<"\t"<<cursor_info[4]<<"\t"<<cursor_info[5]<<std::endl    
//						  <<"\t"<<cursor_info[6]<<"\t"<<cursor_info[7]<<"\t" <<cursor_info[8]
//						  <<"\t"<<cursor_info[9]<<"\t"<<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl    
//						  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]
//						  <<"\t"<<cursor_info[15]<<"\t"<<cursor_info[16]<<"\t"<<cursor_info[17]<<std::endl    
//						  <<"\t"<<cursor_info[18]<<"\t"<<cursor_info[19]<<"\t"<<cursor_info[20]
//						  <<"\t"<<cursor_info[21]<<"\t"<<cursor_info[22]<<"\t"<<cursor_info[23]<<std::endl    
//						  <<"\t"<<cursor_info[24]<<"\t"<<cursor_info[25]<<"\t"<<cursor_info[26]
//						  <<"\t"<<cursor_info[27]<<"\t"<<cursor_info[28]<<"\t"<<cursor_info[29]<<std::endl    
//						  <<"\t"<<cursor_info[30]<<"\t"<<cursor_info[31]<<"\t"<<cursor_info[32]
//						  <<"\t"<<cursor_info[33]<<"\t"<<cursor_info[34]<<"\t"<<cursor_info[35]<<std::endl    
//						  <<std::endl;
						cursor_info+=36;
						not_observed=false;
						break;
					}
				}
				if(not_observed)
				{
					fp<<"\nadd a new line landmark\tscan_id "<<scans[i]->id<<"\tline_id "<<scans[i]->line_matches[j].cur->id<<std::endl;
					LineLM *line_lm=new LineLM(scans[i]->line_matches[j].cur,
											   scans[i]->Tcg.inv());
					line_lm->addObservation(scans[i]->line_matches[j].cur,scans[i]->Tcg.inv());
					line_lm->id=line_map_.size();
					line_map_.push_back(line_lm);

					*cursor_feature++=line_lm->id;
					*cursor_camera++=i+1;
					*cursor++=scans[i]->line_matches[j].cur->u(0);
					*cursor++=scans[i]->line_matches[j].cur->u(1);
					*cursor++=scans[i]->line_matches[j].cur->u(2);
					*cursor++=scans[i]->line_matches[j].cur->v(0);
					*cursor++=scans[i]->line_matches[j].cur->v(1);
					*cursor++=scans[i]->line_matches[j].cur->v(2);
//					fp<<scans[i]->line_matches[j].cur->sqrt_info<<std::endl;
					memcpy(cursor_info,scans[i]->line_matches[j].cur->sqrt_info.data(),36*sizeof(double));
//					fp<<"\t"<<"sqrt_information "<<std::endl
//					  <<"\t"<<cursor_info[0]<<"\t"<<cursor_info[1]<<"\t"<<cursor_info[2]
//					  <<"\t"<<cursor_info[3]<<"\t"<<cursor_info[4]<<"\t"<<cursor_info[5]<<std::endl    
//					  <<"\t"<<cursor_info[6]<<"\t"<<cursor_info[7]<<"\t" <<cursor_info[8]
//					  <<"\t"<<cursor_info[9]<<"\t"<<cursor_info[10]<<"\t"<<cursor_info[11]<<std::endl    
//					  <<"\t"<<cursor_info[12]<<"\t"<<cursor_info[13]<<"\t"<<cursor_info[14]
//					  <<"\t"<<cursor_info[15]<<"\t"<<cursor_info[16]<<"\t"<<cursor_info[17]<<std::endl    
//					  <<"\t"<<cursor_info[18]<<"\t"<<cursor_info[19]<<"\t"<<cursor_info[20]
//					  <<"\t"<<cursor_info[21]<<"\t"<<cursor_info[22]<<"\t"<<cursor_info[23]<<std::endl    
//					  <<"\t"<<cursor_info[24]<<"\t"<<cursor_info[25]<<"\t"<<cursor_info[26]
//					  <<"\t"<<cursor_info[27]<<"\t"<<cursor_info[28]<<"\t"<<cursor_info[29]<<std::endl    
//					  <<"\t"<<cursor_info[30]<<"\t"<<cursor_info[31]<<"\t"<<cursor_info[32]
//					  <<"\t"<<cursor_info[33]<<"\t"<<cursor_info[34]<<"\t"<<cursor_info[35]<<std::endl    
//					  <<std::endl;
					cursor_info+=36;
				}
			}
		}

		num_planes_=plane_map_.size();
		num_lines_=line_map_.size();
		num_parameters_=7*num_cameras_+4*num_planes_+6*num_lines_;
		parameters_=new double[num_parameters_];

		// fill parameters_;
		cursor=parameters_;
//		fp<<"\nparameters_"<<std::endl;
		{ // scan_ref
			Transform Tcg=scans[0]->scan_ref->Tcg;
			Eigen::Quaterniond quat=Tcg.Quat();
			*cursor++=quat.w();
			*cursor++=quat.x();
			*cursor++=quat.y();
			*cursor++=quat.z();
			*cursor++=Tcg.t(0);
			*cursor++=Tcg.t(1);
			*cursor++=Tcg.t(2);
//			fp<<quat.w()<<"\t"<<quat.vec().transpose()<<"\t"<<Tcg.t.transpose()<<std::endl;
		}
		for(size_t i=0;i<scans.size();i++)
		{
			Transform Tcg=scans[i]->Tcg;
			Eigen::Quaterniond quat=Tcg.Quat();
			*cursor++=quat.w();
			*cursor++=quat.x();
			*cursor++=quat.y();
			*cursor++=quat.z();
			*cursor++=Tcg.t(0);
			*cursor++=Tcg.t(1);
			*cursor++=Tcg.t(2);
//			fp<<quat.w()<<"\t"<<quat.vec().transpose()<<"\t"<<Tcg.t.transpose()<<std::endl;
		}
		for(size_t i=0;i<plane_map_.size();i++)
		{
			*cursor++=plane_map_[i]->pi(0);
			*cursor++=plane_map_[i]->pi(1);
			*cursor++=plane_map_[i]->pi(2);
			*cursor++=plane_map_[i]->pi(3);
//			fp<<plane_map_[i]->id<<"\t"<<plane_map_[i]->pi(0)<<"\t"<<plane_map_[i]->pi(1)<<"\t"<<plane_map_[i]->pi(2)<<"\t"<<plane_map_[i]->pi(3)<<std::endl;
		}
		for(size_t i=0;i<line_map_.size();i++)
		{
			*cursor++=line_map_[i]->L(0);
			*cursor++=line_map_[i]->L(1);
			*cursor++=line_map_[i]->L(2);
			*cursor++=line_map_[i]->L(3);
			*cursor++=line_map_[i]->L(4);
			*cursor++=line_map_[i]->L(5);
//			fp<<line_map_[i]->id<<"\t"
//			   <<line_map_[i]->L(0)<<"\t"<<line_map_[i]->L(1)<<"\t"<<line_map_[i]->L(2)<<"\t"
//			   <<line_map_[i]->L(3)<<"\t"<<line_map_[i]->L(4)<<"\t"<<line_map_[i]->L(5)<<std::endl;
		}

		fp<<"num_observations_ = "<<num_observations_<<std::endl;
		fp<<"num_observ_planes_ = "<<num_observ_planes_<<std::endl;
		fp<<"num_observ_lines_ = "<<num_observ_lines_<<std::endl<<std::endl;
		fp<<"num_parameters_ = "<<num_parameters_<<std::endl;
		fp<<"num_cameras_ = "<<num_parameters_<<std::endl;
		fp<<"num_planes_ = "<<num_planes_<<std::endl;
		fp<<"num_lines_ = "<<num_lines_<<std::endl<<std::endl;

		fp<<"\nplane_map_"<<std::endl;
		for(size_t i=0;i<plane_map_.size();i++)
		{
			fp<<plane_map_[i]->id<<"\t"<<plane_map_[i]->pi(0)<<"\t"<<plane_map_[i]->pi(1)<<"\t"<<plane_map_[i]->pi(2)<<"\t"<<plane_map_[i]->pi(3)<<std::endl;
			for(size_t j=0;j<plane_map_[i]->planes.size();j++)
			{
				fp<<"\t"<<plane_map_[i]->cameras[j].t.transpose()<<"\t"<<plane_map_[i]->planes[j]->id<<std::endl;
			}
		}

		fp<<"\nline_map_"<<std::endl;
		for(size_t i=0;i<line_map_.size();i++)
		{
			fp<<line_map_[i]->id<<"\t"
			  <<line_map_[i]->L(0)<<"\t"<<line_map_[i]->L(1)<<"\t"<<line_map_[i]->L(2)<<"\t"
			  <<line_map_[i]->L(3)<<"\t"<<line_map_[i]->L(4)<<"\t"<<line_map_[i]->L(5)<<std::endl;
			for(size_t j=0;j<line_map_[i]->lines.size();j++)
			{
				fp<<"\t"<<line_map_[i]->cameras[j].t.transpose()<<"\t"<<line_map_[i]->lines[j]->id<<std::endl;
			}
		}

		cursor=observations_;
		cursor_info=sqrt_information_;
		fp<<"\nplane observations_ "<<std::endl;
		for(size_t i=0;i<num_observ_planes_;i++)
		{
			fp<<i<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<std::endl;
//			for(size_t j=0;j<16;j++)
//			{
//				fp<<"\t"<<*cursor_info++;
//			}
//			fp<<std::endl;
		}
		fp<<"\nline observations_ "<<std::endl;
		for(size_t i=0;i<num_observ_lines_;i++)
		{
			fp<<i<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<std::endl;
//			for(size_t j=0;j<36;j++)
//			{
//				fp<<"\t"<<*cursor_info++;
//			}
//			fp<<std::endl;
		}

		cursor_camera=camera_index_;
		cursor_feature=feature_index_;
		fp<<"\nfeature_index_"<<std::endl;
		for(size_t i=0;i<num_observ_planes_;i++)
		{
			fp<<i<<"\t"<<*cursor_feature++<<std::endl;
		}
		for(size_t i=0;i<num_observ_lines_;i++)
		{
			fp<<i+num_observ_planes_<<"\t"<<*cursor_feature++<<std::endl;
		}

		fp<<"\ncamera_index_"<<std::endl;
		for(size_t i=0;i<num_observations_;i++)
		{
			fp<<i<<"\t"<<*cursor_camera++<<std::endl;
		}

		cursor=parameters_;
		fp<<"\nparameters_"<<std::endl;
		for(size_t i=0;i<num_cameras_;i++)
		{
			fp<<i<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<std::endl;
		}
		for(size_t i=0;i<num_planes_;i++)
		{
			fp<<i<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<std::endl;
		}
		for(size_t i=0;i<num_lines_;i++)
		{
			fp<<i<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<"\t";
			fp<<*cursor++<<std::endl;
		}

		fp.close();
	}*/

	PlaneLineBA::~PlaneLineBA()
	{
//		delete []feature_index_;
//		delete []camera_index_;
//		delete []observations_;
//		delete []sqrt_information_;
//		delete []parameters_;
//		delete []observ_shadows_;
//		delete []sqrt_information_shadows_;
//		delete []shadow_camera_index_;
//		delete []shadow_plane_index_;
//		delete []shadow_line_index_;

//		for(size_t i=0;i<plane_map_.size();i++)
//		{ delete plane_map_[i]; }

//		for(size_t i=0;i<line_map_.size();i++)
//		{ delete line_map_[i]; }
	}

	bool PlaneErrorCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobian) const
	{
		std::ofstream fp;
		fp.open("plane_error.txt",std::ios::app);

		Eigen::Quaterniond q_cg(parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
		Eigen::Vector3d t_cg(parameters[0][4],parameters[0][5],parameters[0][6]);
		Eigen::Map<const Eigen::Vector4d> pi(parameters[1]);
		Eigen::Map<Eigen::Vector4d> residual(residuals);
		fp<<"q_cg "<<q_cg.w()<<"\t"<<q_cg.vec().transpose()<<std::endl;
		fp<<"t_cg "<<t_cg.transpose()<<std::endl;
		fp<<"pi "<<pi.transpose()<<std::endl;

		Eigen::Matrix3d R_cg=q_cg.toRotationMatrix();
		Eigen::Matrix4d T_cg; // transform for planes;
		T_cg.setIdentity();
		T_cg.block<3,3>(0,0)=R_cg;
		T_cg.block<1,3>(3,0)=-t_cg.transpose()*R_cg;

		Eigen::Vector4d nd;
		double n_norm=pi.block<3,1>(0,0).norm();
		nd=pi/n_norm;

		residual = observed_pi_-T_cg*nd;
		residual.applyOnTheLeft(sqrt_information_);
		fp<<"T_cg"<<std::endl<<T_cg<<std::endl;
		fp<<"sqrt_information_"<<std::endl<<sqrt_information_<<std::endl;
		fp<<"observed_pi_ "<<observed_pi_.transpose()<<std::endl;
		fp<<"residual "<<residual.transpose()<<std::endl;

		if(jacobian)
		{
			fp<<"jacobian!=NULL"<<std::endl;
			for(size_t i=0;i<28;i++) jacobian[0][i]=0.1;
			for(size_t i=0;i<16;i++) jacobian[1][i]=0.1;
//				Eigen::Map<Eigen::Matrix<double,4,7,Eigen::RowMajor> > jacobian_pose(jacobian[0]);
//				Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor> > jacobian_plane(jacobian[1]);
		}

		return true;
	}

	bool LineLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
	{
//		std::ofstream fp;
//		fp.open("aaa.txt",std::ios::app);

		Eigen::Map<const Eigen::Vector3d> u(x);
		Eigen::Map<const Eigen::Vector3d> v(x+3);
		Eigen::Map<const Eigen::Vector3d> vec_theta(delta);
		const double theta=delta[3];
//		fp<<"u="<<u.transpose()<<std::endl;
//		fp<<"v="<<v.transpose()<<std::endl;
//		fp<<"vec_theta="<<vec_theta.transpose()<<std::endl;
//		fp<<"theta="<<theta<<std::endl;

		Eigen::Matrix3d U;
		Eigen::Matrix2d W;
		Eigen::Vector3d uxv=u.cross(v);
		double sigma_1=u.norm();
		double sigma_2=v.norm();
		double sigma_norm=sqrt(sigma_1*sigma_1+sigma_2*sigma_2);

		U.block<3,1>(0,0)=u/sigma_1;
		U.block<3,1>(0,1)=v/sigma_2;
		U.block<3,1>(0,2)=uxv/uxv.norm();
		W(0,0)=sigma_1/sigma_norm;
		W(0,1)=-sigma_2/sigma_norm;
		W(1,0)=sigma_2/sigma_norm;
		W(1,1)=sigma_1/sigma_norm;
//		fp<<"U="<<std::endl<<U<<std::endl;
//		fp<<"W="<<std::endl<<W<<std::endl;

		Eigen::Matrix3d R_vec_theta;
		Eigen::Matrix2d R_theta;
		Eigen::AngleAxisd angle_axis(vec_theta.norm(),vec_theta/vec_theta.norm());
		R_vec_theta=angle_axis.toRotationMatrix();
		if(vec_theta.norm()<1e-7)
		{
			R_vec_theta.setIdentity();
		}
		R_theta(0,0)=cos(theta);
		R_theta(0,1)=-sin(theta);
		R_theta(1,0)=sin(theta);
		R_theta(1,1)=cos(theta);
//		fp<<"R_vec_theta="<<std::endl<<R_vec_theta<<std::endl;
//		fp<<"R_theta="<<std::endl<<R_theta<<std::endl;

		U.applyOnTheRight(R_vec_theta);
		W.applyOnTheRight(R_theta);
		double w11=W(0,0);
		double w21=W(1,0);
		Eigen::Vector3d u1=U.block<3,1>(0,0);
		Eigen::Vector3d u2=U.block<3,1>(0,1);
		Eigen::Vector3d u3=U.block<3,1>(0,2);

		Eigen::Map<Eigen::Vector3d> u_update(x_plus_delta);
		Eigen::Map<Eigen::Vector3d> v_update(x_plus_delta+3);
		u_update=w11*u1;
		v_update=w21*u2;
//		u_update/=v_update.norm();
//		v_update.normalize();
//		fp<<"u_update="<<u_update.transpose()<<std::endl;
//		fp<<"v_update="<<v_update.transpose()<<std::endl;
		
		return true;
	}

	bool LineLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
	{
		Eigen::Map<const Eigen::Vector3d> u(x);
		Eigen::Map<const Eigen::Vector3d> v(x+3);
		Eigen::Map<Eigen::Matrix<double,6,4,Eigen::RowMajor> > J(jacobian);

		double w11=u.norm();
		double w21=v.norm();
		Eigen::Vector3d u1=u/w11;
		Eigen::Vector3d u2=v/w21;
		Eigen::Vector3d u3=u1.cross(u2);

		J.setZero();
		J.block<3,1>(3,0)=w21*u3;
		J.block<3,1>(0,1)=-w11*u3;
		J.block<3,1>(0,2)=w11*u2;
		J.block<3,1>(3,2)=-w21*u1;
		J.block<3,1>(0,3)=-w21*u1;
		J.block<3,1>(3,3)=w11*u2;

		return true;
	}

//	bool MotionEstimation::alignOccludingLines(Scan *scan, Transform &Tcr)
//	{
//		fp.open("motion_estimation.txt",std::ios::app);
//		if(debug) 
//			fp<<std::endl<<"alignOccludingLines *******************************************"<<std::endl;
//
//		// weighting factor of the line and plane parts in cost function;
//		double alpha_l=1,alpha_pi=1;
//		
//		Eigen::Matrix4d Ai, M;
//		M.setZero();
//		for(size_t i=0;i<scan->line_matches.size();i++)
//		{
//			Eigen::Vector3d vc=scan->line_matches[i].cur->v;
//			Eigen::Vector3d vr=scan->line_matches[i].ref->v;
//			Eigen::Matrix<double,1,1> vcTvr=vc.transpose()*vr;
//			if(vcTvr(0,0)<0)
//			{
//				scan->line_matches[i].cur->v=-scan->line_matches[i].cur->v;
//				scan->line_matches[i].cur->u=-scan->line_matches[i].cur->u;
//				vc=scan->line_matches[i].cur->v;
//			}
//			Eigen::Vector3d tmp=vr-vc;
//			Ai.setZero();
//			Ai.block<1,3>(0,1)=tmp.transpose();
//			Ai.block<3,1>(1,0)=-tmp;
//			tmp=vr+vc;
//			Ai.block<3,3>(1,1)=Transform::skew_sym(tmp);
//			M+=Ai.transpose()*Ai;
//		}
//
//		Eigen::Matrix4d Bi, M_pi;
//		M_pi.setZero();
//		for(size_t i=0;i<scan->plane_matches.size();i++)
//		{
//			Eigen::Vector3d nc=scan->plane_matches[i].cur->n;
//			Eigen::Vector3d nr=scan->plane_matches[i].ref->n;
//			Eigen::Vector3d tmp=nr-nc;
//			Bi.setZero();
//			Bi.block<1,3>(0,1)=tmp.transpose();
//			Bi.block<3,1>(1,0)=-tmp;
//			tmp=nr+nc;
//			Bi.block<3,3>(1,1)=Transform::skew_sym(tmp);
//			M_pi+=Bi.transpose()*Bi;
//		}
//
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es;
//		es.compute(alpha_l*M+alpha_pi*M_pi);
//		Eigen::Vector4d eigenvalues=es.eigenvalues();
//		Eigen::Matrix4d eigenvectors=es.eigenvectors();
//		if(debug)
//		{
//			fp<<"eigenvalues="<<eigenvalues.transpose()<<std::endl;
//			fp<<"eigenvectors="<<std::endl<<eigenvectors<<std::endl<<std::endl;
//		}
//		
//		double ev_min=DBL_MAX;
//		int idx=-1;
//		for(size_t j=0;j<4;j++)
//		{
//			if(eigenvalues(j)<ev_min)
//			{
//				ev_min=eigenvalues(j);
//				idx=j;
//			}
//		}
//		Eigen::Vector4d quat=eigenvectors.block<4,1>(0,idx);
//		Eigen::Quaterniond qcr(quat(0),quat(1),quat(2),quat(3));
//		Tcr.R=qcr.toRotationMatrix();
//		if(debug)
//		{
//			fp<<"qcr="<<qcr.w()<<", "<<qcr.x()<<", "<<qcr.y()<<", "<<qcr.z()<<std::endl;
//			fp<<"Rcr="<<std::endl<<Tcr.R<<std::endl;
//			fp<<"det(Rcr)="<<Tcr.R.determinant()<<std::endl<<std::endl;
//		}
//
//		Eigen::Matrix3d A;
//		Eigen::Vector3d b;
//		for(size_t i=0;i<scan->line_matches.size();i++)
//		{
//			Eigen::Vector3d vc=scan->line_matches[i].cur->v;
//			Eigen::Vector3d vr=scan->line_matches[i].ref->v;
//			Eigen::Vector3d tmp=Tcr.R*vr;
//			Eigen::Matrix3d tmp_mat=Transform::skew_sym(tmp);
//			Eigen::Vector3d uc=scan->line_matches[i].cur->u;
//			Eigen::Vector3d ur=scan->line_matches[i].ref->u;
//			tmp=uc-Tcr.R*ur;
//			A+=tmp_mat.transpose()*tmp_mat;
//			b+=-tmp_mat.transpose()*tmp;
//		}
//
//		Eigen::Matrix3d A_pi;
//		Eigen::Vector3d b_pi;
//		for(size_t i=0;i<scan->plane_matches.size();i++)
//		{
//			Eigen::Vector3d nc=scan->plane_matches[i].cur->n;
//			Eigen::Vector3d nr=scan->plane_matches[i].ref->n;
//			Eigen::Vector3d tmp=Tcr.R*nr;
//			Eigen::Matrix3d tmp_mat=tmp*tmp.transpose();
//			double dc=scan->plane_matches[i].cur->d;
//			double dr=scan->plane_matches[i].ref->d;
//			tmp=tmp*(dc-dr);
//			A_pi+=tmp_mat;
//			b_pi+=-tmp;
//		}
//
//		A=alpha_l*A+alpha_pi*A_pi;
//		b=alpha_l*b+alpha_pi*b_pi;
//
//		Eigen::Matrix3d A_inv;
//		double det;
//		bool invertible;
//		A.computeInverseAndDetWithCheck(A_inv,det,invertible);
//		if(debug)
//		{
//			fp<<"A="<<std::endl<<A<<std::endl;
//			fp<<"b="<<b.transpose()<<std::endl<<std::endl;
//			if(invertible)
//			{
//				fp<<"matrix A invertible: "<<det<<std::endl<<A_inv<<std::endl<<std::endl;
//			}
//			else
//			{
//				fp<<"matrix A not invertible."<<std::endl<<std::endl;
//			}
//		}
//		if(invertible)
//		{
////			Tcr.t.setZero();
//			Tcr.t=A_inv*b;
//			if(debug)
//			{
//				fp<<"tcr="<<Tcr.t.transpose()<<std::endl;
//				double error_pre=0, error=0;
//				for(size_t i=0;i<scan->line_matches.size();i++)
//				{
//					Eigen::Vector3d vc=scan->line_matches[i].cur->v;
//					Eigen::Vector3d vr=scan->line_matches[i].ref->v;
//					Eigen::Vector3d tmp=Tcr.R*vr;
//					Eigen::Matrix3d tmp_mat=Transform::skew_sym(tmp);
//					Eigen::Vector3d uc=scan->line_matches[i].cur->u;
//					Eigen::Vector3d ur=scan->line_matches[i].ref->u;
//					tmp=uc-Tcr.R*ur;
//					error_pre+=tmp.norm();
//					tmp=tmp+tmp_mat*Tcr.t;
//					error+=tmp.norm();
//				}
//				fp<<"before tcr\t"<<error_pre<<std::endl;
//				fp<<"after tcr \t"<<error<<std::endl;
//			}
//			fp.close();
//			return true;
//		}
//		else
//		{
//			std::cout<<"aligning occluding lines failure!";
//			fp.close();
//			return false;
//		}
//	}

	bool MotionEstimation::estimate(Map *map)
	{
		debug=true;
		if(debug) 
		{
			fp.open("motion_estimation.txt",std::ios::app);
			fp<<std::endl<<"estimate*******************************************"<<std::endl;
		}

//		int num_frames=3;
//		if(map->cameras.size()<num_frames) num_frames=map->cameras.size();

		PlaneLineBA *plane_line_ba=new PlaneLineBA(map);//,num_frames,use_plane_,use_line_,use_shadow_);
		ceres::Problem problem;
		buildProblem(plane_line_ba,&problem);
//		if(debug)
//		fp<<std::fixed<<scan->time()
//		  <<"\t"<<plane_line_ba->num_cameras()<<"\t"<<plane_line_ba->num_planes()<<"\t"<<plane_line_ba->num_lines()
//		  <<"\t"<<plane_line_ba->num_observ_planes()<<"\t"<<plane_line_ba->num_observ_lines()<<std::endl;
//		  <<"\t"<<plane_line_ba->num_observ_shadows()<<std::endl;
		
//		displayPlaneLineBA(scan,plane_line_ba,vis);
//		vis->spin();

		ceres::Solver::Options options;
		options.minimizer_progress_to_stdout=true;
		options.max_num_iterations=max_iterations;
		options.linear_solver_type=ceres::DENSE_SCHUR;
//		options.trust_region_strategy_type=ceres::DOGLEG;
//		options.gradient_tolerance=1e-16;
//		options.function_tolerance=1e-16;
		ceres::Solver::Summary summary;
		Solve(options,&problem,&summary);
//		std::cout<<summary.FullReport()<<"\n";
		
//		const int idx_camera_initial=map->cameras.size()-num_frames;
//		const int camera_block_size=plane_line_ba->camera_block_size();
//		double* cameras=plane_line_ba->mutable_cameras();
//		double *camera=cameras; // first frame;
//		Eigen::Quaterniond quat_rg(camera[0],camera[1],camera[2],camera[3]);
//		Eigen::Vector3d t_tmp_rg(camera[4],camera[5],camera[6]);
//		ulysses::Transform Trg=Transform(quat_rg,t_tmp_rg);
//		double *camera2=cameras+camera_block_size; // second frame;
//		Eigen::Quaterniond quat_cg(camera2[0],camera2[1],camera2[2],camera2[3]);
//		Eigen::Vector3d t_tmp_cg(camera2[4],camera2[5],camera2[6]);
//		ulysses::Transform Tcg=Transform(quat_cg,t_tmp_cg);
//		Tcr=Tcg*Trg.inv();

		/* the followings are updates of the map */
		/*****************************************/
//		if(debug) fp<<"\nafter optimization\n";
		const int camera_block_size=plane_line_ba->camera_block_size();
		double* cameras=plane_line_ba->mutable_cameras();
		if(debug) fp<<"\nBA cameras\n";
		for(size_t i=0;i<plane_line_ba->num_cameras();i++)
		{
			double* camera=cameras+camera_block_size*i;
			if(debug) fp<<"\t"<<camera[0]<<"\t"<<camera[1]<<"\t"<<camera[2]<<"\t"<<camera[3]
						  <<"\t"<<camera[4]<<"\t"<<camera[5]<<"\t"<<camera[6]<<std::endl;
			Eigen::Quaterniond quat(camera[0],camera[1],camera[2],camera[3]);
			Eigen::Vector3d t_tmp(camera[4],camera[5],camera[6]);
//			map->cameras[idx_camera_initial+i]=Transform(quat,t_tmp);
			double t=plane_line_ba->timestamps(i);
			map->updateCamera(t,Transform(quat,t_tmp));
		}
//		Tcr=map->cameras[map->cameras.size()-1]*map->cameras[map->cameras.size()-2].inv();
		/*****************************************/

//		Transform Tcr_gt=scan->Tcw*scan->scan_ref->Tcw.inv();
//		Vector6d delta_xi=Tcr_gt.getMotionVector()-Tcr.getMotionVector();
//		fp<<"\nconstraint\n";
//		Matrix6d Psi;
//		plane_line_ba->computePsi(1,Psi);
//		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
//		es.compute(Psi);
//		Vector6d eigenvalues=es.eigenvalues();
//		Matrix6d eigenvectors=es.eigenvectors();
//		Vector6d eg_val;
//		double sum=0;
//		for(int i=0;i<6;i++)
//		{
//			eg_val(i)=sqrt(fabs(eigenvalues(i)));
//			sum+=eg_val(i);
//		}
//		eg_val/=sum;
////		delta_xi.normalize();
//		for(int i=0;i<6;i++)
//		{
//			double delta=fabs(delta_xi.transpose()*eigenvectors.block<6,1>(0,i));
//			fp<<delta<<"\t"<<eg_val(i)<<std::endl;
//		}
//		fp<<std::endl;
		

		// for debugging
//		observ_planes=plane_line_ba->num_observ_planes();
//		observ_lines=plane_line_ba->num_observ_lines();
//		observ_shadows=plane_line_ba->num_observ_shadows();

		const int plane_block_size=plane_line_ba->plane_block_size();
		const int line_block_size=plane_line_ba->line_block_size();
//		int* idx_map=plane_line_ba->feature_index_map();

		double* planes=plane_line_ba->mutable_planes();
		if(debug) fp<<"\nBA planes\n";
		for(size_t i=0;i<plane_line_ba->num_planes();i++)
		{
			double* plane=planes+plane_block_size*i;
			std::string id=plane_line_ba->features_id(i);
			Landmark* lm=map->findLandmark(id);
			memcpy(lm->planelm()->pi.data(),plane,plane_block_size*sizeof(double));
//			map->planes[*idx_map]->pi(0)=plane[0];
//			map->planes[*idx_map]->pi(1)=plane[1];
//			map->planes[*idx_map]->pi(2)=plane[2];
//			map->planes[*idx_map]->pi(3)=plane[3];
//			idx_map++;
			if(debug) fp<<"\t"<<plane[0]<<"\t"<<plane[1]<<"\t"<<plane[2]<<"\t"<<plane[3]<<std::endl;
		}

		double* lines=plane_line_ba->mutable_lines();
		if(debug) fp<<"\nBA lines\n";
		for(size_t i=0;i<plane_line_ba->num_lines();i++)
		{
			double* line=lines+line_block_size*i;
			std::string id=plane_line_ba->features_id(i+plane_line_ba->num_planes());
			Landmark* lm=map->findLandmark(id);
			memcpy(lm->linelm()->L.data(),line,line_block_size*sizeof(double));
//			map->lines[*idx_map]->L(0)=line[0];
//			map->lines[*idx_map]->L(1)=line[1];
//			map->lines[*idx_map]->L(2)=line[2];
//			map->lines[*idx_map]->L(3)=line[3];
//			map->lines[*idx_map]->L(4)=line[4];
//			map->lines[*idx_map]->L(5)=line[5];
//			idx_map++;
			if(debug) fp<<"\t"<<line[0]<<"\t"<<line[1]<<"\t"<<line[2]<<"\t"<<line[3]<<"\t"<<line[4]<<"\t"<<line[5]<<std::endl;
		}

//		if(debug) fp<<"\nmap->cameras"<<std::endl;
//		for(size_t i=0;i<map->cameras.size();i++)
//		{
//			Eigen::Quaterniond q=map->cameras[i].Quat();
//			if(debug) fp<<"\t"<<i<<"\t"<<q.w()<<"\t"<<q.vec().transpose()<<"\t"<<map->cameras[i].t.transpose()<<std::endl;
//		}
//
//		if(debug) fp<<"\nmap->planes"<<std::endl;
//		for(size_t i=0;i<map->planes.size();i++)
//		{
//			if(debug) fp<<"\t"<<i<<"\t"<<map->planes[i]->pi.transpose()<<std::endl;
//		}
//
//		if(debug) fp<<"\nmap->lines"<<std::endl;
//		for(size_t i=0;i<map->lines.size();i++)
//		{
////			if(debug) fp<<"\t"<<i<<"\t"<<map->lines[i]->L.transpose()<<std::endl;
//			Vector6d l=map->lines[i]->L;
//			l=l/l.block<3,1>(3,0).norm();
//			if(debug) fp<<"\t"<<i<<"\t"<<l.transpose()<<std::endl;
//		}

//		if(debug)
//		{
//			fp<<"*********************************************************"<<std::endl;
//			fp<<"plane residual after optimization"<<std::endl<<std::endl;
//
//			double* observations=plane_line_ba->observations();
//			double* sqrt_information=plane_line_ba->sqrt_information();
//
//			for(size_t i=0;i<plane_line_ba->num_observ_planes();i++)
//			{
//				// each residual block takes a plane/line and a camera as input
//				// and outputs a 4/6 dimensional residual;
//	//			Eigen::Vector4d pi(observations[i*4],observations[i*4+1],observations[i*4+2],observations[i*4+3]);
//				Eigen::Vector4d pi(observations);
//				observations+=plane_block_size;
//				Eigen::Matrix4d information(sqrt_information);
//				sqrt_information+=plane_block_size*plane_block_size;
//
//				double* camera=cameras+camera_block_size*plane_line_ba->camera_index()[i];
//				double* plane=planes+plane_block_size*plane_line_ba->feature_index()[i];
//
//				// for debugging
//				Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
//				Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
//				Eigen::Map<const Eigen::Vector4d> p(plane);
//				Transform Tcg(q_cg,t_cg);
//				Eigen::Vector4d ppi=Tcg.getPlaneTransform()*p;
//				Eigen::Vector4d delta=ppi-pi;
//				Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
//				Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
//				if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
//				if(debug) fp<<ppi.transpose()<<std::endl;
//				if(debug) fp<<pi.transpose()<<std::endl<<std::endl;
//			}
//			fp<<"---------------------------------------------------------"<<std::endl;
//			fp<<"line residual after optimization"<<std::endl<<std::endl;
//
//			for(size_t i=0;i<plane_line_ba->num_observ_lines();i++)
//			{
//				Vector6d L(observations);
//				observations+=line_block_size;
//				Matrix6d information(sqrt_information);
//				sqrt_information+=line_block_size*line_block_size;
//
//				double* camera=cameras+camera_block_size*plane_line_ba->camera_index()[i+plane_line_ba->num_observ_planes()];
//				double* line=lines+line_block_size*plane_line_ba->feature_index()[i+plane_line_ba->num_observ_planes()];
//
//				Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
//				Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
//				Eigen::Map<const Vector6d> l(line);
//				Transform Tcg(q_cg,t_cg);
//				Vector6d l_pi=Tcg.getLineTransform()*l;
//				Vector6d delta=l_pi-L;
//				Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
//				Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
//				if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
//				fp<<delta.transpose()<<std::endl;
//				fp<<l_pi.transpose()<<std::endl;
//				fp<<L.transpose()<<std::endl<<std::endl;
//				fp<<information<<std::endl<<std::endl;
//			}
////			fp<<"---------------------------------------------------------"<<std::endl;
////			fp<<"shadow residual after optimization"<<std::endl<<std::endl;
////
////			double* observ_shadows=plane_line_ba->observ_shadows();
////			double* sqrt_information_shadows=plane_line_ba->sqrt_information_shadows();
////			for(size_t i=0;i<plane_line_ba->num_observ_shadows();i++)
////			{
////				// each residual block takes a plane/line and a camera as input
////				// and outputs a 4/6 dimensional residual;
////				Vector6d L(observ_shadows);
////				observ_shadows+=line_block_size;
////				Matrix6d information(sqrt_information_shadows);
////				sqrt_information_shadows+=line_block_size*line_block_size;
////
////				double* camera=cameras+camera_block_size*plane_line_ba->shadow_camera_index()[i];
////				double* plane=planes+plane_block_size*plane_line_ba->shadow_plane_index()[i];
////				double* line=lines+line_block_size*plane_line_ba->shadow_line_index()[i];
////
////				Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
////				Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
////				Eigen::Map<const Eigen::Vector4d> pi(plane);
////				Eigen::Map<const Vector6d> l(line);
////				Transform Tcg(q_cg,t_cg);
////				Vector6d l_pi=computeLineShadow(l,pi,Tcg);
////				Vector6d delta=l_pi-L;
////				Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
////				Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
////				if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
////				fp<<l_pi.transpose()<<std::endl;
////				fp<<L.transpose()<<std::endl<<std::endl;
////			}
////			fp<<"*********************************************************"<<std::endl;
//		}



//		if(debug)
//		{
//			std::cout<<std::fixed<<scan->scan_ref->time_stamp<<" "
//					 <<map->cameras[map->cameras.size()-2].t.transpose()<<" "
//					 <<map->cameras[map->cameras.size()-2].Quaternion().transpose()<<std::endl;
//			std::cout<<std::fixed<<scan->time_stamp<<" "
//					 <<map->cameras[map->cameras.size()-1].t.transpose()<<" "
//					 <<map->cameras[map->cameras.size()-1].Quaternion().transpose()<<std::endl;
//			visScans(scan,scan->scan_ref,ulysses::Transform(),vis);
//			std::cout<<"initial pose"<<std::endl;
//			vis->spin();
//			visScans(scan,scan->scan_ref,Tcr,vis);
//			std::cout<<"optimized pose"<<std::endl;
//			vis->spin();
//		}

//		boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("Viewer"));
//		vis->setBackgroundColor (0, 0, 0);
//		vis->initCameraParameters ();
//		vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());
//
//		fp<<"\tdistance to occluded lines\n";
//		int idx_camera=map->cameras.size()-1; // current frame;
//		for(int i=0;i<map->indices_lines[idx_camera].size();i++)
//		{ // line [i] observed in current frame;
//			int idx_line=map->indices_lines[idx_camera][i];
//			for(int j=0;j<map->indices_planes[idx_camera].size();j++)
//			{ // plane [j] observed in current frame;
//				int idx_plane=map->indices_planes[idx_camera][j];
//				// the shadow of line [i] on the plane [j];
//				// described in the world frame;
//				Vector6d L_pi=map->computeLineShadow(idx_camera,idx_plane,idx_line);
//				L_pi=map->cameras[idx_camera].getLineTransform()*L_pi;
//				std::cout<<"line "<<idx_line<<" plane "<<idx_plane<<std::endl;
//				double min=DBL_MAX;
//				int idx=-1;
//				Line *closest_line;
//				for(std::list<Line*>::iterator it=scan->lines_occluded.begin();
//											   it!=scan->lines_occluded.end();it++)
//				{
//					Vector6d L_pi_observe;
//					L_pi_observe.block<3,1>(0,0)=(*it)->u;
//					L_pi_observe.block<3,1>(3,0)=(*it)->v;
//					Vector6d delta=L_pi-L_pi_observe;
//					Eigen::Matrix<double,1,1> tmp=delta.transpose()*(*it)->sqrt_info*delta;
//					double dist=tmp(0,0);
//					if(dist<min)
//					{
//						min=dist;
//						closest_line=*it;
//					}
//					fp<<"\t"<<dist;
//				}
//				fp<<std::endl;
//
//				std::cout<<"min dist = "<<min<<std::endl;
//				displayLineShadow(scan,map,L_pi,closest_line,idx_camera,idx_plane,idx_line,vis);
//				vis->spin();
//			}
//		}
//		for(int i=0;i<map->lines.size();i++)
//		{
//			for(int j=0;j<map->lines[i]->indices_cameras.size();j++)
//			{
//				int idx_camera=map->lines[i]->indices_cameras[j];
//				for(int k=0;k<map->indices_planes[idx_camara].size();k++)
//				{
//					int idx_plane=map->indices_planes[idx_camera][k];
//					map->lines[i] // occlduing line
//					map->planes[idx_plane] // the projecting plane
//				}
//			}
//		}

		if(debug) fp.close();
		delete plane_line_ba;
//		cout<<"deleted plane_line_ba"<<std::endl;
		return true;
	}


	void MotionEstimation::buildProblem(PlaneLineBA* plane_line_ba, ceres::Problem* problem)
	{
		if(debug) fp<<"buildProblem: *******************************************"<<std::endl;
//		fp<<plane_line_ba->num_observ_planes()<<"\t"
//		  <<plane_line_ba->num_observ_lines()<<"\t"
//		  <<plane_line_ba->num_observ_shadows()<<std::endl;

		const int camera_block_size=plane_line_ba->camera_block_size();
		const int plane_block_size=plane_line_ba->plane_block_size();
		const int line_block_size=plane_line_ba->line_block_size();
		double* cameras=plane_line_ba->mutable_cameras();
		double* planes=plane_line_ba->mutable_planes();
		double* lines=plane_line_ba->mutable_lines();

		double* observations=plane_line_ba->observations();
		double* sqrt_information=plane_line_ba->sqrt_information();
//		if(debug) fp<<"plane residual before optimization"<<std::endl<<std::endl;
//		if(false)
		if(debug) fp<<"plane observations"<<endl;
		for(size_t i=0;i<plane_line_ba->num_observ_planes();i++)
		{
			// each residual block takes a plane/line and a camera as input
			// and outputs a 4/6 dimensional residual;
//			Eigen::Vector4d pi(observations[i*4],observations[i*4+1],observations[i*4+2],observations[i*4+3]);
			Eigen::Vector4d pi(observations);
			observations+=plane_block_size;
			Eigen::Matrix4d information(sqrt_information);
			sqrt_information+=plane_block_size*plane_block_size;

			double* camera=cameras+camera_block_size*plane_line_ba->camera_index(i);
			double* plane=planes+plane_block_size*plane_line_ba->feature_index(i);

			if(debug)
			{
				fp<<i<<endl;
				fp<<pi.transpose()<<endl;
				fp<<"camera idx="<<plane_line_ba->camera_index(i)<<endl;
				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
				fp<<"feature idx="<<plane_line_ba->feature_index(i)<<endl;
				for(int j=0;j<plane_block_size;j++) fp<<plane[j]<<" "; fp<<endl;
			}

//			// for debugging
//			Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
//			Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
//			Eigen::Map<const Eigen::Vector4d> p(plane);
//			Transform Tcg(q_cg,t_cg);
//			Eigen::Vector4d ppi=Tcg.getPlaneTransform()*p;
//			Eigen::Vector4d delta=ppi-pi;
//			Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
//			Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
//			if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
//			if(debug) fp<<ppi.transpose()<<std::endl;
//			if(debug) fp<<pi.transpose()<<std::endl<<std::endl;

//			information.setIdentity();
			ceres::CostFunction* cost_function=PlaneError::Create(pi,information);
			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
			problem->AddResidualBlock(cost_function,loss_function,camera,plane);
		}
//		if(debug) fp<<"---------------------------------------------------------"<<std::endl;
//		if(debug) fp<<"line residual before optimization"<<std::endl<<std::endl;

//		if(use_line_residual_)
		if(debug) fp<<"line observations"<<endl;
		for(size_t i=0;i<plane_line_ba->num_observ_lines();i++)
		{
			// each residual block takes a plane/line and a camera as input
			// and outputs a 4/6 dimensional residual;
			Vector6d L(observations);
			observations+=line_block_size;
			Matrix6d information(sqrt_information);
			sqrt_information+=line_block_size*line_block_size;

			double* camera=cameras+camera_block_size*plane_line_ba->camera_index(i+plane_line_ba->num_observ_planes());
			double* line=lines+line_block_size*(plane_line_ba->feature_index(i+plane_line_ba->num_observ_planes())-plane_line_ba->num_planes());

			if(debug)
			{
				fp<<i<<endl;
				fp<<L.transpose()<<endl;
				fp<<"camera idx="<<plane_line_ba->camera_index(i+plane_line_ba->num_observ_planes())<<endl;
				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
				fp<<"feature idx="<<plane_line_ba->feature_index(i+plane_line_ba->num_observ_planes())<<endl;
				for(int j=0;j<line_block_size;j++) fp<<line[j]<<" "; fp<<endl;
			}

//			// for debugging
//			Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
//			Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
//			Eigen::Map<const Vector6d> l(line);
//			Transform Tcg(q_cg,t_cg);
//			Vector6d l_pi=Tcg.getLineTransform()*l;
//			Vector6d delta=l_pi-L;
//			Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
//			Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
//			if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
//			if(debug) fp<<delta.transpose()<<std::endl;
//			if(debug) fp<<l_pi.transpose()<<std::endl;
//			if(debug) fp<<L.transpose()<<std::endl<<std::endl;
//			if(debug) fp<<information<<std::endl<<std::endl;

//			information.setIdentity();
			ceres::CostFunction* cost_function;
			cost_function=LineError::Create(L,information);
			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
			problem->AddResidualBlock(cost_function,loss_function,camera,line);
		}
//		if(debug) fp<<"---------------------------------------------------------"<<std::endl;
//		if(debug) fp<<"shadow residual before optimization"<<std::endl<<std::endl;
//
//		double* observ_shadows=plane_line_ba->observ_shadows();
//		double* sqrt_information_shadows=plane_line_ba->sqrt_information_shadows();
//		for(size_t i=0;i<plane_line_ba->num_observ_shadows();i++)
//		{
//			// each residual block takes a plane/line and a camera as input
//			// and outputs a 4/6 dimensional residual;
//			Vector6d L(observ_shadows);
//			observ_shadows+=line_block_size;
//			Matrix6d information(sqrt_information_shadows);
//			sqrt_information_shadows+=line_block_size*line_block_size;
//
//			double* camera=cameras+camera_block_size*plane_line_ba->shadow_camera_index()[i];
//			double* plane=planes+plane_block_size*plane_line_ba->shadow_plane_index()[i];
//			double* line=lines+line_block_size*plane_line_ba->shadow_line_index()[i];
//
//			// for debugging
//			Eigen::Quaterniond q_cg(camera[0],camera[1],camera[2],camera[3]);
//			Eigen::Vector3d t_cg(camera[4],camera[5],camera[6]);
//			Eigen::Map<const Eigen::Vector4d> pi(plane);
//			Eigen::Map<const Vector6d> l(line);
//			Transform Tcg(q_cg,t_cg);
//			Vector6d l_pi=computeLineShadow(l,pi,Tcg);
//			Vector6d delta=l_pi-L;
//			Eigen::Matrix<double,1,1> error=delta.transpose()*information*information*delta;
//			Eigen::Matrix<double,1,1> error1=delta.transpose()*delta;
//			if(debug) fp<<i<<"\t"<<error<<" "<<error1<<std::endl;
//			if(debug) fp<<l_pi.transpose()<<std::endl;
//			if(debug) fp<<L.transpose()<<std::endl<<std::endl;
//
////			information.setIdentity();
//			ceres::CostFunction* cost_function;
//			cost_function=LineShadowError::Create(L,information);
//			ceres::LossFunction* loss_function=NULL;//new ceres::CauchyLoss(1.0);
//			problem->AddResidualBlock(cost_function,loss_function,camera,plane,line);
//
//		}
		// LocalParameterization for cameras;
		ceres::LocalParameterization* camera_parameterization=
			new ceres::ProductParameterization(new ceres::QuaternionParameterization(),new ceres::IdentityParameterization(3));
		for(size_t i=0;i<plane_line_ba->num_cameras();i++)
		{
			problem->AddParameterBlock(cameras+camera_block_size*i,camera_block_size,camera_parameterization);
			if(i==0) problem->SetParameterBlockConstant(cameras+camera_block_size*i);
		}

		// LocalParameterization for planes;
		ceres::LocalParameterization* plane_parameterization=new ceres::QuaternionParameterization();
		for(size_t i=0;i<plane_line_ba->num_planes();i++)
		{
			problem->AddParameterBlock(planes+plane_block_size*i,plane_block_size,plane_parameterization);
//			double *plane=planes+plane_block_size*i;
//			for(int j=0;j<plane_block_size;j++) fp<<plane[j]<<" "; fp<<endl;
//			if(!use_line_residual_) 
			if(feature_constant) problem->SetParameterBlockConstant(planes+plane_block_size*i);
		}

		// LocalParameterization for lines;
		ceres::LocalParameterization* line_parameterization=new LineLocalParameterization();
		for(size_t i=0;i<plane_line_ba->num_lines();i++)
		{
			problem->AddParameterBlock(lines+line_block_size*i,line_block_size,line_parameterization);
//			if(!use_line_residual_) 
			if(feature_constant) problem->SetParameterBlockConstant(lines+line_block_size*i);
		}
	}

//	Vector6d MotionEstimation::computeLineShadow(Vector6d l, Eigen::Vector4d pi, Transform Tcg)
//	{
//			Transform Tgc=Tcg.inv();
//			Eigen::Vector3d t_gc=Tgc.t;
//
//			Eigen::Vector3d u=l.block<3,1>(0,0);
//			Eigen::Vector3d v=l.block<3,1>(3,0);
//			Eigen::Vector3d n=pi.block<3,1>(0,0);
//			double d=pi(3)/n.norm();
//			n.normalize();
//			
//			// compute the shadow line (described in world frame);
//			Vector6d L_pi;
//			Eigen::Vector3d uu=-d*(Transform::skew_sym(v)*t_gc+u)-n*t_gc.transpose()*u;
//			Eigen::Vector3d vv=-Transform::skew_sym(n)*Transform::skew_sym(v)*t_gc+Transform::skew_sym(u)*n;
//			d=uu.norm()/vv.norm();
//			uu.normalize();
//			vv.normalize();
//			L_pi.block<3,1>(0,0)=d*uu;
//			L_pi.block<3,1>(3,0)=vv;
//
//			double tmp=u.transpose()*uu;
//			if(tmp<0) L_pi=-L_pi;
//
//			// compute 6x6 transform for lines;
//			Matrix6d T_cg=Tcg.getLineTransform(); // transform for lines;
//
//			L_pi=T_cg*L_pi;
//			return L_pi;
//
////		Transform Tgc=Tcg.inv();
////		Eigen::Vector4d tgc;
////		tgc.block<3,1>(0,0)=Tgc.t;
////		tgc(3)=1;
////		Eigen::Vector4d pi_l=Transform::PluckerCoords2DualMat(l)*tgc;
////		Eigen::Matrix4d L_pi_dual=pi*pi_l.transpose()-pi_l*pi.transpose();
////		Vector6d l_pi=Transform::PluckerDualMat2Coords(L_pi_dual);
////		Eigen::Vector3d u=l_pi.block<3,1>(0,0);
////		Eigen::Vector3d v=l_pi.block<3,1>(3,0);
////		double d=u.norm()/v.norm();
////		u.normalize();
////		v.normalize();
////		l_pi.block<3,1>(0,0)=d*u;
////		l_pi.block<3,1>(3,0)=v;
////		return l_pi;
//	}

//	void visScans(Scan *scan, Scan *scan_ref, Transform Tcr, 
//							 boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
//	{
//		using namespace std;
//		char id[20];
//
//		vis->removeAllPointClouds();
//		vis->removeAllShapes();
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//		sprintf(id,"scan_ref");
//		pcl::transformPointCloud(*scan_ref->point_cloud,*plane,Tcr.getMatrix4f());
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, 0,0,255);
//		if (!vis->updatePointCloud (plane, color1, id))
//			vis->addPointCloud (plane, color1, id);
//
//		sprintf(id,"scan_cur");
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud, 255,0,0);
//		if (!vis->updatePointCloud (scan->point_cloud, color2, id))
//			vis->addPointCloud (scan->point_cloud, color2, id);
//	}
}

