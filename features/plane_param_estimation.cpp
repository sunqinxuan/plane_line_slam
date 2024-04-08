/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-10-17 22:13
#
# Filename:		plane_param_estimation.cpp
#
# Description: 
#
===============================================*/

#include <features/plane_param_estimation.h>


namespace ulysses
{
	void PlaneParamEstimation::estimatePlaneParams(Plane *plane, IntrinsicParam cam)
	{
		fp.open("plane_param_estimation.txt",std::ios::app);
		if(debug)
			fp<<std::endl<<"*********************************************************"<<std::endl;
		Eigen::Vector3d n=Eigen::Vector3d::Zero();
		Eigen::Vector3d pt=Eigen::Vector3d::Zero();
		Eigen::Matrix<double,1,1> d;
		for(size_t i=0;i<plane->points.size();i++)
		{
			n=n+plane->points[i].normal;
			pt=pt+plane->points[i].xyz;
		}
		n=n/plane->points.size();
		pt=pt/plane->points.size();
		d=-n.transpose()*pt;
		for(size_t i=0;i<plane->points.size();i++)
		{
//			plane->points[i].compute_cov(cam);
			compute_point_weight(plane->points[i],n,d(0,0),cam);
//			Eigen::Vector3d ray=plane->points[i].xyz/plane->points[i].xyz.norm();
//			fp<<n.transpose()*ray<<"\t"<<plane->points[i].xyz(2)<<"\t"<<n.transpose()*plane->points[i].cov*n<<"\t"<<plane->points[i].weight<<std::endl;
		}
		compute_plane_centroid(plane);
		compute_plane_cov_inv(plane);
		if(debug)
		{
			fp<<std::endl<<"plane fitting method - "<<pln_fitting_method<<std::endl;
			fp<<"\tavg point normal - "<<n.transpose()<<std::endl;
			fp<<"\tplane_centroid - "<<plane->centroid.transpose()<<std::endl;
			fp<<"\tplane_cov_inv - "<<std::endl<<plane->cov_inv<<std::endl;
			fp<<"\tplane_sqrt_cov_inv - "<<std::endl<<plane->sqrt_cov_inv<<std::endl;
		}
		compute_scatter_matrix(plane);
		Eigen::EigenSolver<Eigen::Matrix3d> es(plane->scatter_matrix);
		Eigen::Matrix<std::complex<double>,3,1> eigenvalues=es.eigenvalues();
		Eigen::Matrix<std::complex<double>,3,3> eigenvectors=es.eigenvectors();
		if(debug)
		{
			fp<<"\tmatrix Sw -"<<std::endl<<plane->scatter_matrix<<std::endl;
			fp<<"\teigenvalues - "<<eigenvalues.transpose()<<std::endl;
			fp<<"\teigenvectors -"<<std::endl<<eigenvectors<<std::endl;
		}
		double ev_min=DBL_MAX;
		size_t ev_min_idx=-1;
		for(size_t i=0;i<3;i++)
		{
			if(eigenvalues(i).real()<ev_min)
			{
				ev_min=eigenvalues(i).real();
				ev_min_idx=i;
			}
		}
		plane->normal(0)=eigenvectors(0,ev_min_idx).real();
		plane->normal(1)=eigenvectors(1,ev_min_idx).real();
		plane->normal(2)=eigenvectors(2,ev_min_idx).real();
		if(plane->normal.transpose()*plane->centroid>0)
		{
			plane->normal=-plane->normal;
		}
		plane->d=-plane->normal.transpose()*plane->centroid;
		if(debug)
		{
			fp<<"\tplane param - "<<plane->normal.transpose()<<", "<<plane->d<<std::endl;
		}
		fp.close();
	}

	void PlaneParamEstimation::compute_point_weight(Point &point, Eigen::Vector3d n, double d, IntrinsicParam cam)
	{
		Eigen::Matrix<double,1,1> tmp;
//		Eigen::Matrix3d dv_dp, Cov, Cov_inv;
//		bool invertible;
//		double det;
		if(pln_fitting_method==0)
		{
			point.weight=1;
		}
		else if(pln_fitting_method==1)
		{
			Eigen::Matrix3d K=cam.getMatrix();
			Eigen::Matrix3d K_inv=K.inverse();
			Eigen::Vector3d u(point.u,point.v,1);
			Eigen::Vector3d Ku=K_inv*u;
			double sigma_z=cam.m_fp*cam.sigma_disparity*point.xyz(2)*point.xyz(2);
			Eigen::Matrix3d C_u=Eigen::Matrix3d::Zero();
			C_u(0,0)=cam.sigma_u;
			C_u(1,1)=cam.sigma_v;
			point.cov=Ku*sigma_z*Ku.transpose()+point.xyz(2)*point.xyz(2)*K_inv*C_u*K_inv.transpose();
			tmp=n.transpose()*point.cov*n;
			point.weight=1.0/tmp(0,0);
			// dv_dp=I-n*n^T;
			// Cov=dv_dp*point.cov*dv_dp;
			// weight =n^T*Cov_inv*n;
//			dv_dp=Eigen::Matrix3d::Identity()-n*n.transpose();
//			Cov=dv_dp*point.cov*dv_dp.transpose();
//			Cov_inv=Cov.inverse();
//			Cov_inv=dv_dp.inverse().transpose()*point.cov_inv*dv_dp.inverse();
//			Cov.computeInverseWithCheck(Cov_inv,invertible);
//			if(invertible)
//			{
//			tmp=n.transpose()*Cov_inv*n;
//			point.weight=tmp(0,0);
//			}
//			else
//			{
//				std::cerr<<"Cov uninvertible!"<<std::endl;
//			}
		}
//		else if(pln_fitting_method==2)
//		{
//			// dv_dp=I*(1+d/n^T*p)-p*n^T*d/(n^T*p)^2;
//			// Cov=dv_dp*point.cov*dv_dp;
//			// weight_angle = [p_pi/(pln_n^T*p_pi)]^T * cov_inv * [p_pi/(pln_n^T*p_pi)];
//			tmp=n.transpose()*point.xyz;
//			dv_dp=Eigen::Matrix3d::Identity()*(1+d/tmp(0,0))
//				  -point.xyz*n.transpose()*(d/(tmp(0,0)*tmp(0,0)));
////			Cov=dv_dp*point.cov*dv_dp.transpose();
////			Cov_inv=Cov.inverse();
//			Cov_inv=dv_dp.inverse().transpose()*point.cov_inv*dv_dp.inverse();
////			point.cov.computeInverseAndDetWithCheck(Cov_inv,det,invertible);
////			Cov.computeInverseAndDetWithCheck(Cov_inv,det,invertible);
////			if(debug)
////			{
////				fp<<"point "<<point.xyz.transpose()<<std::endl;
////				fp<<"\tdv_dp - "<<std::endl<<dv_dp<<std::endl;
////				fp<<"\tpoint cov - "<<std::endl<<point.cov<<std::endl;
////				fp<<"\tCov - "<<std::endl<<Cov<<std::endl;
//////				fp<<"\tdet - "<<det<<std::endl;
//////				fp<<"\tinvertible - "<<invertible<<std::endl;
////				fp<<"\tCov_inv - "<<std::endl<<Cov_inv<<std::endl;
////			}
////			if(invertible)
////			{
//			tmp=point.xyz.transpose()*Cov_inv*point.xyz/
//				((n.transpose()*point.xyz)*(n.transpose()*point.xyz));
//			point.weight=tmp(0,0);
////			}
////			else
////			{
////				std::cerr<<"Cov uninvertible!"<<std::endl;
////			}
//		}
//		if(point.weight<0)
//		{
//			fp<<"point weight - "<<point.weight<<std::endl;
//			fp<<"point "<<point.xyz.transpose()<<std::endl;
//			fp<<"\tdv_dp - "<<std::endl<<dv_dp<<std::endl;
//			fp<<"\tpoint cov - "<<point.cov.determinant()<<std::endl;
//			fp<<"\tCov - "<<point.cov.determinant()<<"\t"<<Cov.determinant()<<"\t"<<Cov_inv.determinant()<<std::endl;
//			fp<<"\tCov_inv - "<<Cov_inv.determinant()<<std::endl;
//		}
	}

	void PlaneParamEstimation::compute_plane_centroid(Plane *plane)
	{
		plane->centroid.setZero();
		double sum=0;
		for(size_t i=0;i<plane->points.size();i++)
		{
			plane->centroid=plane->centroid+plane->points[i].xyz*plane->points[i].weight;
			sum+=plane->points[i].weight;
		}
		plane->centroid=plane->centroid/sum;
	}

	void PlaneParamEstimation::compute_plane_cov_inv(Plane *plane)
	{
		plane->cov_inv.setZero();
		Eigen::Matrix4d tmp;
		for(size_t i=0;i<plane->points.size();i++)
		{
			tmp.block<3,3>(0,0)=plane->points[i].xyz*plane->points[i].xyz.transpose();
			tmp.block<3,1>(0,3)=plane->points[i].xyz;
			tmp.block<1,3>(3,0)=plane->points[i].xyz.transpose();
			tmp(3,3)=1;
			tmp=tmp*plane->points[i].weight;
			plane->cov_inv=plane->cov_inv+tmp;
		}
//		plane->cov_inv/=plane->points.size();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es;
		es.compute(plane->cov_inv);
		Eigen::Vector4d Lambda=es.eigenvalues();
		Eigen::Matrix4d U=es.eigenvectors();
		Eigen::Matrix4d sqrt_Lambda=Eigen::Matrix4d::Zero();
		Eigen::Matrix4d inv_Lambda=Eigen::Matrix4d::Zero();
		for(size_t i=0;i<4;i++)
		{
			sqrt_Lambda(i,i)=sqrt(Lambda(i));
			if(Lambda(i)>0.01)
			{
				inv_Lambda(i,i)=1.0/Lambda(i);
			}
		}
		plane->sqrt_cov_inv=U*sqrt_Lambda*U.transpose();
		plane->cov=U*inv_Lambda*U.transpose();
	}

	void PlaneParamEstimation::compute_scatter_matrix(Plane *plane)
	{
		plane->scatter_matrix.setZero();
		for(size_t i=0;i<plane->points.size();i++)
		{
			Eigen::Vector3d pi_vec=plane->points[i].xyz-plane->centroid;
			plane->scatter_matrix=plane->scatter_matrix+pi_vec*pi_vec.transpose()*plane->points[i].weight;
		}
	}
}
