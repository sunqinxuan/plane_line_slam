/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-07 09:18
#
# Filename:		plane_extraction_base.cpp
#
# Description: 
#
************************************************/

#include "features/plane_extraction_base.h"

namespace ulysses
{

	void PlaneExtractionBase::fitPlaneModel(Plane *plane)
	{
//		fp.open("plane_extraction.txt",std::ios::app);
		if(debug) fp<<std::endl<<"*************fitPlaneModel()*************************"<<std::endl;

		computeCentroidScatter(plane);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->scatter_point);
		Eigen::Vector3d eig_vals=es.eigenvalues();
		Eigen::Matrix3d eig_vecs=es.eigenvectors();
		eig_vals=eig_vals.cwiseAbs();
		double min=DBL_MAX;
		int idx=-1;
		for(int i=0;i<3;i++)
		{
			if(eig_vals(i)<min)
			{
				min=eig_vals(i);
				idx=i;
			}
		}
		if(debug)
		{
			fp<<"eigenvalues: "<<eig_vals.transpose()<<std::endl;
			fp<<"eigenvectors: "<<std::endl<<eig_vecs<<std::endl;
		}

		plane->n=eig_vecs.block<3,1>(0,idx);
		if(plane->n.transpose()*plane->centroid_point>0)
		{
			plane->n=-plane->n;
		}
		plane->d=-plane->n.transpose()*plane->centroid_point;
		if(debug) fp<<"fitted plane model: ("<<plane->n.transpose()<<","<<plane->d<<")"<<std::endl;

		computePlaneInfo(plane);

//		fp.close();
	}


	void PlaneExtractionBase::computeCentroidScatter(Plane *plane)
	{
		Eigen::Vector3d tmp;
		plane->centroid_point.setZero();
		plane->centroid_color.setZero();
		plane->scatter_point.setZero();
		plane->scatter_color.setZero();
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			plane->centroid_point(0)+=plane->ptr_points->at(idx).x;
			plane->centroid_point(1)+=plane->ptr_points->at(idx).y;
			plane->centroid_point(2)+=plane->ptr_points->at(idx).z;
			plane->centroid_color(0)+=double(plane->ptr_points->at(idx).r)/255.0;
			plane->centroid_color(1)+=double(plane->ptr_points->at(idx).g)/255.0;
			plane->centroid_color(2)+=double(plane->ptr_points->at(idx).b)/255.0;
		}
		plane->centroid_point/=plane->indices.size();
		plane->centroid_color/=plane->indices.size();
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			Eigen::Vector3d tmp;
			tmp(0)=plane->ptr_points->at(idx).x-plane->centroid_point(0);
			tmp(1)=plane->ptr_points->at(idx).y-plane->centroid_point(1);
			tmp(2)=plane->ptr_points->at(idx).z-plane->centroid_point(2);
			plane->scatter_point+=tmp*tmp.transpose();
			tmp(0)=double(plane->ptr_points->at(idx).r)/255.0-plane->centroid_color(0);
			tmp(1)=double(plane->ptr_points->at(idx).g)/255.0-plane->centroid_color(1);
			tmp(2)=double(plane->ptr_points->at(idx).b)/255.0-plane->centroid_color(2);
			plane->scatter_color+=tmp*tmp.transpose();
		}
		plane->scatter_point/=plane->indices.size();
		plane->scatter_color/=plane->indices.size();

		if(debug)
		{
			fp<<"plane->centroid_point: "<<plane->centroid_point.transpose()<<std::endl;
			fp<<"plane->scatter_point: "<<std::endl<<plane->scatter_point<<std::endl;
			fp<<"plane->centroid_color: "<<plane->centroid_color.transpose()<<std::endl;
			fp<<"plane->scatter_color: "<<std::endl<<plane->scatter_color<<std::endl;
		}
	}

	void PlaneExtractionBase::computePlaneInfo(Plane *plane)
	{
		plane->info.setZero();
		Eigen::Matrix4d tmp;
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			Eigen::Vector3d pt;
			pt(0)=plane->ptr_points->at(idx).x;
			pt(1)=plane->ptr_points->at(idx).y;
			pt(2)=plane->ptr_points->at(idx).z;
			tmp.block<3,3>(0,0)=pt*pt.transpose();
			tmp.block<3,1>(0,3)=pt;
			tmp.block<1,3>(3,0)=pt.transpose();
			tmp(3,3)=1;
			plane->info+=tmp;
		}
		//plane->info/=plane->indices.size();
		if(debug) fp<<"plane->info: "<<std::endl<<plane->info<<std::endl;

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es;
		es.compute(plane->info);
		Eigen::Vector4d Lambda=es.eigenvalues();
		Eigen::Matrix4d U=es.eigenvectors();
		Eigen::Matrix4d sqrt_Lambda=Eigen::Matrix4d::Zero();
		Eigen::Matrix4d inv_Lambda=Eigen::Matrix4d::Zero();
		for(int i=0;i<4;i++)
		{
			sqrt_Lambda(i,i)=sqrt(Lambda(i));
			if(Lambda(i)>0.01)
			{
				inv_Lambda(i,i)=1.0/Lambda(i);
			}
		}
		plane->sqrt_info=U*sqrt_Lambda*U.transpose();
		plane->cov=U*inv_Lambda*U.transpose();
		if(debug) fp<<"plane->sqrt_info: "<<std::endl<<plane->sqrt_info<<std::endl;
		if(debug) fp<<"plane->cov: "<<std::endl<<plane->cov<<std::endl;
	}

	void PlaneExtractionBase::visPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->points(), id))
			vis->addPointCloud (scan->points(), id);

		std::cout<<"\nvisualize extracted planes"<<std::endl;
		int i=0;
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++,i++)
		{
			if(it->second->Type()==PLANE) it->second->vis(vis,red[i%14],grn[i%14],blu[i%14]);
//			pcl::copyPointCloud(*scan->points(),it->second->plane()->inliers,*plane);
////			sprintf(id,"plane%d",i);
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(plane,red[i%14],grn[i%14],blu[i%14]);
//			if (!vis->updatePointCloud (plane, color, it->first))
//				vis->addPointCloud (plane, color, it->first);
		}
//		for(int i=0;i<scan->observed_planes.size();i++)
//		{
//			plane->resize(scan->observed_planes[i]->indices.size());
//			for(int j=0;j<scan->observed_planes[i]->indices.size();j++)
//			{
//				int idx=scan->observed_planes[i]->indices[j];
//				plane->at(j).x=scan->point_cloud->at(idx).x;
//				plane->at(j).y=scan->point_cloud->at(idx).y;
//				plane->at(j).z=scan->point_cloud->at(idx).z;
//			}
//		}
	}

}
