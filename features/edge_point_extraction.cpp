/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 15:16
#
# Filename:		edge_point_extraction.cpp
#
# Description: 
#
===============================================*/

#include "features/edge_point_extraction.h"

namespace ulysses
{
	void EdgePointExtraction::extractEdgePoints(Scan *scan)
	{
		if(debug) fp.open("edge_point_extraction.txt",std::ios::app);
		if(debug) fp<<std::endl<<"******************************************************************"<<std::endl;

		// edge
		pcl::PointCloud<pcl::Label>::Ptr labels_edge=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> edge_indices;

		// for edge detection;
		// change the invalid depth in scan->point_cloud from zero to infinite;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		*cloud_tmp=*scan->points();
		for(size_t i=0;i<cloud_tmp->height;i++)
		{
			for(size_t j=0;j<cloud_tmp->width;j++)
			{
				double dep=cloud_tmp->points[cloud_tmp->width*i+j].z;
				if(std::abs(dep)<1e-4)
				{
					cloud_tmp->points[cloud_tmp->width*i+j].z=std::numeric_limits<double>::max();
				}
			}
		}

		// edge detection;
		if (getEdgeType () & EDGELABEL_HIGH_CURVATURE)
		{
			pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputNormals(scan->normals());
		}
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputCloud(cloud_tmp);
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::compute(*labels_edge, edge_indices);

		if(debug)
		{
			fp<<"organized edge detection "<<std::endl;
//			fp<<"\tEDGELABEL_NAN_BOUNDARY - "<<edge_indices[0].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_OCCLUDING - "<<edge_indices[1].indices.size()<<std::endl;
//			fp<<"\tEDGELABEL_OCCLUDED - "<<edge_indices[2].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_HIGH_CURVATURE - "<<edge_indices[3].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_RGB_CANNY - "<<edge_indices[4].indices.size()<<std::endl;
		}

//		// scan->edge_points;
//		// fitting local line segment of each edge point;
//		ANNkd_tree *kdtree; // kdtree built by occluding points;
//		ANNpoint query_point=annAllocPt(3);
//		ANNidxArray index=new ANNidx[K_ANN];
//		ANNdistArray distance=new ANNdist[K_ANN];
//		// edge_indices[1] - occluding points;
//		// edge_indices[2] - occluded points;
//		ANNpointArray edge_points=annAllocPts(edge_indices[1].indices.size(),3);
//		// kdtree build by the pixel coordinates of the occluding points;
//		// used for the association of occluding and occluded points;
//		ANNkd_tree *kdtree_pixel;
//		ANNidxArray index_pixel=new ANNidx[K_ANN];
//		ANNdistArray distance_pixel=new ANNdist[K_ANN];
//		ANNpointArray edge_points_pixel=annAllocPts(edge_indices[1].indices.size(),2);
//		ANNpoint query_point_pixel=annAllocPt(2);
//		scan->edge_points.resize(edge_indices[1].indices.size());
		for(size_t i=0;i<edge_indices[1].indices.size();i++)
		{
			int idx=edge_indices[1].indices[i];
			scan->addEdgePoint(idx);
//			// fill scan->edge_points;
//			scan->edge_points[i]=new EdgePoint;
//			scan->edge_points[i]->xyz(0)=scan->point_cloud->at(idx).x;
//			scan->edge_points[i]->xyz(1)=scan->point_cloud->at(idx).y;
//			scan->edge_points[i]->xyz(2)=scan->point_cloud->at(idx).z;
//			scan->edge_points[i]->pixel(0)=scan->pixel_cloud->at(idx).x;
//			scan->edge_points[i]->pixel(1)=scan->pixel_cloud->at(idx).y;
//			scan->edge_points[i]->rgb(0)=0;
//			scan->edge_points[i]->rgb(1)=0;
//			scan->edge_points[i]->rgb(2)=0;
//			scan->edge_points[i]->index=idx;
		}

//		int N=scan->edge_points.size();
//		scan->edge_points.resize(N+edge_indices[3].indices.size());
		for(size_t i=0;i<edge_indices[3].indices.size();i++)
		{
			int idx=edge_indices[3].indices[i];
			scan->addEdgePoint(idx);
//			int j=N+i;
//			// fill scan->edge_points;
//			scan->edge_points[j]=new EdgePoint;
//			scan->edge_points[j]->xyz(0)=scan->point_cloud->at(idx).x;
//			scan->edge_points[j]->xyz(1)=scan->point_cloud->at(idx).y;
//			scan->edge_points[j]->xyz(2)=scan->point_cloud->at(idx).z;
//			scan->edge_points[j]->pixel(0)=scan->pixel_cloud->at(idx).x;
//			scan->edge_points[j]->pixel(1)=scan->pixel_cloud->at(idx).y;
//			scan->edge_points[j]->rgb(0)=0;
//			scan->edge_points[j]->rgb(1)=0;
//			scan->edge_points[j]->rgb(2)=0;
//			scan->edge_points[j]->index=idx;
		}
		for(size_t i=0;i<edge_indices[4].indices.size();i++)
		{
			int idx=edge_indices[4].indices[i];
			scan->addEdgePoint(idx);
		}

		/*
		// build the kd-tree using the occluding edge points;
		kdtree=new ANNkd_tree(edge_points,scan->edge_points.size(),3);
		// build the kd-tree using the pixel coordinates of the occluding edge points;
		kdtree_pixel=new ANNkd_tree(edge_points_pixel,scan->edge_points.size(),2);
		// for each occluding edge point;
		// search for the nearest neighbor in the occluding edge points;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
		Eigen::Vector3d Lambda;
		Eigen::Matrix3d U;
		Eigen::Vector3d e1;
		e1.setOnes();
		for(size_t i=0;i<scan->edge_points.size();i++)
		{
			query_point[0]=scan->edge_points[i]->xyz(0);
			query_point[1]=scan->edge_points[i]->xyz(1);
			query_point[2]=scan->edge_points[i]->xyz(2);
			int points_in_radius=kdtree->annkFRSearch(query_point,sqRad_ANN,K_ANN,index,distance);
			//	ANNpoint q, // query point
			//	ANNdist sqRad, // squared radius
			//	int k = 0, // number of near neighbors to return
			//	ANNidxArray nn_idx = NULL, // nearest neighbor array (modified)
			//	ANNdistArray dd = NULL, // dist to near neighbors (modified)
			//	double eps = 0.0); // error bound
			if(points_in_radius>7)
			{
				scan->edge_points[i]->isEdge=true;
				for(size_t j=0;j<K_ANN;j++)
				{
					if(index[j]==ANN_NULL_IDX)
						continue;
					scan->edge_points[i]->neighbors.push_back(scan->edge_points[index[j]]);
				}
				Eigen::Vector3d mean=Eigen::Vector3d::Zero();
				for(size_t j=0;j<scan->edge_points[i]->neighbors.size();j++)
				{
					mean+=scan->edge_points[i]->neighbors[j]->xyz;
				}
				mean=mean/scan->edge_points[i]->neighbors.size();
				scan->edge_points[i]->cov.setZero();
				for(size_t j=0;j<scan->edge_points[i]->neighbors.size();j++)
				{
					Eigen::Vector3d vec3d=scan->edge_points[i]->neighbors[j]->xyz-mean;
					scan->edge_points[i]->cov+=vec3d*vec3d.transpose();
				}
				scan->edge_points[i]->cov=scan->edge_points[i]->cov/(scan->edge_points[i]->neighbors.size()-1);
				if(scan->edge_points[i]->cov.determinant()<1e-20)
				{
					scan->edge_points[i]->isEdge=false;
				}
				else
				{
					es.compute(scan->edge_points[i]->cov);
					Lambda=es.eigenvalues();
					U=es.eigenvectors();
//					fp<<Lambda.transpose()<<"\t"<<Lambda(2)/Lambda(0)<<"\t"<<Lambda(2)/Lambda(1)<<"\t"<<Lambda(1)/Lambda(2)/0.01+(Lambda(1)/Lambda(0)-1)/5.0<<std::endl;
					scan->edge_points[i]->meas_Edge=Lambda(1)/Lambda(2)/0.01+(Lambda(1)/Lambda(0)-1)/5.0;
					if(scan->edge_points[i]->meas_Edge>edge_meas)
					{
						scan->edge_points[i]->isEdge=false;
					}
					else
					{
						Eigen::Vector3d u1=U.block<3,1>(0,2);
						Eigen::Vector3d u2=U.block<3,1>(0,1);
						Eigen::Vector3d u3=U.block<3,1>(0,0);
						scan->edge_points[i]->cov=u2*u2.transpose()+u3*u3.transpose();
						if(u1.transpose()*e1<0)
							scan->edge_points[i]->dir=-u1;
						else
							scan->edge_points[i]->dir=u1;
					}
				}
			}
		}
		*/

//		// occluded points;
//		// shadow on the plane;
//		ANNkd_tree *kdtree_occluded;
//		ANNidxArray index_occluded=new ANNidx[K_ANN];
//		ANNdistArray distance_occluded=new ANNdist[K_ANN];
//		ANNpointArray edge_points_occluded=annAllocPts(edge_indices[2].indices.size(),3);
//		scan->edge_points_occluded.resize(edge_indices[2].indices.size());
//		for(size_t i=0;i<edge_indices[2].indices.size();i++)
//		{
//			int idx=edge_indices[2].indices[i];
//			// fill scan->edge_points_occluded;
//			scan->edge_points_occluded[i]=new EdgePoint;
//			scan->edge_points_occluded[i]->xyz(0)=scan->point_cloud->at(idx).x;
//			scan->edge_points_occluded[i]->xyz(1)=scan->point_cloud->at(idx).y;
//			scan->edge_points_occluded[i]->xyz(2)=scan->point_cloud->at(idx).z;
//			scan->edge_points_occluded[i]->pixel(0)=scan->pixel_cloud->at(idx).x;
//			scan->edge_points_occluded[i]->pixel(1)=scan->pixel_cloud->at(idx).y;
//			scan->edge_points_occluded[i]->index=idx;
//		}
//
		/*
		// build the kd-tree using the occluded edge points;
		// for each occluded edge point;
		// search for the nearest 3D neighbor in the occluded edge points;
		// and search for the nearest 3D neighbor in the image plane for the occluding point;
		kdtree_occluded=new ANNkd_tree(edge_points_occluded,scan->edge_points_occluded.size(),3);
		for(size_t i=0;i<scan->edge_points_occluded.size();i++)
		{
			// search for nearest occluded point in the 3D space;
			query_point[0]=scan->edge_points_occluded[i]->xyz(0);
			query_point[1]=scan->edge_points_occluded[i]->xyz(1);
			query_point[2]=scan->edge_points_occluded[i]->xyz(2);
			int points_in_radius=kdtree_occluded->annkFRSearch(query_point,sqRad_ANN,K_ANN,index_occluded,distance_occluded);
			if(points_in_radius>7)
			{
				scan->edge_points_occluded[i]->isEdge=true;
				for(size_t j=0;j<K_ANN;j++)
				{
					if(index_occluded[j]==ANN_NULL_IDX)
						continue;
					scan->edge_points_occluded[i]->neighbors.push_back(scan->edge_points_occluded[index_occluded[j]]);
				}
				Eigen::Vector3d mean=Eigen::Vector3d::Zero();
				for(size_t j=0;j<scan->edge_points_occluded[i]->neighbors.size();j++)
				{
					mean+=scan->edge_points_occluded[i]->neighbors[j]->xyz;
				}
				mean=mean/scan->edge_points_occluded[i]->neighbors.size();
				scan->edge_points_occluded[i]->cov.setZero();
				for(size_t j=0;j<scan->edge_points_occluded[i]->neighbors.size();j++)
				{
					Eigen::Vector3d vec3d=scan->edge_points_occluded[i]->neighbors[j]->xyz-mean;
					scan->edge_points_occluded[i]->cov+=vec3d*vec3d.transpose();
				}
				scan->edge_points_occluded[i]->cov=scan->edge_points_occluded[i]->cov/(scan->edge_points_occluded[i]->neighbors.size()-1);
				if(scan->edge_points_occluded[i]->cov.determinant()<1e-20)
					scan->edge_points_occluded[i]->isEdge=false;
			}
			// search for nearest occluding point in the image plane;
			query_point_pixel[0]=scan->edge_points_occluded[i]->pixel(0);
			query_point_pixel[1]=scan->edge_points_occluded[i]->pixel(1);
			kdtree_pixel->annkSearch(query_point_pixel,1,index_pixel,distance_pixel,0);
			EdgePoint *occluding=scan->edge_points[index_pixel[0]];
			EdgePoint *occluded=scan->edge_points_occluded[i];
			if(distance_pixel[0]<thres_pxl_sq && occluding->isEdge)// && occluded->isEdge)
			{
				scan->edge_points_occluded[i]->occluding=scan->edge_points[index_pixel[0]];
				scan->edge_points_occluded[i]->occluding->occluded=scan->edge_points_occluded[i];

				double dist_min=DBL_MAX;
				int idx_min=-1;
				for(size_t j=0;j<scan->observed_planes.size();j++)
				{
					Eigen::Vector3d tmp=occluded->xyz-scan->observed_planes[j]->projected_point(occluding->xyz);
					double dist=tmp.norm();
					if(dist<dist_min) {dist_min=dist;idx_min=j;}
				}
				if(dist_min<thres_occluded_dist)
				{
					double ratio=occluding->xyz.norm()/occluded->xyz.norm();
					Eigen::Matrix<double,1,1> tmp=scan->observed_planes[idx_min]->normal.transpose()*occluding->xyz;
					double angle=-tmp(0,0)/occluding->xyz.norm();
//					fp<<ratio<<"\t"<<angle<<std::endl;
					if(ratio>thres_ratio && angle>thres_angle)
					{
						ProjectiveRay *proj_ray=new ProjectiveRay(occluding,occluded);
						proj_ray->plane=scan->observed_planes[idx_min];
						proj_ray->occluded_proj=new EdgePoint;
						proj_ray->occluded_proj->xyz=proj_ray->plane->projected_point(occluding->xyz);
						scan->projective_rays.push_back(proj_ray);
					}
//					fp<<dist_min<<std::endl;
				}

//				if(debug)
//				{
//					fp<<i<<"\t"<<index_pixel[0]<<"\t"<<distance_pixel[0]
//						 <<"\n\t"<<scan->edge_points_occluded[i]->xyz.transpose()
//						 <<"\n\t"<<scan->edge_points_occluded[i]->occluding->xyz.transpose()<<std::endl;
//				}
			}
//			else
//			{
//				scan->edge_points_occluded[i]->occluding=0;
////				if(debug)
////				{
////					fp<<i<<"\t"<<index_pixel[0]<<"\t"<<distance_pixel[0]
////						 <<"\n\t"<<scan->edge_points_occluded[i]->xyz.transpose()
////						 <<"\n\t"<<scan->edge_points_occluded[i]->occluding<<std::endl;
////				}
//			}
			for(size_t j=0;j<scan->observed_planes.size();j++)
			{
//				fp<<scan->observed_planes[j]->point_plane_dist(scan->edge_points_occluded[i]->xyz)<<"\t";
				if(scan->observed_planes[j]->point_plane_dist(scan->edge_points_occluded[i]->xyz)<=0.1)
				{
					if(scan->edge_points_occluded[i]->plane==0)
					{
						scan->edge_points_occluded[i]->plane=scan->observed_planes[j];
					}
					else if(scan->edge_points_occluded[i]->plane->point_plane_dist(scan->edge_points_occluded[i]->xyz)>scan->observed_planes[j]->point_plane_dist(scan->edge_points_occluded[i]->xyz))
					{
						scan->edge_points_occluded[i]->plane=scan->observed_planes[j];
					}
				}
			}
//			if(scan->edge_points_occluded[i]->plane==0)
//				fp<<"---\t"<<scan->edge_points_occluded[i]->plane;
//			else
//				fp<<"---\t"<<scan->edge_points_occluded[i]->plane->point_plane_dist(scan->edge_points_occluded[i]->xyz);
//			fp<<std::endl;
		}
		*/



		/*
		fitLinesHough(scan->edge_points,scan->lines_occluding,EDGELABEL_OCCLUDING);
		std::cout<<"fitted lines in occluding points: "<<scan->lines_occluding.size()<<std::endl;
		for(std::list<Line*>::iterator it_line=scan->lines_occluding.begin();it_line!=scan->lines_occluding.end();it_line++)
		{
			fitLinesLS(*it_line);
			std::cout<<"\t"<<(*it_line)->m<<"\t"<<(*it_line)->n<<"\t"<<(*it_line)->x0<<"\t"<<(*it_line)->y0<<std::endl;

			double z=(*it_line)->end_point_1(2);
			(*it_line)->end_point_1(0)=(*it_line)->m*z+(*it_line)->x0;
			(*it_line)->end_point_1(1)=(*it_line)->n*z+(*it_line)->y0;
			z=(*it_line)->end_point_2(2);
			(*it_line)->end_point_2(0)=(*it_line)->m*z+(*it_line)->x0;
			(*it_line)->end_point_2(1)=(*it_line)->n*z+(*it_line)->y0;

			(*it_line)->end_point_1_img=scan->cam.project((*it_line)->end_point_1);
			(*it_line)->end_point_2_img=scan->cam.project((*it_line)->end_point_2);
			double x1=(*it_line)->end_point_1_img(0);
			double y1=(*it_line)->end_point_1_img(1);
			double x2=(*it_line)->end_point_2_img(0);
			double y2=(*it_line)->end_point_2_img(1);
			(*it_line)->a=(y1-y2)/(x1*y2-x2*y1);
			(*it_line)->b=(x2-x1)/(x1*y2-x2*y1);
		}

		fitLinesHough(scan->edge_points_occluded,scan->lines_occluded,EDGELABEL_OCCLUDED);
		std::cout<<"fitted lines in occluded points: "<<scan->lines_occluded.size()<<std::endl;
		for(std::list<Line*>::iterator it_line=scan->lines_occluded.begin();it_line!=scan->lines_occluded.end();it_line++)
		{
			fitLinesLS(*it_line);
			std::cout<<"\t"<<(*it_line)->m<<"\t"<<(*it_line)->n<<"\t"<<(*it_line)->x0<<"\t"<<(*it_line)->y0<<std::endl;

			double z=(*it_line)->end_point_1(2);
			(*it_line)->end_point_1(0)=(*it_line)->m*z+(*it_line)->x0;
			(*it_line)->end_point_1(1)=(*it_line)->n*z+(*it_line)->y0;
			z=(*it_line)->end_point_2(2);
			(*it_line)->end_point_2(0)=(*it_line)->m*z+(*it_line)->x0;
			(*it_line)->end_point_2(1)=(*it_line)->n*z+(*it_line)->y0;

			(*it_line)->end_point_1_img=scan->cam.project((*it_line)->end_point_1);
			(*it_line)->end_point_2_img=scan->cam.project((*it_line)->end_point_2);
			double x1=(*it_line)->end_point_1_img(0);
			double y1=(*it_line)->end_point_1_img(1);
			double x2=(*it_line)->end_point_2_img(0);
			double y2=(*it_line)->end_point_2_img(1);
			(*it_line)->a=(y1-y2)/(x1*y2-x2*y1);
			(*it_line)->b=(x2-x1)/(x1*y2-x2*y1);
		}

		for(std::list<Line*>::iterator it_occluding=scan->lines_occluding.begin();it_occluding!=scan->lines_occluding.end();it_occluding++)
		{
			double x1=(*it_occluding)->end_point_1_img(0);
			double y1=(*it_occluding)->end_point_1_img(1);
			double x2=(*it_occluding)->end_point_2_img(0);
			double y2=(*it_occluding)->end_point_2_img(1);
			for(std::list<Line*>::iterator it_occluded=scan->lines_occluded.begin();it_occluded!=scan->lines_occluded.end();it_occluded++)
			{
				double a=(*it_occluded)->a;
				double b=(*it_occluded)->b;
				double d1=fabs(a*x1+b*y1+1)/(sqrt(a*a+b*b));
				double d2=fabs(a*x2+b*y2+1)/(sqrt(a*a+b*b));
				std::cout<<d1<<","<<d2<<"\t";
				if(d1<10 && d2<10)
				{
					(*it_occluding)->occluded.push_back(*it_occluded);
				}
			}
			std::cout<<std::endl;
		}

		for(std::list<Line*>::iterator it_occluding=scan->lines_occluding.begin();it_occluding!=scan->lines_occluding.end();it_occluding++)
		{
			if((*it_occluding)->occluded.size()==0)
				continue;
			for(std::list<EdgePoint*>::iterator it=(*it_occluding)->points.begin();it!=(*it_occluding)->points.end();it++)
			{
				(*it)->xyz(0)=(*it_occluding)->m*(*it)->xyz(2)+(*it_occluding)->x0;
				(*it)->xyz(1)=(*it_occluding)->n*(*it)->xyz(2)+(*it_occluding)->y0;
			}
			for(size_t i=0;i<(*it_occluding)->occluded.size();i++)
			{
				for(std::list<EdgePoint*>::iterator it =(*it_occluding)->occluded[i]->points.begin();
													it!=(*it_occluding)->occluded[i]->points.end();it++)
				{
					(*it)->xyz(0)=(*it_occluding)->m*(*it)->xyz(2)+(*it_occluding)->x0;
					(*it)->xyz(1)=(*it_occluding)->n*(*it)->xyz(2)+(*it_occluding)->y0;
				}
			}
		}
		*/
		

//		if(debug)
//		{
//			fp<<"extracted edge Points - "<<scan->edge_points.size()<<std::endl;
//			for(size_t i=0;i<scan->edge_points.size();i++)
//			{
//				fp<<i<<" - "<<scan->edge_points[i]->xyz.transpose();
//				if(scan->edge_points[i]->isEdge)
//				{
//					fp<<"\tneighbors - "<<scan->edge_points[i]->neighbors.size()<<std::endl;
//					fp<<"\tcov - "<<std::endl<<scan->edge_points[i]->cov<<std::endl;
//				}
//			}
//		}

//		if(debug)
//		{
//			visEdgePoints(scan,vis);
//			vis->spin();
//		}

//		annDeallocPt(query_point);
//		annDeallocPt(query_point_pixel);
//		annDeallocPts(edge_points);
//		annDeallocPts(edge_points_pixel);
//		annDeallocPts(edge_points_occluded);
//		delete kdtree;
//		delete kdtree_pixel;
//		delete kdtree_occluded;
//		delete index;
//		delete index_pixel;
//		delete index_occluded;
//		delete distance;
//		delete distance_pixel;
//		delete distance_occluded;
		if(debug) fp.close();
	}

	void EdgePointExtraction::visEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		// add raw scan data;
		sprintf(id,"scan%f",scan->time());
		if (!vis->updatePointCloud (scan->points(), id))
			vis->addPointCloud (scan->points(), id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add occluding points;
		std::cout<<"\nvisualize occluding edge points - red"<<std::endl;
		edge->resize(scan->sizeEdgePoint());
		for(size_t i=0;i<scan->sizeEdgePoint();i++)
		{
			edge->at(i).x=scan->edgePoint(i)->xyz(0);
			edge->at(i).y=scan->edgePoint(i)->xyz(1);
			edge->at(i).z=scan->edgePoint(i)->xyz(2);
			// unsigned int color=(1-scan->edge_points[i]->meas_Edge/20.0)*255.0;
			// unsigned char r=color;
			// unsigned char g=color;
			// unsigned char b=color;
			edge->at(i).r=255;
			edge->at(i).g=0;
			edge->at(i).b=0;
		}
		sprintf(id,"edgePoints%f",scan->time());
		if (!vis->updatePointCloud (edge, id))
			vis->addPointCloud (edge, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
		
//		// add occluded points;
//		std::cout<<"visualize occluded edge points - green"<<std::endl;
//		edge->resize(scan->edge_points_occluded.size());
//		for(size_t i=0;i<scan->edge_points_occluded.size();i++)
//		{
//			edge->at(i).x=scan->edge_points_occluded[i]->xyz(0);
//			edge->at(i).y=scan->edge_points_occluded[i]->xyz(1);
//			edge->at(i).z=scan->edge_points_occluded[i]->xyz(2);
//			edge->at(i).r=0;
//			edge->at(i).g=255;
//			edge->at(i).b=0;
//		}
//		sprintf(id,"edgePoints_occluded%d",scan->id);
//		if (!vis->updatePointCloud (edge, id))
//			vis->addPointCloud (edge, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);

	}

}

