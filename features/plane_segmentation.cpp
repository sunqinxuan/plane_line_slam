/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-07 14:22
#
# Filename:		plane_segmentation.cpp
#
# Description: 
#
************************************************/

#include "features/plane_segmentation.h"

namespace ulysses
{
	using namespace std;
	double PlaneSegmentation::extractPlanes(Scan *scan)
	{
		if(debug)
		{
			fp.open("plane_extraction.txt",std::ios::app);
			fp<<"*****************extractPlanes**************************************"<<std::endl;
		}

		timeval start, end;
		double timeused;
		gettimeofday(&start,NULL);


		std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
		std::vector<pcl::PointIndices> inlier_indices;
		std::vector<pcl::ModelCoefficients> model_coefficients;
		pcl::PointCloud<pcl::Label>::Ptr labels_plane=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> label_indices;
		std::vector<pcl::PointIndices> boundary_indices;

		// plane segmentation;
		plane_segment=new pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>;
		plane_segment->setMinInliers(inliers);
		plane_segment->setAngularThreshold(pcl::deg2rad(ang));
		plane_segment->setDistanceThreshold(dist); 
		plane_segment->setMaximumCurvature(curv);
		if(debug)
		{
			fp<<"MinInliers: "<<plane_segment->getMinInliers()<<std::endl;
			fp<<"AngularThreshold: "<<pcl::rad2deg(plane_segment->getAngularThreshold())<<std::endl;
			fp<<"DistanceThreshold: "<<plane_segment->getDistanceThreshold()<<std::endl; 
			fp<<"MaximumCurvature: "<<plane_segment->getMaximumCurvature()<<std::endl;
		}

		plane_segment->setInputNormals(scan->normals());
		plane_segment->setInputCloud(scan->points());
		plane_segment->segmentAndRefine(regions, model_coefficients, inlier_indices, labels_plane, label_indices, boundary_indices);
		delete plane_segment;
//		cout<<"regions: "<<regions.size()<<endl;
//		cout<<"inlier_indices: "<<inlier_indices.size()<<endl;
//		cout<<"labels_plane: "<<labels_plane->points.size()<<endl;
//		cout<<"label_indices: "<<label_indices.size()<<endl;
//		cout<<"boundary_indices: "<<boundary_indices.size()<<endl;
//		cv::Mat img=cv::Mat::zeros(480,640,CV_16UC1);
//		int count=0;
//		for(int i=0;i<label_indices.size();i++)
//		{
//			if(label_indices[i].indices.size()<1000) continue;
//			cout<<i<<endl;
//			for(int j=0;j<label_indices[i].indices.size();j++)
//			{
//				int row=label_indices[i].indices[j]/img.cols;
//				int col=label_indices[i].indices[j]%img.cols;
//				img.at<unsigned short>(row,col)=65535-7000*count;
//			}
//			count++;
//		}
//		for(int i=0;i<img.rows;i++)
//		{
//			for(int j=0;j<img.cols;j++)
//			{
//				img.at<unsigned short>(i,j)=65535-2*labels_plane->points[i*img.cols+j].label;
//			}
//		}
//		cv::imwrite("img2.png",img);
//		cv::imshow("img",img);
//		cv::waitKey();

//		// generate the plane labels;
//		pcl::Label invalid_pt;
//		invalid_pt.label = unsigned (0);
//		labels_plane->points.resize (scan->points()->size(), invalid_pt);
//		labels_plane->width = scan->points()->width;
//		labels_plane->height = scan->points()->height;
//		for(size_t i=0;i<labels_plane->points.size();i++)
//		{
//			labels_plane->points[i].label=0;
//		}
//		for(size_t i=0;i<inlier_indices.size();i++)
//		{
//			for(size_t j=0;j<inlier_indices[i].indices.size();j++)
//			{
//				labels_plane->at(inlier_indices[i].indices[j]).label=i+1;
//			}
//		}

//		for(int i=0;i<img.rows;i++)
//		{
//			for(int j=0;j<img.cols;j++)
//			{
//				img.at<unsigned short>(i,j)=9362*labels_plane->points[i*img.cols+j].label;
//			}
//		}
//		cv::imshow("img",img);
//		cv::waitKey();

		// save the planar regions to scan->observed_planes;
		for(size_t i=0;i<regions.size();i++)
		{
			Plane *plane=new Plane;
			Eigen::Vector3f centroid=regions[i].getCentroid();
			Eigen::Vector4f coefficients=regions[i].getCoefficients();
			plane->centroid_point(0)=centroid(0);
			plane->centroid_point(1)=centroid(1);
			plane->centroid_point(2)=centroid(2);
			if(coefficients(3)<0) coefficients=-coefficients;
			coefficients=coefficients/coefficients.block<3,1>(0,0).norm();
			plane->n(0)=coefficients(0);
			plane->n(1)=coefficients(1);
			plane->n(2)=coefficients(2);
			plane->d=coefficients(3);
			plane->indices=inlier_indices[i].indices;
			plane->ptr_points=scan->points();
			fitPlaneModel(plane);

			Feature *feature=new Feature(plane);
//			scan->observed_planes.push_back(plane);
			scan->addFeature(feature);
		}

		if(debug)
		{
			fp<<std::endl<<"====================================="<<std::endl;
			fp<<"extracted planes: "<<std::endl; scan->printFeatures(fp);
		}

		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++)
		{
			for(iterFeature it2=scan->beginFeature();it2!=scan->endFeature();it2++)
			{
				if(it1->second->Type()!=PLANE || it2->second->Type()!=PLANE || it1==it2) continue;

				if(it1->second->plane()->n.cross(it2->second->plane()->n).norm()<THRES_RAD 
					&& fabs(it1->second->plane()->d-it2->second->plane()->d)<THRES_DIST)
				{
					it1->second->plane()->indices.insert(it1->second->plane()->indices.end(),
						it2->second->plane()->indices.begin(),it2->second->plane()->indices.end());
					fitPlaneModel(it1->second->plane());
					delete it2->second;
					it2=scan->eraseFeature(it2); it2--;
				}
			}
		}

		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

		if(debug) fp.close();
		return timeused;
	}

}
