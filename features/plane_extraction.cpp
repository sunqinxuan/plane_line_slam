/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 16:49
#
# Filename:		plane_extraction.cpp
#
# Description: 
#
===============================================*/

#include "features/plane_extraction.h"

namespace ulysses
{
	void Cell::computeAttribute()
	{
		Eigen::Vector3d tmp;
		if(isEmpty)
			return;
		// compute the expectation of position and color;
		for(std::vector<Point>::iterator it=points.begin();it!=points.end();++it)
		{
			avg_pps=avg_pps+it->pps;
			avg_rgb=avg_rgb+it->rgb;
		}
		avg_pps=avg_pps*(1.0/points.size());
		avg_rgb=avg_rgb*(1.0/points.size());
		// compute the covariance of position and color;
		for(std::vector<Point>::iterator it=points.begin();it!=points.end();++it)
		{
			tmp=it->pps;
			tmp-=avg_pps;
			cov_pps+=tmp*tmp.transpose();
			tmp=it->rgb;
			tmp-=avg_rgb;
			cov_rgb+=tmp*tmp.transpose();
		}
		// biased estimation of the covariance;
		cov_pps=cov_pps*(1.0/points.size());
		cov_rgb=cov_rgb*(1.0/points.size());
	}

	void Cells_bottom::push_point(Point point_tmp, int idx)
	{
//		std::cout<<"push point"<<std::endl;
		int theta=(point_tmp.pps[0])/delta_theta;
		int phy=(point_tmp.pps[1]+M_PI)/delta_phy;
		int d=(point_tmp.pps[2])/delta_d;
//		std::cout<<theta<<", "<<phy<<", "<<d<<std::endl;
		if(theta>=bins_theta)
			theta=bins_theta-1;
		if(phy>=bins_phy)
			phy=bins_phy-1;
		if(d>=bins_d)
			d=bins_d-1;
		int i=index(d,theta,phy);
		cells[i]->points.push_back(point_tmp);
		cells[i]->indices->indices.push_back(idx);
		cells[i]->isEmpty=false;
	}

	void Cells_bottom::computeCellAttributes()
	{
		for(size_t i=0;i<cells.size();i++)
		{
			if(cells[i]->isEmpty)
				continue;
			cells[i]->computeAttribute();
		}
	}

	void Cells_bottom::SortCells()
	{
		Sorted_Cell tmp_sort;
		sorted_cells.clear();
		for(int i=0;i<cells.size();i++)
		{
			tmp_sort.index=i;
			tmp_sort.num_point=cells[i]->points.size();
			sorted_cells.push_back(tmp_sort);
		}
		std::sort(sorted_cells.begin(), sorted_cells.end());
	}

	bool PlaneExtraction::loadPoints(Scan *scan)
	{
//		fp.open("plane_extraction.txt",std::ios::app);
//		if(debug)
//		{
//			fp<<"*****************loadPoints**************************************"<<std::endl;
//		}
		if(scan->point_cloud->empty() || scan->normal_cloud->empty() || scan->pixel_cloud->empty())
		{
			std::cerr<<"load point_cloud, normal_cloud and pixel_cloud before this."<<std::endl;
			return false;
		}

		cells_bottom->clear();

		computeRotationPCA(scan);
		if(debug)
		{
			fp<<"RotationPCA - "<<std::endl;
			fp<<scan->Rotation_PCA<<std::endl;
		}

		Point point_tmp;
		for(int i=0;i<scan->point_cloud->size();i++)
		{
			if(std::isnan(scan->normal_cloud->at(i).normal_x) && 
			   std::isnan(scan->normal_cloud->at(i).normal_y) && 
			   std::isnan(scan->normal_cloud->at(i).normal_z))
				continue;
			if(fabs(scan->point_cloud->at(i).x)<1e-10 && 
			   fabs(scan->point_cloud->at(i).y)<1e-10 && 
			   fabs(scan->point_cloud->at(i).z)<1e-10)
				continue;
			// coordinate in RGB [0,1];
			point_tmp.rgb[0]=(double)scan->point_cloud->at(i).r/255.0;
			point_tmp.rgb[1]=(double)scan->point_cloud->at(i).g/255.0;
			point_tmp.rgb[2]=(double)scan->point_cloud->at(i).b/255.0;
			// coordinate in the camera coordinate system;
			point_tmp.xyz[0]=scan->point_cloud->at(i).x;
			point_tmp.xyz[1]=scan->point_cloud->at(i).y;
			point_tmp.xyz[2]=scan->point_cloud->at(i).z;
			// local plane normal;
			point_tmp.normal[0]=scan->normal_cloud->at(i).normal_x;
			point_tmp.normal[1]=scan->normal_cloud->at(i).normal_y;
			point_tmp.normal[2]=scan->normal_cloud->at(i).normal_z;
			// coordinate in the PPS, after Rotation_PCA; 
			point_tmp.Cartesian2PPS(scan->Rotation_PCA);
			// pixel coordinate;
			point_tmp.u=scan->pixel_cloud->at(i).x; // i/640;
			point_tmp.v=scan->pixel_cloud->at(i).y; // i%640;
			// push the point into the corresponding cells;
			cells_bottom->push_point(point_tmp,i);
		}

		cells_bottom->computeCellAttributes();

		cells_bottom->SortCells();

//		fp.close();
		return true;
	}
	
	void PlaneExtraction::extractPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		fp.open("plane_extraction.txt",std::ios::app);
		if(debug)
		{
			fp<<"*****************extractPlanes**************************************"<<std::endl;
		}

		loadPoints(scan);

		pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
		bool enough_plane = false;
		Eigen::Vector3d tmp_vec3d;
		pcl::PointXYZRGBA tmp_point_pcl;
		pcl::Normal tmp_normal_pcl;
		Plane *tmp_plane;
		Point tmp_point;
		bool have_same_plane=false;
		bool had_parallel_plane=false;

//		std::cout<<"cells_bottom "<<cells_bottom->size()<<std::endl;
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr allplane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_contain_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_contain_plane_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_contain_plane_image (new pcl::PointCloud<pcl::PointXY>);
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr plane_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXY>::Ptr plane_image (new pcl::PointCloud<pcl::PointXY>);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest_final (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normal;
		pcl::ExtractIndices<pcl::PointXY> extract_image;

		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(120);
		seg.setDistanceThreshold(0.01);

//		std::cout<<"cells_bottom "<<cells_bottom->size()<<std::endl;

		// find the cell with the most points inside;
		std::vector<Sorted_Cell>::iterator iter_sorted_cells=cells_bottom->getHighestCell();
		int maxdir = 0;
		int maxnum = 0;
		maxdir=iter_sorted_cells->index;
		maxnum=iter_sorted_cells->num_point;
		if(debug)
		{
			fp<<"maxdir = "<<maxdir<<", maxnum = "<<maxnum<<std::endl;
		}

		cloud_rest_final->clear();
		scan->observed_planes.clear();
		// extracting planes;
		while ( maxnum > min_plane_size && enough_plane == false )
		{
			// save points in cells[maxdir] to cloud_contain_plane;
			extract.setInputCloud (scan->point_cloud);
			extract.setIndices (cells_bottom->getCell(maxdir)->indices);
			extract.setNegative (false);
			extract.filter (*cloud_contain_plane);
			// the corresponding normals to cloud_contain_plane_normal;
			extract_normal.setInputCloud(scan->normal_cloud);
			extract_normal.setIndices(cells_bottom->getCell(maxdir)->indices);
			extract_normal.setNegative(false);
			extract_normal.filter(*cloud_contain_plane_normal);
			// the corresponding pixel_cloud pixel to cloud_contain_plane_image;
			extract_image.setInputCloud (scan->pixel_cloud);
			extract_image.setIndices (cells_bottom->getCell(maxdir)->indices);
			extract_image.setNegative (false);
			extract_image.filter (*cloud_contain_plane_image);
			// enough points in the cells[maxdir];
			if(debug)
			{
				fp<<"================================================="<<std::endl;
				fp<<"maxdir="<<maxdir<<", maxnum="<<maxnum<<std::endl;
			}
			if ( cloud_contain_plane->size() > min_plane_size)
			{
				// estimate plane parameters using cells[maxdir];
				seg.setInputCloud(cloud_contain_plane);
				seg.segment(*inliers_plane, *coefficients_plane);
				// plane equation: ax+by+cz+d=0;
				// make the plane normal point to the origin;
				unifyPlaneDir(coefficients_plane);

				// expand cloud_contain_plane from cells near cell[maxdir];
					//for (int i_cell = 0;i_cell<8;i_cell++)  
					//{
					//	int tmp;
					//	switch(i_cell)
					//	{
					//	case 0:
					//		tmp=maxdir-1;
					//		break;
					//	case 1:
					//		tmp=maxdir+1;
					//		break;
					//	case 2:
					//		tmp=maxdir-bins_theta;
					//		break;
					//	case 3:
					//		tmp=maxdir-bins_theta-1;
					//		break;
					//	case 4:
					//		tmp=maxdir-bins_theta+1;
					//		break;
					//	case 5:
					//		tmp=maxdir+bins_theta;
					//		break;
					//	case 6:
					//		tmp=maxdir+bins_theta-1;
					//		break;
					//	case 7:
					//		tmp=maxdir+bins_theta+1;
					//		break;
					//	default:
					//		break;
					//	}
					//	if(tmp>=0 && tmp<bins_theta*bins_phy) //bins_d=1
					//	{
					//		for(int i=0;i<cells[tmp].num_points;i++)
					//		{
					//			if(dist_point2plane(cells[tmp].points[i].xyz,coefficients_plane)<maxdist_point2plane)
					//			{
					//				tmp_point_pcl.x=cells[tmp].points[i].xyz[0];
					//				tmp_point_pcl.y=cells[tmp].points[i].xyz[1];
					//				tmp_point_pcl.z=cells[tmp].points[i].xyz[2];
					//				tmp_point_pcl.r=cells[tmp].points[i].rgb[0];
					//				tmp_point_pcl.g=cells[tmp].points[i].rgb[1];
					//				tmp_point_pcl.b=cells[tmp].points[i].rgb[2];
					//				cloud_contain_plane->push_back(tmp_point_pcl);
					//				tmp_normal_pcl.normal_x=cells[tmp].points[i].normal[0];
					//				tmp_normal_pcl.normal_y=cells[tmp].points[i].normal[1];
					//				tmp_normal_pcl.normal_z=cells[tmp].points[i].normal[2];
					//				cloud_contain_plane_normal->push_back(tmp_normal_pcl);
					//				tmp_pixel_pcl.x=cells[tmp].points[i].u;
					//				tmp_pixel_pcl.y=cells[tmp].points[i].v;
					//				cloud_contain_plane_image->push_back(tmp_pixel_pcl);
					//			}
					//		}
					//	}
					//}
					//if(debug)
					//{
					//	fp<<"after expanding cloud_contain_plane:"<<cloud_contain_plane->size()<<std::endl;
					//}

				// extract plane in the expanded cloud_contain_plane;
				seg.setInputCloud(cloud_contain_plane);
				seg.segment(*inliers_plane, *coefficients_plane);

				// plane
				// - extracted plane;
				// - points on plane in the camera coordinate system;
				extract.setInputCloud (cloud_contain_plane);
				extract.setIndices (inliers_plane);
				extract.setNegative (false);
				extract.filter (*plane); // plane extracted in the maxdir direction
				extract.setNegative (true);
				extract.filter (*cloud_contain_plane);

				// plane_normal
				// - extracted plane;
				// - local normal of each point on plane;
				extract_normal.setInputCloud (cloud_contain_plane_normal);
				extract_normal.setIndices (inliers_plane);
				extract_normal.setNegative (false);
				extract_normal.filter (*plane_normal);
				extract_normal.setNegative (true);
				extract_normal.filter (*cloud_contain_plane_normal);

				// plane_image
				// - extracted plane;
				// - pixel coordinate corresponding to each point on plane;
				extract_image.setInputCloud (cloud_contain_plane_image);
				extract_image.setIndices (inliers_plane);
				extract_image.setNegative (false);
				extract_image.filter (*plane_image);
				extract_image.setNegative (true);
				extract_image.filter (*cloud_contain_plane_image);

				have_same_plane=false;

				if(plane->size()<=min_plane_size)
				{
					// if the extracted plane is not large enough;
					// then the following "while" will not be activated;
					*cloud_rest_final=*cloud_rest_final+*cloud_contain_plane;
					*cloud_rest_final=*cloud_rest_final+*plane;
				}

				while(plane->size() > min_plane_size && enough_plane == false)
				{
					if (scan->observed_planes.size() < max_plane)
					{
						// tmp_plane
						// - allocate a new plane feaure;
						// - normal, d: from the extraction method;
						// - points, num_points;
						// - avg_pps, cov_pps, avg_rgb and cov_rgb: computed from points;
						tmp_plane=new Plane;
						unifyPlaneDir(coefficients_plane);
//						tmp_plane->points.clear();
						tmp_plane->n[0]=coefficients_plane->values[0];
						tmp_plane->n[1]=coefficients_plane->values[1];
						tmp_plane->n[2]=coefficients_plane->values[2];
						tmp_plane->d=coefficients_plane->values[3];
						tmp_plane->inliers=*inliers_plane;
						tmp_plane->scan=scan;
//						for(int i_point=0;i_point<plane->size();i_point++)
//						{
//							tmp_point.xyz[0]=plane->at(i_point).x;
//							tmp_point.xyz[1]=plane->at(i_point).y;
//							tmp_point.xyz[2]=plane->at(i_point).z;
//							tmp_point.rgb[0]=plane->at(i_point).r/255.0;
//							tmp_point.rgb[1]=plane->at(i_point).g/255.0;
//							tmp_point.rgb[2]=plane->at(i_point).b/255.0;
//							tmp_point.normal[0]=plane_normal->at(i_point).normal_x;
//							tmp_point.normal[1]=plane_normal->at(i_point).normal_y;
//							tmp_point.normal[2]=plane_normal->at(i_point).normal_z;
//							tmp_point.u=plane_image->at(i_point).x;
//							tmp_point.v=plane_image->at(i_point).y;
//							tmp_point.Cartesian2PPS(scan->Rotation_PCA);
//							tmp_plane->points.push_back(tmp_point);
////							tmp_plane->num_points=tmp_plane->points.size();
//						}
						// compute avg_pps, cov_pps, avg_rgb and cov_rgb of tmp_plane;
//						computePlaneAvgCov(tmp_plane);
//						tmp_plane->pG=tmp_plane->avg_xyz;
						if(debug)
						{
							fp<<"allocate a new plane - "<<std::endl;
							fp<<"\tnormal="<<tmp_plane->n.transpose()<<std::endl;
							fp<<"\td="<<tmp_plane->d<<std::endl;
							fp<<"\tplane size - "<<tmp_plane->inliers.indices.size()<<std::endl;
						}
					
						// if "planes" are not empty, i.e., there are already extracted planes;
						// test if current plane is the same with any one in "planes";
						// if so, fuse the same planes;
						if(scan->observed_planes.size()>0)
						{
							for(int i_plane=0;i_plane<scan->observed_planes.size();i_plane++)
							{
								// angle(n1,n2)<thres_angle, |d1-d2|<thres_dist, delta_color<thres_color;
								double tmp_cos_angle=tmp_plane->n.transpose()*scan->observed_planes[i_plane]->n;
								if(tmp_cos_angle>0.9999)
								{
									tmp_cos_angle=1.0;
								}
								Eigen::Vector3d tmp_delta_rgb=tmp_plane->centroid_color-scan->observed_planes[i_plane]->centroid_color;
								if(debug)
								{
									fp<<"\tsimilarity with "<<i_plane<<"th plane - "<<tmp_cos_angle<<", "<<acos(tmp_cos_angle)<<", "
										     <<fabs(tmp_plane->d-scan->observed_planes[i_plane]->d)<<", "
											 <<tmp_delta_rgb.norm()<<std::endl;
								}
								if(acos(tmp_cos_angle)<thres_angle 
								   && fabs(tmp_plane->d-scan->observed_planes[i_plane]->d)<thres_dist 
								   && tmp_delta_rgb.norm()<thres_color)
								{
									have_same_plane=true;
									fusePlanes(tmp_plane,scan->observed_planes[i_plane]);
//									tmp_plane->release();
									delete tmp_plane;
//									*allplane = *plane + *allplane;
									if(debug)
									{
										fp<<"\tsame with the "<<i_plane<<"th plane, after fusion:"<<std::endl;
										fp<<"\t\tnormal="<<scan->observed_planes[i_plane]->n.transpose()<<std::endl;
										fp<<"\t\td="<<scan->observed_planes[i_plane]->d<<std::endl;
										fp<<"\t\tsize="<<scan->observed_planes[i_plane]->inliers.indices.size()<<std::endl;
									}
									break;
								}
							}
						}

						// if no same plane, add the tmp_plane to "planes";
						if(have_same_plane == false)
						{
							// no similar planes;
							// save current plane tmp_plane to planes;
							tmp_plane->id="plane_"+std::to_string(scan->observed_planes.size());
							scan->observed_planes.push_back(tmp_plane);
//							*allplane = *plane + *allplane;
							if(debug)
							{
								fp<<"the "<<scan->observed_planes.size()-1<<"th plane:"<<std::endl;
								fp<<"\tnormal="<<tmp_plane->n.transpose()<<std::endl;
								fp<<"\td="<<tmp_plane->d<<std::endl;
								fp<<"\tplane size - "<<tmp_plane->inliers.indices.size()<<std::endl;
							}
						}

						// if there are a lot of points left in the cloud_contain_plane;
						// then more planes may be extracted;
						if (cloud_contain_plane->size() > min_plane_size)
						{
							seg.setInputCloud(cloud_contain_plane);
							seg.segment(*inliers_plane, *coefficients_plane);

							extract.setInputCloud (cloud_contain_plane);
							extract.setIndices (inliers_plane);
							extract.setNegative (false);
							extract.filter (*plane);
							extract.setNegative (true);
							extract.filter (*cloud_contain_plane);

							extract_normal.setInputCloud (cloud_contain_plane_normal);
							extract_normal.setIndices (inliers_plane);
							extract_normal.setNegative (false);
							extract_normal.filter (*plane_normal);
							extract_normal.setNegative (true);
							extract_normal.filter (*cloud_contain_plane_normal);

							extract_image.setInputCloud (cloud_contain_plane_image);
							extract_image.setIndices (inliers_plane);
							extract_image.setNegative (false);
							extract_image.filter (*plane_image);
							extract_image.setNegative (true);
							extract_image.filter (*cloud_contain_plane_image);

							have_same_plane=false;

							if (plane->size() > min_plane_size)
							{
								//had_parallel_plane = true;
							}
							else
							{
								*cloud_rest_final=*cloud_rest_final+*plane;
								*cloud_rest_final = *cloud_rest_final + *cloud_contain_plane;
							}
						}
						else
						{
							*cloud_rest_final = *cloud_rest_final + *cloud_contain_plane;
							plane->clear();
						}
					}
					else
					{
						enough_plane = true;
					}		
				}
			}
			else
			{
				*cloud_rest_final=*cloud_rest_final+*cloud_contain_plane;
			}
			if( enough_plane == false )
			{
				iter_sorted_cells--;
				maxdir=iter_sorted_cells->index;
				maxnum=iter_sorted_cells->num_point;
			}
		}
		if(debug)
		{
			fp<<"extracted "<<scan->observed_planes.size()<<" planes"<<std::endl;
			for(int i=0;i<scan->observed_planes.size();i++)
			{
				fp<<"\tnormal="<<scan->observed_planes[i]->n.transpose();
				fp<<", d="<<scan->observed_planes[i]->d<<std::endl;
			}
		}

		for(int i=0;i<scan->observed_planes.size();i++)
		{
			fitPlaneModel(scan->observed_planes[i]);
		}

		if(debug)
		{
			visPlanes(scan,vis);
			vis->spin();
		}

		fp.close();
	}


	void PlaneExtraction::computeRotationPCA(Scan *scan)
	{
		Eigen::Vector3d tmp_vec3d;
		Eigen::Matrix3d tmp_mat3d=Eigen::Matrix3d::Zero();
		int tmp_count=0;
		double tmp_max=0,tmp_min=99999;
		Eigen::Vector3d x,y,z;

		// find the direction with least normal vectors;
		// set this direction as the z axis;
		for(int i=0;i<scan->normal_cloud->size();i++)
		{
			if(std::isnan(scan->normal_cloud->at(i).normal_x) && std::isnan(scan->normal_cloud->at(i).normal_y) && std::isnan(scan->normal_cloud->at(i).normal_z))
				continue;
			else
			{
				tmp_vec3d(0)=scan->normal_cloud->at(i).normal_x;
				tmp_vec3d(1)=scan->normal_cloud->at(i).normal_y;
				tmp_vec3d(2)=scan->normal_cloud->at(i).normal_z;
				tmp_mat3d+=tmp_vec3d*tmp_vec3d.transpose();
				tmp_count++;
			}
		}
		tmp_mat3d=tmp_mat3d*(1.0/tmp_count);
		Eigen::EigenSolver<Eigen::Matrix3d> es(tmp_mat3d);
		for(int i=0;i<3;i++)
		{
			if(es.eigenvalues().real()[i]>tmp_max)
			{
				tmp_max=es.eigenvalues().real()[i];
				y=es.eigenvectors().real().block<3,1>(0,i);
			}
			if(es.eigenvalues().real()[i]<tmp_min)
			{
				tmp_min=es.eigenvalues().real()[i];
				z=es.eigenvectors().real().block<3,1>(0,i);
			}
		}
		x=y.cross(z);
		x.normalize();
		// Rotation_PCA
		// - rotation from the original coordinate to the coordinate where 
		// - the z axis pointing to the direction with least normals;
		scan->Rotation_PCA.block<1,3>(0,0)=x.transpose();
		scan->Rotation_PCA.block<1,3>(1,0)=y.transpose();
		scan->Rotation_PCA.block<1,3>(2,0)=z.transpose();
	}

	// unifyPlaneDir
	// - make the plane normal point to the origin;
	void PlaneExtraction::unifyPlaneDir(pcl::ModelCoefficients::Ptr plane)
	{
		double a=plane->values[0];
		double b=plane->values[1];
		double c=plane->values[2];
		double d=plane->values[3];
		bool flag_reverse;
		if ( abs(a)>=abs(b) && abs(a)>=abs(c) )
		{
			float x = (b+c-d)/a;
			if ( (a*(-x)+b+c)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if ( abs(b)>=abs(a) && abs(b)>=abs(c) )
		{
			float y = (a+c-d)/b;
			if ( (b*(-y)+a+c)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if ( abs(c)>=abs(b) && abs(c)>=abs(a) )
		{
			float z = (a+b-d)/c;
			if ( (c*(-z)+b+a)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if(flag_reverse)
		{
			plane->values[0]=-plane->values[0];
			plane->values[1]=-plane->values[1];
			plane->values[2]=-plane->values[2];
			plane->values[3]=-plane->values[3];
		}
	}

	// dist_point2plane
	// - compute the vertical distance from a point to a plane;
	double PlaneExtraction::dist_point2plane(Eigen::Vector3d point, pcl::ModelCoefficients::Ptr plane)
	{
		//if (!pcl::isFinite(x) && !pcl::isFinite(x) && !pcl::isFinite(x))
		return abs(point[0]*plane->values[0]+point[1]*plane->values[1]+point[2]*plane->values[2]+plane->values[3])/sqrt(plane->values[0]*plane->values[0]+plane->values[1]*plane->values[1]+plane->values[2]*plane->values[2]);
	}

	void PlaneExtraction::fusePlanes(Plane *tmp_plane, Plane *fuse)
	{
		for(std::vector<int>::iterator it=tmp_plane->inliers.indices.begin();it!=tmp_plane->inliers.indices.end();++it)
		{
			fuse->inliers.indices.push_back(*it);
		}
	}
}
