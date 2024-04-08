/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-18 08:52
#
# Filename:		line_extraction.cpp
#
# Description: 
#
************************************************/

#include "features/line_extraction.h"

namespace ulysses
{
	using namespace std;
	using namespace cv;
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
	double LineExtraction::extract(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug) fp.open("line_extraction.txt",std::ios::out);
		int width=scan->imgRGB().cols;
		int height=scan->imgRGB().rows;
		cv::Mat img;
		cv::cvtColor(scan->imgRGB(),img,cv::COLOR_RGB2GRAY);
		cv::Mat img_org=scan->imgRGB().clone();
//		cv::imshow("img",img); 
//		cv::waitKey(0);
//		cv::imshow("img",img_org); 
//		cv::waitKey(0);

//		line_descriptor->GetLineDescriptor(img,scan->key_lines);
//		scan->key_lines_id.resize(scan->key_lines.size(),"");

//		double ww1, ww2;
//		int lineIDLeft;
//		int lineIDRight;
//		int lowest1 = 0, highest1 = 255;
//		int range1 = (highest1 - lowest1) + 1;
//		std::vector<unsigned int> r1(scan->key_lines.size()), g1(scan->key_lines.size()), b1(scan->key_lines.size()); //the color of lines
//		cv::Point startPoint;
//		cv::Point endPoint;
//
//		for (unsigned int i = 0; i < scan->key_lines.size(); i++)
//		{
//			r1[i] = lowest1 + int(rand() % range1);
//			g1[i] = lowest1 + int(rand() % range1);
//			b1[i] = 255 - r1[i];
//			ww1 = 0.2 * (rand() % 5);
//			ww2 = 1 - ww1;
//			char buf[10];
//			sprintf(buf, "%d ", i);
//			startPoint = cv::Point(int(scan->key_lines[i][0].startPointX), int(scan->key_lines[i][0].startPointY));
//			endPoint = cv::Point(int(scan->key_lines[i][0].endPointX), int(scan->key_lines[i][0].endPointY));
//			cv::line(img_org, startPoint, endPoint, CV_RGB(r1[i], g1[i], b1[i]), 4, cv::LINE_AA, 0);
//			//		imshow("LinesInImage1.png",leftColorImage);
//			//		imshow("LinesInImage2.png",rightColorImage);
//			//		waitKey(0);
//		}
//		cv::imshow("img",img_org); 
//		cv::waitKey(0);
//

		double *image;
		image=(double*)calloc((size_t)(img.cols*img.rows),sizeof(double));
		for(int y=0;y<img.rows;y++)
		for(int x=0;x<img.cols;x++)
		{ image[x+y*img.cols]=(double)img.at<unsigned char>(y,x); }
		int X,Y,n,dim=7;
		X=img.cols;
		Y=img.rows;
		double * segs;
		segs = LineSegmentDetection(&n,image,X,Y,0.8,0.6,2.0,22.5,0.0,0.7,1024,NULL,NULL,NULL);
		scan->key_lines_id.resize(n,"");

//		double ww1, ww2;
//		int lineIDLeft;
//		int lineIDRight;
//		int lowest1 = 0, highest1 = 255;
//		int range1 = (highest1 - lowest1) + 1;
//		std::vector<unsigned int> r1(n), g1(n), b1(n); //the color of lines
//		cv::Point startPoint;
//		cv::Point endPoint;
//
//		for (unsigned int i = 0; i < n; i++)
//		{
//			r1[i] = lowest1 + int(rand() % range1);
//			g1[i] = lowest1 + int(rand() % range1);
//			b1[i] = 255 - r1[i];
//			ww1 = 0.2 * (rand() % 5);
//			ww2 = 1 - ww1;
//			char buf[10];
//			sprintf(buf, "%d ", i);
//			startPoint = cv::Point(int(segs[i*dim]), int(segs[i*dim+1]));
//			endPoint = cv::Point(int(segs[i*dim+2]), int(segs[i*dim+3]));
//			cv::line(img_org, startPoint, endPoint, CV_RGB(r1[i], g1[i], b1[i]), 4, cv::LINE_AA, 0);
//			//		imshow("LinesInImage1.png",leftColorImage);
//			//		imshow("LinesInImage2.png",rightColorImage);
//			//		waitKey(0);
//		}
//		cv::imshow("img",img_org); 
//		cv::waitKey(0);

		if(debug) fp<<"LSD n="<<n<<std::endl;
		for(int i=0;i<n;i++)
//		if(debug) fp<<"key_lines.size="<<scan->key_lines.size()<<endl;
//		for(int i=0;i<scan->key_lines.size();i++)
		{
			if(debug) fp<<i<<"--------------------------------"<<endl;
//			cout<<i<<"--------------------------------"<<endl;
			std::list<linePoint*> line_points;

//			double start_point_x=fabs(scan->key_lines[i][0].startPointX);
//			double start_point_y=fabs(scan->key_lines[i][0].startPointY);
//			double end_point_x = fabs(scan->key_lines[i][0].endPointX);
//			double end_point_y = fabs(scan->key_lines[i][0].endPointY);

			double start_point_x=segs[i*dim];
			double start_point_y=segs[i*dim+1];
			double end_point_x=segs[i*dim+2];
			double end_point_y=segs[i*dim+3];

			if(debug)
			{
				fp<<start_point_x<<", "<<start_point_y<<endl;
				fp<<end_point_x<<", "<<end_point_y<<endl;
//				cout<<start_point_x<<", "<<start_point_y<<endl;
//				cout<<end_point_x<<", "<<end_point_y<<endl;
			}


			double dx=fabs(start_point_x-end_point_x);
			double dy=fabs(start_point_y-end_point_y);
			double x1,y1,x2,y2; // x1<=x2;
			int x,y;
			Eigen::Vector3d pt;
			if(fabs(dx)>fabs(dy))
			{
				if(start_point_x<end_point_x)
				{ x1=start_point_x;   y1=start_point_y; x2=end_point_x; y2=end_point_y; }
				else { x1=end_point_x; y1=end_point_y; x2=start_point_x;   y2=start_point_y; }

				dx=x2-x1, dy=y2-y1;
				for(x=(int)x1;x<=(int)x2;x++)
				{
					double a=((double)x-x1)/dx;
					y=(int)(y1+a*dy);
//					if(debug) fp<<"(x,y)=("<<x<<","<<y<<")"<<endl;
					if(x>=width || y>=height) continue;
					// now (x,y) is a point on the line;
					if(fabs(scan->points()->at(x,y).z)<1e-4) continue;
					else 
					{
						pt=Transform::point2eigen(scan->points()->at(x,y));
						line_points.push_back(new linePoint(y*width+x,pt));
						if(debug) fp<<"(x,y)=("<<x<<","<<y<<") - "<<pt.transpose()<<endl;
					}
				}
			}
			else // (fabs(dx)<fabs(dy))
			{
				if(start_point_y<end_point_y)
				{ x1=start_point_x; y1=start_point_y; x2=end_point_x; y2=end_point_y; }
				else { x1=end_point_x; y1=end_point_y; x2=start_point_x; y2=start_point_y; }

				dx=x2-x1, dy=y2-y1;
				for(y=(int)y1;y<=(int)y2;y++)
				{
					double a=((double)y-y1)/dy;
					x=(int)(x1+a*dx);
//					if(debug) fp<<"(x,y)=("<<x<<","<<y<<")"<<endl;
					if(x>=width || y>=height) continue;
					// now (x,y) is a point on the line;
					if(fabs(scan->points()->at(x,y).z)<1e-4) continue;
					else 
					{
						pt=Transform::point2eigen(scan->points()->at(x,y));
						line_points.push_back(new linePoint(y*width+x,pt));
						if(debug) fp<<"(x,y)=("<<x<<","<<y<<") - "<<pt.transpose()<<endl;
					}
				}
			}
			if(debug) fp<<"line_points "<<line_points.size()<<endl;

			double max=1;
			while(max>break_dist)
			{
				if(line_points.size()<5) break;
				if(debug) fp<<"**************"<<endl;
				max=DBL_MIN;
//				cout<<"line points: "<<line_points.size()<<endl;
//				cout<<"max="<<max<<endl;
				for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
				{
					if(line_points.size()<5) break;
					std::list<linePoint*>::iterator it1,it2;
					it1=std::prev(it); it2=std::next(it);
//					cout<<(*it)->point.transpose()<<endl;
					if(it==line_points.begin())
					{
						(*it)->setQuality(*it2);
//						cout<<"begin"<<endl;
					}
					else if(it2==line_points.end())
					{
						(*it)->setQuality(*it1);
//						cout<<"end"<<endl;
					}
					else 
					{
						(*it)->setQuality(*it1,*it2);
//						cout<<"otherwise"<<endl;
					}
					if(debug) fp<<(*it)->quality<<endl;
					if((*it)->quality>max) max=(*it)->quality;
					if((*it)->quality>break_dist)
					{
//						cout<<"delete"<<endl;
						delete *it;
						it=line_points.erase(it);
						it--;
					}
				}
				if(debug) fp<<"**************"<<endl;
			}

			if(debug) fp<<"line_points 1 "<<line_points.size()<<endl;
			if(line_points.size()<5) 
			{
				for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
				{
					delete *it;
				}
				continue;
			}

			Line* line=new Line;
			line->ptr_points=scan->points();
			line->length=0;
			line->idx_lbd=i;

			for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
			{
				line->indices.push_back((*it)->index);
				if(it!=line_points.begin())
				{
					(*it)->setQuality(*std::prev(it));
					line->length+=(*it)->quality;
				}
				if(it==line_points.begin()) line->end_point_1=(*it)->point;
				if(std::next(it)==line_points.end()) line->end_point_2=(*it)->point;
			}
			for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
			{
				delete *it;
			}
			line_points.clear();

			pt=line->end_point_1-line->end_point_2;
			if(debug) fp<<"pt.norm()= "<<pt.norm()<<endl;
			if(debug) fp<<"line->length= "<<line->length<<endl;
			if(debug) fp<<"line->indices.size()= "<<line->indices.size()<<endl;
			if(line->indices.size()>inliers && line->length<pt.norm()*2.0)// && fabs(line->length)<1e-4)
			{
				if(debug) fp<<line->indices.size()<<endl;
				generatePlucker(line);
				if(debug) fp<<line<<endl;
				computeLineInfo(line);
				Feature *feature=new Feature(line);
				scan->addFeature(feature);
				scan->key_lines_id[i]=feature->ID();
				if(debug) fp<<feature<<endl;

//				r1[i] = lowest1 + int(rand() % range1);
//				g1[i] = lowest1 + int(rand() % range1);
//				b1[i] = 255 - r1[i];
//				ww1 = 0.2 * (rand() % 5);
//				ww2 = 1 - ww1;
//				char buf[10];
//				sprintf(buf, "%d ", i);
//				startPoint = cv::Point(int(scan->key_lines[i][0].startPointX), int(scan->key_lines[i][0].startPointY));
//				endPoint = cv::Point(int(scan->key_lines[i][0].endPointX), int(scan->key_lines[i][0].endPointY));
//				cv::line(img_org, startPoint, endPoint, CV_RGB(r1[i], g1[i], b1[i]), 4, cv::LINE_AA, 0);
//				cv::imwrite(std::to_string(scan->time())+".png",img_org);
//				cv::imshow("img",img_org); 
//				cv::waitKey(0);
//
//				vis->removeAllPointClouds();
//				vis->removeAllCoordinateSystems();
//				scan->vis(vis);
//				feature->vis(vis,255,0,0);
//				vis->spin();

			}
			else 
			{
				if(debug) fp<<"delete"<<endl;
				delete line;
			}

		}

		if(debug) fp<<"==============================================================================="<<endl;
//		if(debug) cout<<"==============================================================================="<<endl;


		img.release();
//		img1.release();
//		cout<<"img.release()"<<endl;
		img_org.release();
//		img_org1.release();
//		cout<<"img_org.release()"<<endl;
//		free( (void *) image );
//		free( (void *) segs );

		if(debug)
		{
			fp<<"scan "<<std::fixed<<scan->time()<<endl;
//			cout<<"scan "<<std::fixed<<scan->time()<<endl;
		}
//		fp<<"features "<<scan->sizeFeature()<<endl;
//		std::vector<double> dist_min; dist_min.resize(scan->sizeFeature(),DBL_MAX);
//		std::vector<iterFeature> idx; idx.resize(scan->sizeFeature(),scan->endFeature());
		std::map<std::string,double> dist_min;
		std::map<std::string,std::string> idx;
		int i=0;
		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++,i++)
		{
			dist_min[it1->first]=DBL_MAX;
			idx[it1->first]="";
			for(iterFeature it2=scan->beginFeature();it2!=scan->endFeature();it2++)
			{
				if(it1->second->Type()!=LINE || it2->second->Type()!=LINE || it1==it2) continue;

				Line *l1=it1->second->line();
				Line *l2=it2->second->line();
				double dist=l1->dist2line(l2);
				double dir=it1->second->delta_dir(it2->second);
				if(debug)
				{
					fp<<it1->second<<endl;
					fp<<it2->second<<endl;
					fp<<"dist="<<dist<<endl;
					fp<<"dir="<<dir<<endl;
				}
				if(dist<dist_min[it1->first] && dir<THRES_RAD)
				{
					dist_min[it1->first]=dist;
					idx[it1->first]=it2->first;
					if(debug)
					{
						fp<<"\tdist_min="<<dist_min[it1->first]<<endl;
						fp<<"\tidx= "<<idx[it1->first]<<endl;
					}
				}
				if(debug) fp<<endl;

//				if(it1->second->line()->v.cross(it2->second->line()->v).norm()<THRES_RAD
//					&& it1->second->line()->u.normalized().cross(it2->second->line()->u.normalized()).norm()<THRES_RAD
//					&& fabs(it1->second->line()->u.norm()-it2->second->line()->u.norm())<THRES_DIST)
//				{
//					it1->second->line()->indices.insert(it1->second->line()->indices.end(),
//						it2->second->line()->indices.begin(),it2->second->line()->indices.end());
//					generatePlucker(it1->second->line());
//					computeLineInfo(it1->second->line());
//					delete it2->second;
//					it2=scan->eraseFeature(it2); it2--;
//				}
			}
		}

		if(debug) fp<<endl<<"fusing"<<endl;
		i=0;
		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++,i++)
		{
			if(!idx[it1->first].empty() && dist_min[it1->first]<THRES_DIST)
			{
				Feature* f=scan->findFeature(idx[it1->first]);
				if(f==0) continue;
				Line *l1=it1->second->line();
				Line *l2=f->line();
				if(debug)
				{
					fp<<it1->second<<endl;
					fp<<f<<endl;
					fp<<"dist="<<dist_min[it1->first]<<endl;
					fp<<"dir="<<it1->second->delta_dir(f)<<endl;
					fp<<endl;
				}
				l1->indices.insert(l1->indices.end(),l2->indices.begin(),l2->indices.end());
				generatePlucker(l1);
				computeLineInfo(l1);
				scan->key_lines_id[l2->idx_lbd]=it1->first;
				for(int j=0;j<scan->key_lines_id.size();j++)
				{
					if(scan->key_lines_id[j]==idx[it1->first])
						scan->key_lines_id[j]="";
				}
				delete f;
				scan->eraseFeature(idx[it1->first]);
			}
		}

		if(debug)
		{
			fp<<"==============================================================================="<<endl;
			fp.close();
		}
		return EXIT_SUCCESS;
	}

	double LineExtraction::extract_lbd(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug) fp.open("line_extraction_lbd.txt",std::ios::out);
		int width=scan->imgRGB().cols;
		int height=scan->imgRGB().rows;
		cv::Mat img;
		cv::cvtColor(scan->imgRGB(),img,cv::COLOR_RGB2GRAY);
		cv::Mat img_org=scan->imgRGB().clone();
//		cv::imshow("img",img); 
//		cv::waitKey(0);
//		cv::imshow("img",img_org); 
//		cv::waitKey(0);

		line_descriptor->GetLineDescriptor(img,scan->key_lines);
		scan->key_lines_id.resize(scan->key_lines.size(),"");

		double ww1, ww2;
		int lineIDLeft;
		int lineIDRight;
		int lowest1 = 0, highest1 = 255;
		int range1 = (highest1 - lowest1) + 1;
		std::vector<unsigned int> r1(scan->key_lines.size()), g1(scan->key_lines.size()), b1(scan->key_lines.size()); //the color of lines
		cv::Point startPoint;
		cv::Point endPoint;

		for (unsigned int i = 0; i < scan->key_lines.size(); i++)
		{
			r1[i] = lowest1 + int(rand() % range1);
			g1[i] = lowest1 + int(rand() % range1);
			b1[i] = 255 - r1[i];
			ww1 = 0.2 * (rand() % 5);
			ww2 = 1 - ww1;
			char buf[10];
			sprintf(buf, "%d ", i);
			startPoint = cv::Point(int(scan->key_lines[i][0].startPointX), int(scan->key_lines[i][0].startPointY));
			endPoint = cv::Point(int(scan->key_lines[i][0].endPointX), int(scan->key_lines[i][0].endPointY));
			cv::line(img_org, startPoint, endPoint, CV_RGB(r1[i], g1[i], b1[i]), 4, cv::LINE_AA, 0);
			//		imshow("LinesInImage1.png",leftColorImage);
			//		imshow("LinesInImage2.png",rightColorImage);
			//		waitKey(0);
		}
		cv::imshow("img",img_org); 
		cv::waitKey(0);


//		if(debug) fp<<"LSD n="<<n<<std::endl;
//		for(int i=0;i<n;i++)
		if(debug) fp<<"key_lines.size="<<scan->key_lines.size()<<endl;
		for(int i=0;i<scan->key_lines.size();i++)
		{
			if(debug) fp<<i<<"--------------------------------"<<endl;
//			cout<<i<<"--------------------------------"<<endl;
			std::list<linePoint*> line_points;

			double start_point_x=fabs(scan->key_lines[i][0].startPointX);
			double start_point_y=fabs(scan->key_lines[i][0].startPointY);
			double end_point_x = fabs(scan->key_lines[i][0].endPointX);
			double end_point_y = fabs(scan->key_lines[i][0].endPointY);

			if(debug)
			{
				fp<<start_point_x<<", "<<start_point_y<<endl;
				fp<<end_point_x<<", "<<end_point_y<<endl;
//				cout<<start_point_x<<", "<<start_point_y<<endl;
//				cout<<end_point_x<<", "<<end_point_y<<endl;
			}


			double dx=fabs(start_point_x-end_point_x);
			double dy=fabs(start_point_y-end_point_y);
			double x1,y1,x2,y2; // x1<=x2;
			int x,y;
			Eigen::Vector3d pt;
			if(fabs(dx)>fabs(dy))
			{
				if(start_point_x<end_point_x)
				{ x1=start_point_x;   y1=start_point_y; x2=end_point_x; y2=end_point_y; }
				else { x1=end_point_x; y1=end_point_y; x2=start_point_x;   y2=start_point_y; }

				dx=x2-x1, dy=y2-y1;
				for(x=(int)x1;x<=(int)x2;x++)
				{
					double a=((double)x-x1)/dx;
					y=(int)(y1+a*dy);
//					if(debug) fp<<"(x,y)=("<<x<<","<<y<<")"<<endl;
					if(x>=width || y>=height) continue;
					// now (x,y) is a point on the line;
					if(fabs(scan->points()->at(x,y).z)<1e-4) continue;
					else 
					{
						pt=Transform::point2eigen(scan->points()->at(x,y));
						line_points.push_back(new linePoint(y*width+x,pt));
						if(debug) fp<<"(x,y)=("<<x<<","<<y<<") - "<<pt.transpose()<<endl;
					}
				}
			}
			else // (fabs(dx)<fabs(dy))
			{
				if(start_point_y<end_point_y)
				{ x1=start_point_x; y1=start_point_y; x2=end_point_x; y2=end_point_y; }
				else { x1=end_point_x; y1=end_point_y; x2=start_point_x; y2=start_point_y; }

				dx=x2-x1, dy=y2-y1;
				for(y=(int)y1;y<=(int)y2;y++)
				{
					double a=((double)y-y1)/dy;
					x=(int)(x1+a*dx);
//					if(debug) fp<<"(x,y)=("<<x<<","<<y<<")"<<endl;
					if(x>=width || y>=height) continue;
					// now (x,y) is a point on the line;
					if(fabs(scan->points()->at(x,y).z)<1e-4) continue;
					else 
					{
						pt=Transform::point2eigen(scan->points()->at(x,y));
						line_points.push_back(new linePoint(y*width+x,pt));
						if(debug) fp<<"(x,y)=("<<x<<","<<y<<") - "<<pt.transpose()<<endl;
					}
				}
			}
			if(debug) fp<<"line_points "<<line_points.size()<<endl;

			double max=1;
			while(max>break_dist)
			{
				if(line_points.size()<5) break;
				if(debug) fp<<"**************"<<endl;
				max=DBL_MIN;
//				cout<<"line points: "<<line_points.size()<<endl;
//				cout<<"max="<<max<<endl;
				for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
				{
					if(line_points.size()<5) break;
					std::list<linePoint*>::iterator it1,it2;
					it1=std::prev(it); it2=std::next(it);
//					cout<<(*it)->point.transpose()<<endl;
					if(it==line_points.begin())
					{
						(*it)->setQuality(*it2);
//						cout<<"begin"<<endl;
					}
					else if(it2==line_points.end())
					{
						(*it)->setQuality(*it1);
//						cout<<"end"<<endl;
					}
					else 
					{
						(*it)->setQuality(*it1,*it2);
//						cout<<"otherwise"<<endl;
					}
					if(debug) fp<<(*it)->quality<<endl;
					if((*it)->quality>max) max=(*it)->quality;
					if((*it)->quality>break_dist)
					{
//						cout<<"delete"<<endl;
						delete *it;
						it=line_points.erase(it);
						it--;
					}
				}
				if(debug) fp<<"**************"<<endl;
			}

			if(debug) fp<<"line_points 1 "<<line_points.size()<<endl;
			if(line_points.size()<5) 
			{
				for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
				{
					delete *it;
				}
				continue;
			}

			Line* line=new Line;
			line->ptr_points=scan->points();
			line->length=0;
			line->idx_lbd=i;

			for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
			{
				line->indices.push_back((*it)->index);
				if(it!=line_points.begin())
				{
					(*it)->setQuality(*std::prev(it));
					line->length+=(*it)->quality;
				}
				if(it==line_points.begin()) line->end_point_1=(*it)->point;
				if(std::next(it)==line_points.end()) line->end_point_2=(*it)->point;
			}
			for(std::list<linePoint*>::iterator it=line_points.begin();it!=line_points.end();it++)
			{
				delete *it;
			}
			line_points.clear();

			pt=line->end_point_1-line->end_point_2;
			if(debug) fp<<"pt.norm()= "<<pt.norm()<<endl;
			if(debug) fp<<"line->length= "<<line->length<<endl;
			if(debug) fp<<"line->indices.size()= "<<line->indices.size()<<endl;
			if(line->indices.size()>inliers && line->length<pt.norm()*2.0)// && fabs(line->length)<1e-4)
			{
				if(debug) fp<<line->indices.size()<<endl;
				generatePlucker(line);
				if(debug) fp<<line<<endl;
				computeLineInfo(line);
				Feature *feature=new Feature(line);
				scan->addFeature(feature);
				scan->key_lines_id[i]=feature->ID();
				if(debug) fp<<feature<<endl;

//				r1[i] = lowest1 + int(rand() % range1);
//				g1[i] = lowest1 + int(rand() % range1);
//				b1[i] = 255 - r1[i];
//				ww1 = 0.2 * (rand() % 5);
//				ww2 = 1 - ww1;
//				char buf[10];
//				sprintf(buf, "%d ", i);
//				startPoint = cv::Point(int(scan->key_lines[i][0].startPointX), int(scan->key_lines[i][0].startPointY));
//				endPoint = cv::Point(int(scan->key_lines[i][0].endPointX), int(scan->key_lines[i][0].endPointY));
//				cv::line(img_org, startPoint, endPoint, CV_RGB(r1[i], g1[i], b1[i]), 4, cv::LINE_AA, 0);
//				cv::imwrite(std::to_string(scan->time())+".png",img_org);
//				cv::imshow("img",img_org); 
//				cv::waitKey(0);
//
//				vis->removeAllPointClouds();
//				vis->removeAllCoordinateSystems();
//				scan->vis(vis);
//				feature->vis(vis,255,0,0);
//				vis->spin();

			}
			else 
			{
				if(debug) fp<<"delete"<<endl;
				delete line;
			}

		}

		if(debug) fp<<"==============================================================================="<<endl;
//		if(debug) cout<<"==============================================================================="<<endl;


		img.release();
//		img1.release();
//		cout<<"img.release()"<<endl;
		img_org.release();
//		img_org1.release();
//		cout<<"img_org.release()"<<endl;
//		free( (void *) image );
//		free( (void *) segs );

		if(debug)
		{
			fp<<"scan "<<std::fixed<<scan->time()<<endl;
//			cout<<"scan "<<std::fixed<<scan->time()<<endl;
		}
//		fp<<"features "<<scan->sizeFeature()<<endl;
//		std::vector<double> dist_min; dist_min.resize(scan->sizeFeature(),DBL_MAX);
//		std::vector<iterFeature> idx; idx.resize(scan->sizeFeature(),scan->endFeature());
		std::map<std::string,double> dist_min;
		std::map<std::string,std::string> idx;
		int i=0;
		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++,i++)
		{
			dist_min[it1->first]=DBL_MAX;
			idx[it1->first]="";
			for(iterFeature it2=scan->beginFeature();it2!=scan->endFeature();it2++)
			{
				if(it1->second->Type()!=LINE || it2->second->Type()!=LINE || it1==it2) continue;

				Line *l1=it1->second->line();
				Line *l2=it2->second->line();
				double dist=l1->dist2line(l2);
				double dir=it1->second->delta_dir(it2->second);
				if(debug)
				{
					fp<<it1->second<<endl;
					fp<<it2->second<<endl;
					fp<<"dist="<<dist<<endl;
					fp<<"dir="<<dir<<endl;
				}
				if(dist<dist_min[it1->first] && dir<THRES_RAD)
				{
					dist_min[it1->first]=dist;
					idx[it1->first]=it2->first;
					if(debug)
					{
						fp<<"\tdist_min="<<dist_min[it1->first]<<endl;
						fp<<"\tidx= "<<idx[it1->first]<<endl;
					}
				}
				if(debug) fp<<endl;

//				if(it1->second->line()->v.cross(it2->second->line()->v).norm()<THRES_RAD
//					&& it1->second->line()->u.normalized().cross(it2->second->line()->u.normalized()).norm()<THRES_RAD
//					&& fabs(it1->second->line()->u.norm()-it2->second->line()->u.norm())<THRES_DIST)
//				{
//					it1->second->line()->indices.insert(it1->second->line()->indices.end(),
//						it2->second->line()->indices.begin(),it2->second->line()->indices.end());
//					generatePlucker(it1->second->line());
//					computeLineInfo(it1->second->line());
//					delete it2->second;
//					it2=scan->eraseFeature(it2); it2--;
//				}
			}
		}

		if(debug) fp<<endl<<"fusing"<<endl;
		i=0;
		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++,i++)
		{
			if(!idx[it1->first].empty() && dist_min[it1->first]<THRES_DIST)
			{
				Feature* f=scan->findFeature(idx[it1->first]);
				if(f==0) continue;
				Line *l1=it1->second->line();
				Line *l2=f->line();
				if(debug)
				{
					fp<<it1->second<<endl;
					fp<<f<<endl;
					fp<<"dist="<<dist_min[it1->first]<<endl;
					fp<<"dir="<<it1->second->delta_dir(f)<<endl;
					fp<<endl;
				}
				l1->indices.insert(l1->indices.end(),l2->indices.begin(),l2->indices.end());
				generatePlucker(l1);
				computeLineInfo(l1);
				scan->key_lines_id[l2->idx_lbd]=it1->first;
				for(int j=0;j<scan->key_lines_id.size();j++)
				{
					if(scan->key_lines_id[j]==idx[it1->first])
						scan->key_lines_id[j]="";
				}
				delete f;
				scan->eraseFeature(idx[it1->first]);
			}
		}

		if(debug)
		{
			fp<<"==============================================================================="<<endl;
			fp.close();
		}
		return EXIT_SUCCESS;
	}

	double LineExtraction::match_lbd(Scan *scan, Scan *ref)
	{
		if(debug) fp.open("line_matching_lbd.txt");
		if(ref==0) return -1;
		ScaleLines key_lines_cur; std::vector<int> idx_cur;
		ScaleLines key_lines_ref; std::vector<int> idx_ref;
		if(debug) fp<<"scan_cur"<<endl;
		for(int i=0;i<scan->key_lines.size();i++)
		{
			if(scan->key_lines_id[i].empty()) continue;
			key_lines_cur.push_back(scan->key_lines[i]);
			idx_cur.push_back(i);
			if(debug) fp<<scan->key_lines_id[i]<<endl;
		}
		if(debug) fp<<"scan_cur"<<endl;
		for(int i=0;i<ref->key_lines.size();i++)
		{
			if(ref->key_lines_id[i].empty()) continue;
			key_lines_ref.push_back(ref->key_lines[i]);
			idx_ref.push_back(i);
			if(debug) fp<<ref->key_lines_id[i]<<endl;
		}
		if(debug) fp<<endl;

		timeval start, end;
		double timeused;
		gettimeofday(&start,NULL);
		std::vector<unsigned int> matchResult;

//		line_match->LineMatching(scan->key_lines,ref->key_lines,matchResult);
		line_match->LineMatching(key_lines_cur,key_lines_ref,matchResult);

		scan->association()=new FeatureAssociation;

//		cv::Mat leftColorImage =scan->imgRGB().clone();
//		cv::Mat rightColorImage=ref->imgRGB().clone();
//		unsigned int imageWidth = leftColorImage.cols; //图片的宽与高
//		unsigned int imageHeight = leftColorImage.rows;
//		cv::Point startPoint;
//		cv::Point endPoint;
//		cv::Point point;
//		srand((unsigned)time(0));
//		int lowest=100, highest=255;
//		int range=(highest-lowest)+1;
//		unsigned int r, g, b; //the color of lines
//
//		double ww1, ww2;
//		int lowest1 = 0, highest1 = 255;
//		int range1 = (highest1 - lowest1) + 1;
//		std::vector<unsigned int> r1(matchResult.size() / 2), g1(matchResult.size() / 2), b1(matchResult.size() / 2); //the color of lines

		int lineIDLeft;
		int lineIDRight;
		for (unsigned int pair = 0; pair < matchResult.size() / 2; pair++)
		{
//			r1[pair] = lowest1 + int(rand() % range1);
//			g1[pair] = lowest1 + int(rand() % range1);
//			b1[pair] = 255 - r1[pair];
//			ww1 = 0.2 * (rand() % 5);
//			ww2 = 1 - ww1;
//			char buf[10];
//			sprintf(buf, "%d ", pair);

			lineIDLeft = matchResult[2 * pair];
			lineIDRight = matchResult[2 * pair + 1];
			int i_cur=idx_cur[lineIDLeft];
			int i_ref=idx_ref[lineIDRight];
			std::string id_cur=scan->key_lines_id[i_cur];
			std::string id_ref=ref->key_lines_id[i_ref];
			if(id_cur.empty() || id_ref.empty()) continue;
			if(debug) fp<<"id_cur="<<id_cur<<", id_ref="<<id_ref<<endl;
			scan->association()->insert(scan->findFeature(id_cur),ref->findFeature(id_ref));
			

//			startPoint = cv::Point(int(scan->key_lines[i_cur][0].startPointX), int(scan->key_lines[i_cur][0].startPointY));
//			endPoint = cv::Point(int(scan->key_lines[i_cur][0].endPointX), int(scan->key_lines[i_cur][0].endPointY));
//			cv::line(leftColorImage, startPoint, endPoint, CV_RGB(r1[pair], g1[pair], b1[pair]), 4, cv::LINE_AA, 0);
//
//			startPoint = cvPoint(int(ref->key_lines[i_ref][0].startPointX), int(ref->key_lines[i_ref][0].startPointY));
//			endPoint = cvPoint(int(ref->key_lines[i_ref][0].endPointX), int(ref->key_lines[i_ref][0].endPointY));
//			cv::line(rightColorImage, startPoint, endPoint, CV_RGB(r1[pair], g1[pair], b1[pair]), 4, cv::LINE_AA, 0);

		}
//		cv::imshow("imgleft", leftColorImage);
//		cv::imshow("imgright", rightColorImage);
//		cv::waitKey();

		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;


//		cv::Mat cvResultColorImage1 = cv::Mat(cv::Size(imageWidth * 2, imageHeight), leftColorImage.type(), 3);
//		cv::Mat cvResultColorImage2 = cv::Mat(cv::Size(imageWidth * 2, imageHeight), leftColorImage.type(), 3);
//		cv::Mat cvResultColorImage = cv::Mat(cv::Size(imageWidth * 2, imageHeight), leftColorImage.type(), 3);
//		cv::Mat roi = cvResultColorImage1(cv::Rect(0, 0, imageWidth, imageHeight));
//		cv::resize(leftColorImage, roi, roi.size(), 0, 0, 0);
//
//		cv::Mat roi2 = cvResultColorImage1(cv::Rect(imageWidth, 0, imageWidth, imageHeight));
//		cv::resize(rightColorImage, roi2, roi2.size(), 0, 0, 0);
//		cvResultColorImage1.copyTo(cvResultColorImage2);
//
//		for (unsigned int pair = 0; pair < matchResult.size() / 2; pair++)
//		{
//			lineIDLeft = matchResult[2 * pair];
//			lineIDRight = matchResult[2 * pair + 1];
//			int i_cur=idx_cur[lineIDLeft];
//			int i_ref=idx_ref[lineIDRight];
//			std::string id_cur=scan->key_lines_id[i_cur];
//			std::string id_ref=ref->key_lines_id[i_ref];
//			if(id_cur.empty() || id_ref.empty()) continue;
//
//			startPoint = cv::Point(int(scan->key_lines[i_cur][0].startPointX), int(scan->key_lines[i_cur][0].startPointY));
//			endPoint = cv::Point(int(ref->key_lines[i_ref][0].startPointX + imageWidth), int(ref->key_lines[i_ref][0].startPointY));
//			cv::line(cvResultColorImage2, startPoint, endPoint, CV_RGB(r1[pair], g1[pair], b1[pair]), 1, cv::LINE_AA, 0);
//		}
//		cv::addWeighted(cvResultColorImage1, 0.5, cvResultColorImage2, 0.5, 0.0, cvResultColorImage, -1);
//
//		cv::imshow("result.png", cvResultColorImage);
//		cv::waitKey();
//		cvResultColorImage.release();
//		cvResultColorImage1.release();
//		cvResultColorImage2.release();
//		leftColorImage.release();
//		rightColorImage.release();
//		roi.release();
//		roi2.release();

		if(debug) fp.close();
		return timeused;
	}

	double LineExtraction::extractLines(Scan *scan)
	{
		if(debug) fp.open("line_extraction.txt",std::ios::app);
		if(debug) fp<<std::endl<<"*****************************************************"<<std::endl;

		timeval start, end;
		double timeused;
		gettimeofday(&start,NULL);

		edge_extraction->extractEdgePoints(scan);
//		edge_extraction->visEdgePoints(scan,vis);
//		vis->spin();

		extractLinesHough(scan,1.0);

		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{
			if(it->second->Type()==LINE)
			{
				generatePlucker(it->second->line());
				computeLineInfo(it->second->line());
				if(debug) fp<<it->second->line()<<std::endl;
			}
		}

		for(iterFeature it1=scan->beginFeature();it1!=scan->endFeature();it1++)
		{
			for(iterFeature it2=scan->beginFeature();it2!=scan->endFeature();it2++)
			{
				if(it1->second->Type()!=LINE || it2->second->Type()!=LINE || it1==it2) continue;
				if(it1->second->line()->v.cross(it2->second->line()->v).norm()<THRES_RAD 
					&& it1->second->line()->u.normalized().cross(it2->second->line()->u.normalized()).norm()<0.02
					&& fabs(it1->second->line()->u.norm()-it2->second->line()->u.norm())<0.05)
				{
					it1->second->line()->points.insert(it1->second->line()->points.end(),
						it2->second->line()->points.begin(),it2->second->line()->points.end());
					generatePlucker(it1->second->line());
					computeLineInfo(it1->second->line());
					delete it2->second;
					it2=scan->eraseFeature(it2); it2--;
				}

			}
		}

		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//		int id=0;
//		if(debug) fp<<"\noccluding lines: "<<scan->lines_occluding.size()<<std::endl;
//		for(std::list<Line*>::iterator it_line=scan->lines_occluding.begin();it_line!=scan->lines_occluding.end();it_line++)
//		{
//			(*it_line)->id="line_"+std::to_string(id);
//			generatePlucker(*it_line);
//			computeLineInfo(*it_line);
//			if(debug) fp<<(*it_line)->id<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<std::endl;
//			// associate occluding lines with occluded lines;
//			Eigen::Vector3d u=(*it_line)->u;
//			Eigen::Vector3d v=(*it_line)->v;
//			double min=DBL_MAX;
//			std::list<Line*>::iterator idx_min=scan->lines_occluding.end();
//			for(std::list<Line*>::iterator it_occluded=scan->lines_occluded.begin();
//										   it_occluded!=scan->lines_occluded.end();it_occluded++)
//			{
//				Eigen::Vector3d u_pi=(*it_occluded)->u;
//				u.normalize();
//				u_pi.normalize();
//				double dist=u.transpose()*u_pi;
//				dist=fabs(dist);
//				dist=acos(dist)*180.0/M_PI;
//				if(dist<min)
//				{
//					min=dist;
//					idx_min=it_occluded;
//				}
//			}
//			if(idx_min!=scan->lines_occluding.end())
//			{
////				double d=fabs((*idx_min)->u.norm()/(*it_line)->u.norm());
//	//			std::cout<<"min="<<min<<" d="<<d<<std::endl;
//				if(min<thres_line2shadow)// && d>1.0)
//				{
//					(*it_line)->occluded=*idx_min;
//					if(debug) fp<<"\toccluded\t"<<(*it_line)->occluded->u.transpose()<<std::endl;
//	//				std::cout<<"acos(u^T*u_pi) = "<<min<<std::endl;
//	//				displayLineShadow(scan,*idx_min,*it_line,vis);
//	//				vis->spin();
//				}
//			}
//		}

		if(debug) fp.close();
		return timeused;

//		if(debug)
//		{
//			visLines(scan,vis);
//			vis->spin();
//		}
	}

	void LineExtraction::generatePlucker(Line *line)
	{
		// refine the extracted line using the inlier points;
		const int N=line->indices.size();
		Eigen::Matrix3d pG_skew=Eigen::Matrix3d::Zero();
		for(int i=0;i<line->indices.size();i++)
		{
			pG_skew+=Transform::skew_sym(Transform::point2eigen(line->ptr_points->at(line->indices[i])));
		}
		pG_skew/=N;
		Eigen::Vector3d pG=Transform::skew_sym_inv(pG_skew);
		Eigen::Matrix3d P=Eigen::Matrix3d::Zero();
		for(int i=0;i<line->indices.size();i++)
		{
			Eigen::Vector3d tmp=Transform::point2eigen(line->ptr_points->at(line->indices[i]))-pG;
			P+=-Transform::skew_sym(tmp)*Transform::skew_sym(tmp);
		}
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
		es.compute(P);
		Eigen::Vector3d eigenvalues=es.eigenvalues();
		Eigen::Matrix3d eigenvectors=es.eigenvectors();

		double ev_min=DBL_MAX;
		int idx=-1;
		for(int j=0;j<3;j++)
		{
			if(eigenvalues(j)<ev_min)
			{
				ev_min=eigenvalues(j);
				idx=j;
			}
		}

		line->v=eigenvectors.block<3,1>(0,idx);
		line->u=pG_skew*line->v;
	}
//	{
//		// refine the extracted line using the inlier points;
//		const int N=line->points.size();
//		Eigen::Matrix3d pG_skew=Eigen::Matrix3d::Zero();
//		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
//		{
//			pG_skew+=Transform::skew_sym((*it)->xyz);
//		}
//		pG_skew/=N;
//		Eigen::Vector3d pG=Transform::skew_sym_inv(pG_skew);
//		Eigen::Matrix3d P=Eigen::Matrix3d::Zero();
//		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
//		{
//			Eigen::Vector3d tmp=(*it)->xyz-pG;
//			P+=-Transform::skew_sym(tmp)*Transform::skew_sym(tmp);
//		}
//
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
//		es.compute(P);
//		Eigen::Vector3d eigenvalues=es.eigenvalues();
//		Eigen::Matrix3d eigenvectors=es.eigenvectors();
//
//		double ev_min=DBL_MAX;
//		int idx=-1;
//		for(int j=0;j<3;j++)
//		{
//			if(eigenvalues(j)<ev_min)
//			{
//				ev_min=eigenvalues(j);
//				idx=j;
//			}
//		}
//
//		line->v=eigenvectors.block<3,1>(0,idx);
//		line->u=pG_skew*line->v;
//
//	}

	/*
	void LineExtraction::generatePlucker1(Line *line)
	{
		if(debug) fp<<"\n------generatePlucker"<<std::endl;
		// refine the extracted line using the inlier points;
		// compute line-> m,n,x0,y0;
		int N=line->points.size();
		Eigen::Matrix2d A,B;
		A.setZero();
		B.setZero();
		B(1,1)=N;
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			A(0,0)+=(*it)->xyz(0)*(*it)->xyz(2);
			A(0,1)+=(*it)->xyz(0);
			A(1,0)+=(*it)->xyz(1)*(*it)->xyz(2);
			A(1,1)+=(*it)->xyz(1);
			B(0,0)+=(*it)->xyz(2)*(*it)->xyz(2);
			B(0,1)+=(*it)->xyz(2);
			B(1,0)+=(*it)->xyz(2);
		}
		Eigen::Matrix2d param=A*B.inverse();
		line->m=param(0,0);
		line->n=param(1,0);
		line->x0=param(0,1);
		line->y0=param(1,1);
		if(debug) fp<<"\tm="<<line->m<<"\tn="<<line->n<<"\tx0="<<line->x0<<"\ty0="<<line->y0<<std::endl;

		// refine the endpoints of the line;
		if(debug)
		{
			fp<<"\tend_point_1\t"<<line->end_point_1.transpose()<<std::endl;
			fp<<"\tend_point_2\t"<<line->end_point_2.transpose()<<std::endl;
		}
		double z=line->end_point_1(2);
		line->end_point_1(0)=line->m*z+line->x0;
		line->end_point_1(1)=line->n*z+line->y0;
		z=line->end_point_2(2);
		line->end_point_2(0)=line->m*z+line->x0;
		line->end_point_2(1)=line->n*z+line->y0;
		if(debug)
		{
			fp<<"\tend_point_1\t"<<line->end_point_1.transpose()<<std::endl;
			fp<<"\tend_point_2\t"<<line->end_point_2.transpose()<<std::endl;
		}

		// generate Plucker coordinates for the line;
		line->u=line->end_point_1.cross(line->end_point_2);
		line->v=line->end_point_2-line->end_point_1;
		if(debug) fp<<"\toriginal\t"<<line->u.transpose()<<"\t"<<line->v.transpose()<<std::endl;
		double d=line->u.norm()/line->v.norm();
		line->u.normalize();
		line->u=d*line->u;
		line->v.normalize();
		if(debug) fp<<"\tnormalized\t"<<line->u.transpose()<<"\t"<<line->v.transpose()<<std::endl;
	}*/

	void LineExtraction::computeLineInfo(Line *line)
	{
		line->info.setZero();
		Matrix6d tmp;
		for(int i=0;i<line->indices.size();i++)
		{
			Eigen::Vector3d pt=Transform::point2eigen(line->ptr_points->at(line->indices[i]));
			tmp.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
			tmp.block<3,3>(0,3)=-Transform::skew_sym(pt);
			tmp.block<3,3>(3,0)=Transform::skew_sym(pt);
			tmp.block<3,3>(3,3)=-Transform::skew_sym(pt)*Transform::skew_sym(pt);
			line->info=line->info+tmp;
		}

		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
		es.compute(line->info);
		Vector6d Lambda=es.eigenvalues();
		Matrix6d U=es.eigenvectors();
		Matrix6d sqrt_Lambda=Matrix6d::Zero();
		Matrix6d inv_Lambda=Matrix6d::Zero();
		for(int i=0;i<6;i++)
		{
			sqrt_Lambda(i,i)=sqrt(Lambda(i));
			if(Lambda(i)>0.01)
			{
				inv_Lambda(i,i)=1.0/Lambda(i);
			}
		}
		line->sqrt_info=U*sqrt_Lambda*U.transpose();
		line->cov=U*inv_Lambda*U.transpose();

	}
//	{
//		line->info.setZero();
//		Matrix6d tmp;
//		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
//		{
//			tmp.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
//			tmp.block<3,3>(0,3)=-Transform::skew_sym((*it)->xyz);
//			tmp.block<3,3>(3,0)=Transform::skew_sym((*it)->xyz);
//			tmp.block<3,3>(3,3)=-Transform::skew_sym((*it)->xyz)*Transform::skew_sym((*it)->xyz);
//			line->info=line->info+tmp;
//		}
////		line->info/=line->points.size();
//
//		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
//		es.compute(line->info);
//		Vector6d Lambda=es.eigenvalues();
//		Matrix6d U=es.eigenvectors();
//		Matrix6d sqrt_Lambda=Matrix6d::Zero();
//		Matrix6d inv_Lambda=Matrix6d::Zero();
//		for(int i=0;i<6;i++)
//		{
//			sqrt_Lambda(i,i)=sqrt(Lambda(i));
//			if(Lambda(i)>0.01)
//			{
//				inv_Lambda(i,i)=1.0/Lambda(i);
//			}
//		}
//		line->sqrt_info=U*sqrt_Lambda*U.transpose();
//		line->cov=U*inv_Lambda*U.transpose();
//
//	}

	void LineExtraction::extractLinesHough(Scan* scan, double scale)
	{

		if(debug) fp<<"extractLinesHough-----------------------------------"<<std::endl;
		int height=scan->imgDepth().rows;
		int width=scan->imgDepth().cols;
		cv::Mat img_occluding=cv::Mat::zeros(height,width,CV_8UC3);
		std::list<Line*> lines_occluding;
		for(int i=0;i<scan->sizeEdgePoint();i++)
		{
			int x=scan->edgePoint(i)->pixel(1);//height
			int y=scan->edgePoint(i)->pixel(0);//width
			img_occluding.at<cv::Vec3b>(x,y)[0]=255;//blue
			img_occluding.at<cv::Vec3b>(x,y)[1]=0;
			img_occluding.at<cv::Vec3b>(x,y)[2]=0;
		}
//		cv::imshow("contours",img_occluding);
//		cv::waitKey(0);

		cv::Mat contours=cv::Mat::zeros(height,width,CV_8UC1);
		for(int i=0;i<scan->sizeEdgePoint();i++)
		{
			int x=scan->edgePoint(i)->pixel(1);
			int y=scan->edgePoint(i)->pixel(0);
			contours.at<unsigned char>(x,y)=255;
		}
//		if(debug)
//		{
//			cv::imshow("image",contours);
//			cv::waitKey(0);
//		}

		// extract 2D lines in image using Hough transform (OpenCV);
		// void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(contours, lines, rho*scale, theta*scale, threshold, minLineLength, maxLineGap);
		std::vector<Line*> lines_tmp;
		lines_tmp.resize(lines.size());
//		for(std::vector<cv::Vec4i>::iterator it=lines.begin();it!=lines.end();it++)
		for(int i=0;i<lines.size();i++)
		{
			cv::Point pt1(lines[i][0],lines[i][1]);
			cv::Point pt2(lines[i][2],lines[i][3]);
			cv::line(img_occluding, pt1, pt2, CV_RGB(0,255,0));
			double x1=lines[i][0];
			double y1=lines[i][1];
			double x2=lines[i][2];
			double y2=lines[i][3];
			double tmp=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
			double a=(y1-y2)/tmp;
			double b=(x2-x1)/tmp;
			double d=(x1*y2-x2*y1)/tmp;
			Line *line=new Line;
			for(int j=0;j<scan->sizeEdgePoint();j++)
			{
				int x=scan->edgePoint(j)->pixel(0);
				int y=scan->edgePoint(j)->pixel(1);
				if(fabs(a*x+y*b+d)<5.0 && scan->edgePoint(j)->onLine==false)
				{
					scan->edgePoint(j)->onLine=true;
					line->points_tmp.push_back(scan->edgePoint(j));
					img_occluding.at<cv::Vec3b>(int(y),int(x))[0]=0;
					img_occluding.at<cv::Vec3b>(int(y),int(x))[1]=0;
					img_occluding.at<cv::Vec3b>(int(y),int(x))[2]=255;//red
				}
			}
			// lines_occluding.push_back(line);
			lines_tmp[i]=line;
		}
//		if(debug)
//		{
//			cv::imshow("image",img_occluding);
//			cv::waitKey(0);
//		}

		// 3D RANSAC (PCL);
		// merge similar lines;
		// same them in ordered list;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::VectorXf line_coeff;
		std::vector<int> line_inliers;
		Eigen::Matrix<double,1,1> tmp;
		bool merge=false;
		for(int i=0;i<lines_tmp.size();i++)
		{
//			if(debug) fp<<i<<"-th line in lines_tmp "<<lines_tmp[i]->points_tmp.size()<<std::endl;
			if(lines_tmp[i]->points_tmp.size()<=10)
			{
				delete lines_tmp[i];
				continue;
			}
			merge=false;
			cloud->points.resize(lines_tmp[i]->points_tmp.size());
			for(int j=0;j<lines_tmp[i]->points_tmp.size();j++)
			{
				cloud->points[j].x=lines_tmp[i]->points_tmp[j]->xyz(0);
				cloud->points[j].y=lines_tmp[i]->points_tmp[j]->xyz(1);
				cloud->points[j].z=lines_tmp[i]->points_tmp[j]->xyz(2);
//				fp<<lines_tmp[i]->points[j]->xyz.transpose()<<std::endl;
			}

//			std::cout<<"lines_tmp["<<i<<"]: "<<cloud->size()<<std::endl;
//			if (!vis->updatePointCloud (cloud,"line")) vis->addPointCloud (cloud,"line");
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "line");
//			vis->spin();

			line_inliers.clear();
			pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line
					(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
			ransac.setDistanceThreshold (.01);
//			model_line->setInputCloud(cloud);
//			ransac.setSampleConsensusModel(model_line);
			ransac.computeModel();
			ransac.getInliers(line_inliers);
			
//			std::cout<<"inliers["<<i<<"]: "<<line_inliers.size()<<std::endl;
//			cloud_in->clear();
//			pcl::copyPointCloud(*cloud,line_inliers,*cloud_in);
//			if (!vis->updatePointCloud (cloud_in,"line")) vis->addPointCloud (cloud_in,"line");
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "line");
//			vis->spin();
//
//			if(debug) fp<<"inliers "<<line_inliers.size()<<std::endl;
			ransac.getModelCoefficients(line_coeff);
//			if(debug) fp<<"coefficients "<<line_coeff.transpose()<<std::endl;
			lines_tmp[i]->end_point_1(0)=line_coeff(0);
			lines_tmp[i]->end_point_1(1)=line_coeff(1);
			lines_tmp[i]->end_point_1(2)=line_coeff(2);
			lines_tmp[i]->end_point_2(0)=line_coeff(0);
			lines_tmp[i]->end_point_2(1)=line_coeff(1);
			lines_tmp[i]->end_point_2(2)=line_coeff(2);
			lines_tmp[i]->dir(0)=line_coeff(3);
			lines_tmp[i]->dir(1)=line_coeff(4);
			lines_tmp[i]->dir(2)=line_coeff(5);
			lines_tmp[i]->dir.normalize();
//			lines_tmp[i]->u=lines_tmp[i]->end_point_1.cross(lines_tmp[i]->end_point_2);
//			lines_tmp[i]->v=lines_tmp[i]->end_point_2-lines_tmp[i]->end_point_1;
//			lines_tmp[i]->u/=lines_tmp[i]->v.norm();
//			lines_tmp[i]->v.normalize();
			if(debug)
			{
				fp<<"========================================="<<std::endl;
				fp<<i<<"-th line "<<line_coeff.transpose()<<std::endl;
			}
			for(std::list<Line*>::iterator it_line=lines_occluding.begin();it_line!=lines_occluding.end();it_line++)
			{
				double sim_dir=lines_tmp[i]->dir.transpose()*(*it_line)->dir;
				sim_dir=fabs(sim_dir);
				sim_dir=acos(sim_dir)*180/M_PI;

				Eigen::Vector3d vec_sim_dist=lines_tmp[i]->dir.cross((*it_line)->dir);
				vec_sim_dist.normalize();
				Eigen::Matrix<double,1,1> tmp=vec_sim_dist.transpose()*(lines_tmp[i]->end_point_1-(*it_line)->end_point_1);
				double sim_dist=fabs(tmp(0,0));

//				fp<<sim_dir<<"\t"<<sim_dist<<std::endl;

				if(sim_dir<thres_sim_dir*scale && sim_dist<thres_sim_dist*scale)
				{
//					if(debug)
//					{
//						fp<<"similar with a line in lines_occluding"<<std::endl;
//						fp<<"sim_dir ="<<sim_dir<<std::endl;
//						fp<<"sim_dist="<<sim_dist<<std::endl;
//					}
					// merge [i] and [j];
					for(int k=0;k<line_inliers.size();k++)
					{
						tmp=(*it_line)->dir.transpose()
							*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it_line)->end_point_2);
						if(tmp(0,0)>0)
						{
							(*it_line)->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
						}
						else for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin(); it!=(*it_line)->points.end();it++)
						{
							tmp=(*it_line)->dir.transpose()
								*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it)->xyz);
							if(tmp(0,0)<0)
							{
								(*it_line)->points.insert(it,lines_tmp[i]->points_tmp[line_inliers[k]]);
								break;
							}
						}
						std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();
						(*it_line)->end_point_1=(*it)->xyz;
//						(*it_line)->end_point_1_img=(*it)->pixel;
						it=(*it_line)->points.end(); it--;
						(*it_line)->end_point_2=(*it)->xyz;
//						(*it_line)->end_point_2_img=(*it)->pixel;
					}
					Eigen::Vector3d vec3d=(*it_line)->end_point_1-(*it_line)->end_point_2;
					(*it_line)->length=vec3d.norm();
					// update lines_occluding[j]->dir here;
					delete lines_tmp[i];
					merge=true;

//					std::cout<<"merge["<<i<<"]"<<std::endl;
//					cloud_in->clear();
//					for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin(); it!=(*it_line)->points.end();it++)
//					{
//						pcl::PointXYZ pt; pt.x=(*it)->xyz(0); pt.y=(*it)->xyz(1); pt.z=(*it)->xyz(2);
//						cloud_in->push_back(pt);
//					}
//					if (!vis->updatePointCloud (cloud_in,"line")) vis->addPointCloud (cloud_in,"line");
//					vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "line");
//					vis->spin();

					break;
				}
			}
			if(merge) continue;
			// else add a new line in lines_occluding
//			if(debug) fp<<"add a new line in lines_occluding"<<std::endl;
			Line *line=new Line;
//			line->points.resize(line_inliers.size());
//			line->end_point_1=lines_tmp[i]->end_point_1;
//			line->end_point_2=lines_tmp[i]->end_point_2;
			line->dir=lines_tmp[i]->dir;
//			line->u=lines_tmp[i]->u;
//			line->v=lines_tmp[i]->v;
//			fp<<"inlier size "<<line_inliers.size()<<std::endl; // more than 10;
			for(int k=0;k<line_inliers.size();k++)
			{
				if(line->points.size()==0)
				{
					line->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
					line->end_point_1=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
					line->end_point_2=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
//					fp<<"first pushed point "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<std::endl;
				}
				else
				{
					tmp=line->dir.transpose()*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-line->end_point_2);
					if(tmp(0,0)>0)
					{
						line->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
//						fp<<"at the end "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<"\t"<<tmp<<std::endl;
					}
					else
					for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
					{
						tmp=line->dir.transpose()*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it)->xyz);
						if(tmp(0,0)<0)
						{
							line->points.insert(it,lines_tmp[i]->points_tmp[line_inliers[k]]);
//							fp<<"in the middle "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<"\t"<<(*it)->xyz.transpose()<<std::endl;
							break;
						}
					}
					std::list<EdgePoint*>::iterator it=line->points.begin();
					line->end_point_1=(*it)->xyz;
//					line->end_point_1_img=(*it)->pixel;
					it=line->points.end(); it--;
					line->end_point_2=(*it)->xyz;
//					line->end_point_2_img=(*it)->pixel;
//					Eigen::Vector3d pt=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
//					Eigen::Vector3d ep1=line->end_point_1;
//					Eigen::Vector3d ep2=line->end_point_2;
//					Eigen::Vector3d dir=line->dir;
//					tmp=dir.transpose()*(pt-ep1);
//					if(tmp(0,0)<0) line->end_point_1=pt;
//					tmp=dir.transpose()*(pt-ep2);
//					if(tmp(0,0)>0) line->end_point_2=pt;
				}
			}
			Eigen::Vector3d vec3d=line->end_point_1-line->end_point_2;
			line->length=vec3d.norm();
			lines_occluding.push_back(line);
			delete lines_tmp[i];

//			std::cout<<"line["<<i<<"]"<<std::endl;
//			cloud_in->clear();
//			for(std::list<EdgePoint*>::iterator it=line->points.begin(); it!=line->points.end();it++)
//			{
//				pcl::PointXYZ pt; pt.x=(*it)->xyz(0); pt.y=(*it)->xyz(1); pt.z=(*it)->xyz(2);
//				cloud_in->push_back(pt);
//			}
//			if (!vis->updatePointCloud (cloud_in,"line")) vis->addPointCloud (cloud_in,"line");
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "line");
//			vis->spin();

//			if(debug)
//			{
//				std::list<Line*>::iterator itt_line=std::prev(lines_occluding.end());
////				int ii=lines_occluding.size()-1;
//				fp<<(*itt_line)->end_point_1.transpose()<<std::endl;
//				for(std::list<EdgePoint*>::iterator it=(*itt_line)->points.begin();it!=(*itt_line)->points.end();it++)
//				{
//					fp<<"\t"<<(*it)->xyz.transpose()<<"\t"<<(*itt_line)->dir.transpose()*((*it)->xyz-(*itt_line)->end_point_1)<<std::endl;
//				}
//				fp<<(*itt_line)->end_point_2.transpose()<<std::endl;
//			}
		}

//		for(int i=0;i<lines_occluding.size();i++)
//		if(debug) fp<<std::endl<<"splitting lines "<<std::endl;
		for(std::list<Line*>::iterator it_line=lines_occluding.begin();
									   it_line!=lines_occluding.end();it_line++)
		{
//			if(debug) fp<<"\t"<<lines_occluding.size()<<std::endl;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				if(it==(*it_line)->points.begin()) continue;
				std::list<EdgePoint*>::iterator it_pre=it;
				it_pre--;
				Eigen::Vector3d vec3d=(*it)->xyz-(*it_pre)->xyz;
				if(vec3d.norm()>thres_split*scale)
				{
//					(*it_line)->end_point_2=(*it_pre)->xyz;
					Line *line=new Line;
					line->points.splice(line->points.begin(),(*it_line)->points,it,(*it_line)->points.end());
//					std::list<EdgePoint*>::iterator itt=line->points.begin();
//					line->end_point_1=(*itt)->xyz;
//					line->end_point_2=(*std::prev(line->points.end()))->xyz;
//					vec3d=line->end_point_1-line->end_point_2;
//					line->length=vec3d.norm();
//					line->dir=(*it_line)->dir;
					lines_occluding.push_back(line);
					break;
				}
			}
		}

//		if(debug) fp<<std::endl<<"cunning the lines "<<std::endl;
		for(std::list<Line*>::iterator it_line=lines_occluding.begin();it_line!=lines_occluding.end();)
		{
//			if(debug) fp<<"\t"<<(*it_line)->points.size()<<std::endl;
			// cunning the lines that does not contain enough points;
			if((*it_line)->points.size()<double(min_points_on_line)/scale)
			{
				delete *it_line;
				it_line=lines_occluding.erase(it_line);
			}
			else
			{
				std::list<EdgePoint*>::iterator itt=(*it_line)->points.begin();
				(*it_line)->end_point_1=(*itt)->xyz;
				(*it_line)->end_point_2=(*std::prev((*it_line)->points.end()))->xyz;
				Eigen::Vector3d vec3d=(*it_line)->end_point_1-(*it_line)->end_point_2;
				(*it_line)->length=vec3d.norm();
				(*it_line)->dir=(*it_line)->end_point_2-(*it_line)->end_point_1;
				(*it_line)->dir.normalize();
				Feature *feat=new Feature(*it_line);
				scan->addFeature(feat);

//					std::cout<<"feature: "<<feat->ID()<<std::endl;
//					cloud_in->clear();
//					for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin(); it!=(*it_line)->points.end();it++)
//					{
//						pcl::PointXYZ pt; pt.x=(*it)->xyz(0); pt.y=(*it)->xyz(1); pt.z=(*it)->xyz(2);
//						cloud_in->push_back(pt);
//					}
//					if (!vis->updatePointCloud (cloud_in,"line")) vis->addPointCloud (cloud_in,"line");
//					vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "line");
//					vis->spin();

				it_line++;

			}
		//	compute the dir
		}
//		fp.close();


//		cv::Mat img_occluded=cv::Mat::zeros(480,640,CV_8UC3);
//		for (int i=0;i<labels_edge->height;i++)
//		{
//			for (int j=0;j<labels_edge->width;j++)
//			{
//				if(labels_edge->at(j,i).label==4)//occluded
//				{
//					img_occluded.at<cv::Vec3b>(i,j)[0]=255;
//					img_occluded.at<cv::Vec3b>(i,j)[1]=0;
//					img_occluded.at<cv::Vec3b>(i,j)[2]=0;//red
//				}
//			}
//		}
//		contours=cv::Mat::zeros(480,640,CV_8UC1);
//		for (int i=0;i<labels_edge->height;i++)
//		{
//			for (int j=0;j<labels_edge->width;j++)
//			{
//				if(labels_edge->at(j,i).label==4)
//				{
//					contours.at<unsigned char>(i,j)=255;
//				}
//			}
//		}
//		//void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
//		lines.clear();
//		cv::HoughLinesP(contours, lines, rho, theta, threshold, minLineLength, maxLineGap);

	}



//	void LineExtraction::fitLines(Scan *scan)
//	{
//		fp.open("extract_EdgePoints.txt",std::ios::app);
//		int bins_theta=10,bins_phy=20;
////		std::vector<Eigen::Vector2d> dirs_sphere;
////		dirs_sphere.resize(scan->edge_points.size());
//		Eigen::Vector2d dir;
//		std::vector<std::vector<EdgePoint*> > cells;
//		cells.resize(bins_theta*bins_phy);
//		std::vector<std::vector<EdgePoint*> > lines;
//		for(int i=0;i<scan->edge_points.size();i++)
//		{
//			if(scan->edge_points[i]->isEdge==false)
//				continue;
//			dir(0)=acos(scan->edge_points[i]->dir(2));
//			dir(1)=atan2(scan->edge_points[i]->dir(1),scan->edge_points[i]->dir(0));
//			int row=dir(0)/(M_PI/bins_theta);
//			int col=(dir(1)+M_PI)/(M_PI*2.0/bins_phy);
//			int index=bins_phy*row+col;
//			cells[index].push_back(scan->edge_points[i]);
//		}
////		for(int i=0;i<cells.size();i++)
////		{
////			int row=i/bins_phy;
////			int col=i%bins_phy;
////			if(col==0) fp<<std::endl;
////			fp<<cells[i].size()<<"\t";
////		}
//		std::vector<Sorted_Cell> sorted_cells;
//		sorted_cells.resize(cells.size());
//		for(int i=0;i<cells.size();i++)
//		{
//			sorted_cells[i].index=i;
//			sorted_cells[i].num_point=cells[i].size();
//		}
//		std::sort(sorted_cells.begin(),sorted_cells.end());
//		std::vector<Sorted_Cell>::iterator iter_sorted_cells=sorted_cells.end()-1;
//		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
//		int count=0;
//		timeval time_seed;
//
//		while(true)
//		{
////			fp<<"cell "<<count<<" - "<<iter_sorted_cells->num_point<<std::endl;
//			int idx=iter_sorted_cells->index;
////			Eigen::Vector3d n0;
////			n0.setZero();
////			for(int i=0;i<cells[idx].size();i++)
////			{
//////				cells[idx][i]->rgb(0)=red[count];
//////				cells[idx][i]->rgb(1)=grn[count];
//////				cells[idx][i]->rgb(2)=blu[count];
////				n0+=cells[idx][i]->xyz;
//////				fp<<"\t"<<cells[idx][i]->xyz.transpose()<<"\t"<<cells[idx][i]->rgb.transpose()<<std::endl;
////			}
////			n0/=iter_sorted_cells->num_point;
//			int iter=0;
//			while(true)
//			{
////				srand((int)time(0));
//				gettimeofday(&time_seed,NULL);
//				srand((int)time_seed.tv_usec);
//				int index=(int)(rand()%iter_sorted_cells->num_point);
//				Eigen::Vector3d p0=cells[idx][index]->xyz;
//				Eigen::Vector3d n0=cells[idx][index]->dir;
//				std::vector<EdgePoint*> tmp_line;
//				for(int i=0;i<scan->edge_points.size();i++)
//				{
//					if(scan->edge_points[i]->isEdge==false)
//						continue;
//					if(scan->edge_points[i]->rgb.norm()>0)
//						continue;
//					Eigen::Vector3d p=scan->edge_points[i]->xyz;
//					Eigen::Matrix<double,1,1> mu_tmp=n0.transpose()*(p-p0);
//					double mu=mu_tmp(0,0)/(n0.norm()*n0.norm());
//					p=p-mu*n0-p0;
//					mu=p.norm();
//					double ang=n0.transpose()*scan->edge_points[i]->dir;
//					if(mu<0.03 && ang>0.9)
//						tmp_line.push_back(scan->edge_points[i]);
//				}
//				int thres=(int(iter_sorted_cells->num_point/500.0)+1)*100;
////				fp<<"\t"<<time_seed.tv_usec<<"\t"<<index<<"\t"<<thres<<"\t"<<tmp_line.size()<<std::endl;
//				if(tmp_line.size()>=thres)
//				{
//					for(int i=0;i<tmp_line.size();i++)
//					{
//						tmp_line[i]->rgb(0)=red[count];
//						tmp_line[i]->rgb(1)=grn[count];
//						tmp_line[i]->rgb(2)=blu[count];
////						fp<<"\t\t"<<tmp_line[i]->xyz.transpose()<<std::endl;
//					}
//					count++;
//					lines.push_back(tmp_line);
//					break;
//				}
//				if(iter>20)
//					break;
//				iter++;
//			}
//			if(iter_sorted_cells==sorted_cells.begin()) break;
//			iter_sorted_cells--;
//			if(iter_sorted_cells->num_point<50) break;
//		}
//		fp.close();
//	}
//
//	bool LineExtraction::fitSphere(EdgePoint *edge_point)
//	{
//		fp.open("extract_EdgePoints.txt",std::ios::app);
//		Eigen::Vector3d center;
//		double radius;
//
//		double x_bar=0,y_bar=0,z_bar=0;
//		double xy_bar=0,xz_bar=0,yz_bar=0;
//		double x2_bar=0,y2_bar=0,z2_bar=0;
//		double x2y_bar=0,x2z_bar=0,xy2_bar=0,y2z_bar=0,xz2_bar=0,yz2_bar=0;
//		double x3_bar=0,y3_bar=0,z3_bar=0;
//		for(int i=0;i<edge_point->neighbors.size();i++)
//		{
//			double x=edge_point->neighbors[i]->xyz(0);
//			double y=edge_point->neighbors[i]->xyz(1);
//			double z=edge_point->neighbors[i]->xyz(2);
//			x_bar  +=x;
//			y_bar  +=y;
//			z_bar  +=z;
//			xy_bar +=x*y;
//			xz_bar +=x*z;
//			yz_bar +=y*z;
//			x2_bar +=x*x;
//			y2_bar +=y*y;
//			z2_bar +=z*z;
//			x2y_bar+=x*x*y;
//			x2z_bar+=x*x*z;
//			xy2_bar+=x*y*y;
//			y2z_bar+=y*y*z;
//			xz2_bar+=x*z*z;
//			yz2_bar+=y*y*z;
//			x3_bar +=x*x*x;
//			y3_bar +=y*y*y;
//			z3_bar +=z*z*z;
//		}
//		x_bar  /=edge_point->neighbors.size();
//		y_bar  /=edge_point->neighbors.size();
//		z_bar  /=edge_point->neighbors.size();
//		xy_bar /=edge_point->neighbors.size();
//		xz_bar /=edge_point->neighbors.size();
//		yz_bar /=edge_point->neighbors.size();
//		x2_bar /=edge_point->neighbors.size();
//		y2_bar /=edge_point->neighbors.size();
//		z2_bar /=edge_point->neighbors.size();
//		x2y_bar/=edge_point->neighbors.size();
//		x2z_bar/=edge_point->neighbors.size();
//		xy2_bar/=edge_point->neighbors.size();
//		y2z_bar/=edge_point->neighbors.size();
//		xz2_bar/=edge_point->neighbors.size();
//		yz2_bar/=edge_point->neighbors.size();
//		x3_bar /=edge_point->neighbors.size();
//		y3_bar /=edge_point->neighbors.size();
//		z3_bar /=edge_point->neighbors.size();
//		
//		Eigen::Matrix3d A;
//		A(0,0)=x2_bar-x_bar*x_bar;
//		A(0,1)=xy_bar-x_bar*y_bar;
//		A(0,2)=xz_bar-x_bar*z_bar;
//		A(1,0)=xy_bar-x_bar*y_bar;
//		A(1,1)=y2_bar-y_bar*y_bar;
//		A(1,2)=yz_bar-y_bar*z_bar;
//		A(2,0)=xz_bar-x_bar*z_bar;
//		A(2,1)=yz_bar-y_bar*z_bar;
//		A(2,2)=z2_bar-z_bar*z_bar;
//
//		Eigen::Vector3d b;
//		b(0)=(x3_bar -x_bar*x2_bar)+(xy2_bar-x_bar*y2_bar)+(xz2_bar-x_bar*z2_bar);
//		b(1)=(x2y_bar-y_bar*x2_bar)+(y3_bar -y_bar*y2_bar)+(yz2_bar-y_bar*z2_bar);
//		b(2)=(x2z_bar-z_bar*x2_bar)+(y2z_bar-z_bar*y2_bar)+(z3_bar -z_bar*z2_bar);
//		b*=0.5;
//
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A);
//		Eigen::Vector3d Lambda=es.eigenvalues();
//		Eigen::Matrix3d U=es.eigenvectors();
//		
////		fp<<std::endl<<edge_point->neighbors.size()<<"\t"<<Lambda.transpose()<<std::endl;
////		fp<<A<<std::endl;
//
//		Eigen::Matrix3d A_inv;
//		bool invertible;
//		A.computeInverseWithCheck(A_inv,invertible);
//		if(invertible)
//		{
//			center=A_inv*b;
//			double x0=center(0);
//			double y0=center(1);
//			double z0=center(2);
//			radius=sqrt(x2_bar-2*x0*x_bar+x0*x0
//					   +y2_bar-2*y0*y_bar+y0*y0
//					   +z2_bar-2*z0*z_bar+z0*z0);
//			edge_point->meas_Edge=1.0/radius;
//			fp.close();
//			return true;
//		}
//		else
//		{
//			edge_point->meas_Edge=-1;
//			fp.close();
//			return false;
//		}
//
////		edge_point->meas_Edge
//	}

	void LineExtraction::visLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		scan->vis(vis);
		int i=0;
		std::ofstream fp; fp.open("vislines.txt");
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{
			if(it->second->Type()==LINE)
			{
//				if(it->second->line()->length==0)
				{
					it->second->vis(vis,red[i%14],grn[i%14],blu[i%14]);//255,0,0);//

					fp<<"********************************************************"<<endl;
					fp<<it->second<<endl;
					int N=it->second->line()->indices.size();
					Eigen::Vector3d vt1,vt2;
					for(int j=0;j<N;j++)
					{
						pcl::PointXYZRGBA pt=it->second->line()->ptr_points->at(it->second->line()->indices[j]);
						Eigen::Vector3d v=Transform::point2eigen(pt);
						fp<<v.transpose()<<"\t";
						vt1=v;
						if(j>0) 
						{
							Eigen::Vector3d dv=vt1-vt2;
							fp<<dv.norm()<<endl;
						}
						vt2=vt1;
					}

					pcl::PointXYZRGBA p1=it->second->line()->ptr_points->at(it->second->line()->indices[0]);
					pcl::PointXYZRGBA p2=it->second->line()->ptr_points->at(it->second->line()->indices[it->second->line()->indices.size()-1]);
					Eigen::Vector3d v1=Transform::point2eigen(p1);
					Eigen::Vector3d v2=Transform::point2eigen(p2);
					v1=v1-v2;
					v2=it->second->line()->end_point_1-it->second->line()->end_point_2;
					cout<<v2.norm()<<" - "<<v1.norm()<<" - "<<it->second->line()->length<<" - "<<it->second->line()->indices.size()<<endl;

//					cout<<it->second->line()->indices.size()<<", "<<it->second->line()->length<<endl;
//					vis->spin();
				}

//				Eigen::Vector3d centroid; centroid.setZero();
//				for(int i=0;i<it->second->line()->indices.size();i++)
//				{
//					pcl::PointXYZRGBA pt=it->second->line()->ptr_points->at(it->second->line()->indices[i]);
//					centroid+=Transform::point2eigen(pt);
//				}
//				centroid/=it->second->line()->indices.size();
			}
			i++;
		}
		fp.close();
		vis->spin();

//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(tmp,255,0,0);
//		if (!vis->updatePointCloud (tmp, color, "tmp")) vis->addPointCloud (tmp, color, "tmp");
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "tmp");
//		vis->spin();
	}


}
