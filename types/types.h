/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-12-12 10:04
#
# Filename:		types.h
#
# Description: 
#
===============================================*/

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <list>
#include <limits>
#include <map>
#include <sys/time.h>
#include <random>
#include <chrono>
#include <dirent.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl-1.8/pcl/sample_consensus/sac_model_line.h>
#include <pcl-1.8/pcl/sample_consensus/ransac.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/pcl_painter2D.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/common/centroid.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/features/normal_3d.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/eigen.hpp>
//#include "types/map.h"
//#include "geometric_feature_matching.h"
//#include <g2o/types/sba/types_six_dof_expmap.h>

namespace ulysses
{
	class Scan; 
	class Feature;
	class Landmark;
	struct Plane;
	struct Line;
	struct PlaneLM;
	struct LineLM;

	typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptrPointCloud;
	typedef pcl::PointCloud<pcl::Normal>::Ptr ptrNormalCloud;
	typedef pcl::PointCloud<pcl::PointXY>::Ptr ptrPixelCloud;

	typedef std::map<double,std::vector<std::string> >::iterator iterObserv;
	typedef std::map<double,std::vector<std::string> >::const_iterator const_iterObserv;

	typedef Eigen::Matrix<double,6,1> Vector6d;
	typedef Eigen::Matrix<double,6,6> Matrix6d;
	enum FEATURE_TYPE {PLANE, LINE};

	// CameraIntrinsic 
	struct CameraIntrinsic
	{
		CameraIntrinsic() {}
		CameraIntrinsic(double fx_, double fy_, double cx_, double cy_) {fx=fx_;fy=fy_;cx=cx_;cy=cy_;}

		double fx,fy,cx,cy;
		double m_fp;
		double sigma_disparity, sigma_u, sigma_v;
		double factor;
		int width, height;

		// getMatrix
		// - K =
		// - |fx 0  cx|
		//   |0  fy cy|
		//   |0  0  1 |
		Eigen::Matrix3d getMatrix()
		{
			Eigen::Matrix3d mat;
			mat.setIdentity();
			mat(0,0)=fx;
			mat(1,1)=fy;
			mat(0,2)=cx;
			mat(1,2)=cy;
			return mat;
		}

		// project
		// - project the point in 3d camera frame into the pixel frame;
		// - u=Kp;
		// - u is the homogeneous coordinates;
		// - u=[col,row,1]^T;
		Eigen::Vector3d project(Eigen::Vector3d p)
		{
			Eigen::Vector3d tmp,u;
			tmp=getMatrix()*p/p(2);
			u(0)=tmp(0);
			u(1)=tmp(1);
			u(2)=1;
			//u(0) = p[0]*fx/p[2] + cx;
			//u(1) = p[1]*fy/p[2] + cy;
			return u;
		}
	};

	struct Point
	{
		// xyz in meters;
		Eigen::Vector3d xyz, normal;

		// coordinate in the PPS for the local plane parameters (after Rotation_PCA);
		Eigen::Vector3d pps;
		void Cartesian2PPS(Eigen::Matrix3d Rotation_PCA=Eigen::Matrix3d::Identity())
		{
			Eigen::Vector3d tmp_vec3d;
			tmp_vec3d=Rotation_PCA*normal;
			if(tmp_vec3d(2)>1.0-1e-20)
			{
				tmp_vec3d(2)=1.0;
			}
			if(tmp_vec3d(2)<-1.0+1e-20)
			{
				tmp_vec3d(2)=-1.0;
			}
			pps(0)=acos(tmp_vec3d(2));
//			if(pps(0)>M_PI)
//			{
//				pps(0)=M_PI*2-pps(0);
//			}
			pps(1)=atan2(tmp_vec3d(1),tmp_vec3d(0));
			pps(2)=-normal.dot(xyz);
		}

		// rgb \in [0,1]^3;
		Eigen::Vector3d rgb;

		// u,v: pixel coordinate;
		int u,v; // u<480, v<640

		// cov of the point;
		// measurement uncertainty;
		Eigen::Matrix3d cov, cov_inv; 
		void compute_cov(CameraIntrinsic& cam)
		{
			double sigma_d=cam.m_fp*cam.sigma_disparity*xyz(2)*xyz(2); //m
			double d_fu=xyz(2)/cam.fx;
			double d_fv=xyz(2)/cam.fy;
			double x_d=xyz(0)/xyz(2);
			double y_d=xyz(1)/xyz(2);
			cov(0,0)=d_fu*d_fu*cam.sigma_u*cam.sigma_u+x_d*x_d*sigma_d*sigma_d;
			cov(0,1)=x_d*y_d*sigma_d*sigma_d;
			cov(0,2)=x_d*sigma_d*sigma_d;
			cov(1,0)=cov(0,1);
			cov(1,1)=d_fv*d_fv*cam.sigma_v*cam.sigma_v+y_d*y_d*sigma_d*sigma_d;
			cov(1,2)=y_d*sigma_d*sigma_d;
			cov(2,0)=cov(0,2);
			cov(2,1)=cov(1,2);
			cov(2,2)=sigma_d*sigma_d;
			cov_inv=cov.inverse();
		}

		// weight in the plane fitting;
		// weight = pln_n^T * cov_inv * pln_n;
		// weight_angle = [p_pi/(pln_n^T*p_pi)]^T * cov_inv * [p_pi/(pln_n^T*p_pi)];
		double weight;

	};

	struct Transform
	{
		Transform()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform(const Transform& T)
		{
			R=T.R;
			t=T.t;
//			time_stamp=T.time_stamp;
		}

		Transform(Eigen::Matrix3d R_, Eigen::Vector3d t_)
		{
			R=R_;
			t=t_;
		}

		Transform(Eigen::Quaterniond Q_, Eigen::Vector3d t_)
		{
			R=Q_.toRotationMatrix();
			t=t_;
		}

		Transform(double tx, double ty, double tz, double qx, double qy, double qz, double qw)
		{
			Eigen::Quaterniond quat(qw,qx,qy,qz);
			Eigen::Vector3d trans(tx,ty,tz);
			R=quat.toRotationMatrix();
			t=trans;
		}

		Transform(cv::Mat m)
		{
			Eigen::Matrix4d T;
			cv::cv2eigen(m,T);
			R=T.block<3,3>(0,0);
			t=T.block<3,1>(0,3);
		}

		Eigen::Vector3d t;
		Eigen::Matrix3d R;
//		double time_stamp;

		friend std::ostream & operator << (std::ostream &os, const ulysses::Transform &T)
		{
			os<<T.t.transpose()<<" "<<T.Quaternion().transpose();
			return os;
		}

		// eular angle: Z-Y-X
		Eigen::Vector3d Eulars() {return R.eulerAngles(2,1,0);}

		Eigen::Quaterniond Quat() const {return Eigen::Quaterniond(R);}
		Eigen::Vector4d Quaternion() const
		{
			Eigen::Quaterniond q=Quat();
			Eigen::Vector4d vec;
			vec.block<3,1>(0,0)=q.vec();
			vec(3)=q.w();
			return vec;
		}

		static const Eigen::Matrix3d skew_sym(Eigen::Vector3d u)
		{
			Eigen::Matrix3d U;
			U.setZero();
			U(0,1)=-u(2);
			U(0,2)=u(1);
			U(1,0)=u(2);
			U(1,2)=-u(0);
			U(2,0)=-u(1);
			U(2,1)=u(0);
			return U;
		}

		static const Eigen::Vector3d skew_sym_inv(Eigen::Matrix3d U)
		{
			Eigen::Vector3d u;
			u(1)=U(0,2);
			u(2)=U(1,0);
			u(0)=U(2,1);
			return u;
		}

		void inverse()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			R=R_tmp;
			t=t_tmp;
		}

		Transform inv() const
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			Transform T(R_tmp,t_tmp);
			return T;
		}

		void setIdentity()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		static Transform Identity()
		{
			Transform T;
			T.R.setIdentity();
			T.t.setZero();
			return T;
		}

		Transform operator* (const Transform& tr2) const
		{
			Transform result;
			result.t=t+R*tr2.t;
			result.R=R*tr2.R;
			return result;
		}

		void leftMultiply(Eigen::Isometry3d T)
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=T.rotation()*R;
			t_tmp=T.rotation()*t+T.translation();
			R=R_tmp;
			t=t_tmp;
		}

		Eigen::Vector3d transformPoint(Eigen::Vector3d p)
		{
			return R*p+t;
		}

		Eigen::Vector4d transformPlane(Eigen::Vector4d pln)
		{
			Eigen::Vector4d pln_trans;
			pln_trans.block<3,1>(0,0)=R*pln.block<3,1>(0,0);
			pln_trans(3)=pln(3)-pln_trans.block<3,1>(0,0).transpose()*t;
			return pln_trans;
		}

		Eigen::Matrix4f getMatrix4f()
		{
			Eigen::Matrix4f transform;
			transform.setIdentity();
			for(int i=0;i<3;i++)
			{
				transform(i,3)=(float)t(i);
				for(int j=0;j<3;j++)
				{
					transform(i,j)=(float)R(i,j);
				}
			}
			return transform;
		}

		Eigen::Matrix<double,6,1> getMotionVector()
		{
			Eigen::Matrix<double,6,1> xi;
			xi.block<3,1>(0,0)=t;
			double theta=1/cos((R.trace()-1.0)/2.0);
			Eigen::Vector3d w;
			if(theta<1e-7)
			{
				w=Eigen::Vector3d::Zero();
			}
			else
			{
				w(0)=R(2,1)-R(1,2);
				w(1)=R(0,2)-R(2,0);
				w(2)=R(1,0)-R(0,1);
				w=w/(2*sin(theta));
			}
			xi.block<3,1>(3,0)=w;
			return xi;
		}

		const Eigen::Matrix4d getMatrix() const
		{
			Eigen::Matrix4d T;
			T.setIdentity();
			T.block<3,3>(0,0)=R;
			T.block<3,1>(0,3)=t;
			return T;
		}

		const Eigen::Matrix4d getPlaneTransform() const 
		{
			Eigen::Matrix4d T_pi=inv().getMatrix();
			return T_pi.transpose();
		}

		const Matrix6d getLineTransform() const
		{
			Matrix6d T_L;
			T_L.setZero();
			T_L.block<3,3>(0,0)=R;
			T_L.block<3,3>(3,3)=R;
			T_L.block<3,3>(0,3)=skew_sym(t)*R;
			return T_L;
		}

		static const Eigen::Matrix4d PluckerCoords2Mat(Vector6d l)
		{
			Eigen::Vector3d u=l.block<3,1>(0,0);
			Eigen::Vector3d v=l.block<3,1>(3,0);
			Eigen::Matrix4d L=Eigen::Matrix4d::Zero();
			L.block<3,3>(0,0)=skew_sym(u);
			L.block<3,1>(0,3)=v;
			L.block<1,3>(3,0)=-v.transpose();
			return L;
		}

		static const Eigen::Matrix4d PluckerCoords2DualMat(Vector6d l)
		{
			Eigen::Vector3d u=l.block<3,1>(0,0);
			Eigen::Vector3d v=l.block<3,1>(3,0);
			Eigen::Matrix4d L=Eigen::Matrix4d::Zero();
			L.block<3,3>(0,0)=skew_sym(v);
			L.block<3,1>(0,3)=u;
			L.block<1,3>(3,0)=-u.transpose();
			return L;
		}

		static const Vector6d PluckerMat2Coords(Eigen::Matrix4d L) 
		{
			Vector6d l;
			l.block<3,1>(0,0)=skew_sym_inv(L.block<3,3>(0,0));
			l.block<3,1>(3,0)=L.block<3,1>(0,3);
			return l;
		}

		static const Vector6d PluckerDualMat2Coords(Eigen::Matrix4d L)
		{
			Vector6d l;
			l.block<3,1>(3,0)=skew_sym_inv(L.block<3,3>(0,0));
			l.block<3,1>(0,0)=L.block<3,1>(0,3);
			return l;
		}

		static Eigen::Vector3d point2eigen(pcl::PointXYZRGBA pt)
		{
			Eigen::Vector3d vec;
			vec(0)=pt.x; vec(1)=pt.y; vec(2)=pt.z;
			return vec;
		}

		void vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, double time);

	};

	struct EdgePoint
	{
		EdgePoint() : onLine(false) {}
		
		~EdgePoint() {}

		Eigen::Vector3d xyz;
		Eigen::Vector2d pixel;
		bool isEdge;
		
		Eigen::Matrix3d cov;
		std::vector<EdgePoint*> neighbors;
		double meas_Edge;
		Eigen::Vector3d dir;
		Eigen::Vector3i rgb;

		bool onLine;

		// scan->point_cloud->at(index);
		int idx;
	};

	struct Plane
	{
		Plane() : id("plane_") {} // for slam;
		Plane(const PlaneLM *lm, const Transform &Tcw, double sigma); // for simulateion;
		Plane(const PlaneLM *lm); // for relocalizing;
		Plane(const PlaneLM *lm, const Transform &Tcw); // for labeling;
		Plane(Eigen::Vector3d nn, double dd) : id("plane_"), n(nn), d(dd) {} // for loading data;

		std::string id;
		Eigen::Vector3d n;
		double d;

		Eigen::Vector3d direction() const {return n;}
		Eigen::Vector3d distance() const {return Eigen::Vector3d(d,0.0,0.0);}

		// indices 
		// scan->point_cloud->at(indices[i]);
		std::vector<int> indices;
		ptrPointCloud ptr_points;

		// centroid and scatter matrix 
		// for the points on plane and their colors;
		Eigen::Vector3d centroid_point, centroid_color;
		Eigen::Matrix3d scatter_point, scatter_color;

		// covariance and information matrix 
		// for the plane parameters [n,d];
		Eigen::Matrix4d cov, info, sqrt_info;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		friend std::ostream & operator << (std::ostream &os, const Plane *p)
		{
			os<<p->n.transpose()<<" "<<p->d;
			return os;
		}

		double dist2point(Eigen::Vector3d p)
		{
			return fabs(n.dot(p)+d);
		}

		double dist2plane(Plane *p);

	};

	struct Line
	{
		Line() : id("xline_") {} // for slam;
		Line(const LineLM *lm, const Transform &Tcw, double sigma); // for simulation;
		Line(const LineLM *lm); // for relocalizing;
		Line(const LineLM *lm, const Transform &Tcw); // for labeling;
		Line(Eigen::Vector3d uu, Eigen::Vector3d vv) : id("xline_"), u(uu), v(vv) {} // for loading data;

		~Line() {}

		std::string id;
		// Plucker coordinates;
		// u - normal to the plane joining the line and the origin;
		// v - line direction;
		Eigen::Vector3d u,v;
		Matrix6d cov, info, sqrt_info;

		Eigen::Vector3d direction() const {return v;}
		Eigen::Vector3d distance() const {return u;}

		std::vector<int> indices;
		ptrPointCloud ptr_points;

		// tmp variables during extraction;
		std::vector<EdgePoint*> points_tmp; // for index;
		std::list<EdgePoint*> points;
		Eigen::Vector3d end_point_1, end_point_2;
		Eigen::Vector3d dir;
		double length;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		friend std::ostream & operator << (std::ostream &os, const Line *l)
		{
			os<<l->u.transpose()<<" "<<l->v.transpose();
			return os;
		}

		double dist2point(Eigen::Vector3d p)
		{
			Eigen::Vector3d d=u+Transform::skew_sym(v)*p;
			return d.norm();
		}

		double dist2line(Line *l);

		int idx_lbd;
	};

	struct PlaneLM
	{
		PlaneLM(Plane *plane, Transform Tcg) // for slam;
		{
			id="map_plane_";
			Transform Tgc(Tcg.inv());
			Eigen::Vector4d pln;
			pln.block<3,1>(0,0)=plane->n;
			pln(3)=plane->d;
			pi=Tgc.getPlaneTransform()*pln;
			pi.normalize();
		}
		PlaneLM(Eigen::Vector4d p) // for simulation
		{	
			id="map_plane_";
			pi=p.normalized();
		}
		PlaneLM() {}

		~PlaneLM() {}

		Eigen::Vector4d pi;
		std::string id;
		
		std::vector<int> indices_cameras;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		friend std::ostream & operator << (std::ostream &os, const PlaneLM *p)
		{
			os<<p->pi.transpose();
			return os;
		}
	};

	struct LineLM
	{
		LineLM(Line *line, Transform Tcg) // for slam;
		{
			id="map_xline_";
			Transform Tgc(Tcg.inv());
			Vector6d ln;
			ln.block<3,1>(0,0)=line->u;
			ln.block<3,1>(3,0)=line->v;
			L=Tgc.getLineTransform()*ln;
		}
		LineLM(Vector6d l) // for simulateion;
		{
			id="map_xline_";
			l/=l.block<3,1>(3,0).norm();
			L=l;
		}
		LineLM() {}

		~LineLM() {}

		Vector6d L; 
		std::string id;

		std::vector<int> indices_cameras;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		friend std::ostream & operator << (std::ostream &os, const LineLM *l)
		{
			os<<l->L.transpose();
			return os;
		}
	};


//	template<class F, class L> 
	class Feature 
	{
	public:
		// for slam;
		Feature(Plane *p) : plane_(p), type(PLANE), id(p->id), ptr(0), 
					ptr_indices(&p->indices), ptr_ptr_points(&p->ptr_points) {}
		Feature(Line *l) : line_(l), type(LINE), id(l->id), ptr(0),
					ptr_indices(&l->indices), ptr_ptr_points(&l->ptr_points) {}

		// for simulation;
		Feature(Landmark *lm, const Transform &Tcw, double sigma); // ptr(lm)

		// for relocalizing;
		Feature(Landmark *lm); // ptr(lm)
		// for labeling;
		Feature(Landmark *lm, const Transform &Tcw); // ptr(lm)

		// for loading data;
		Feature(std::string s, Eigen::Vector3d n, double d) : id(s), type(PLANE), plane_(new Plane(n,d)), 
									  ptr(0), ptr_indices(&plane_->indices), ptr_ptr_points(&plane_->ptr_points) {}
		Feature(std::string s, Eigen::Vector3d u, Eigen::Vector3d v) : id(s), type(LINE), line_(new Line(u,v)), 
									   ptr(0), ptr_indices(&line_->indices), ptr_ptr_points(&line_->ptr_points) {}

		~Feature() 
		{
			if(type==PLANE) delete plane_;
			else if(type==LINE) delete line_;
		}

		void release()
		{
			if(type==PLANE) plane_->ptr_points.reset();
			else if(type==LINE) line_->ptr_points.reset();
		}

		friend std::ostream & operator << (std::ostream &os, const Feature *f)
		{
			if(f->type==PLANE) os<<f->id<<" "<<f->plane_;
			else if(f->type==LINE) os<<f->id<<" "<<f->line_;
//			os<<f->ptr_indices->size()<<std::endl;
//			for(int i=0;i<f->ptr_indices->size();i++) { os<<(*f->ptr_indices)[i]<<" "; }
//			os<<std::endl;
			return os;
		}

		// f->indices are available;
		double dist(Feature *f)
		{
			if(type!=f->Type()) return -1;
			if(type==PLANE) return plane_->dist2plane(f->plane());
			else if(type==LINE) return line_->dist2line(f->line());
		}

		void print(std::ostream &os)
		{
			if(type==PLANE) os<<id<<" "<<plane_<<std::endl;
			else if(type==LINE) os<<id<<" "<<line_<<std::endl;
			os<<ptr_indices->size()<<std::endl;
			for(int i=0;i<ptr_indices->size();i++) os<<(*ptr_indices)[i]<<std::endl;
		}

		std::string ID() const {return id;}
		FEATURE_TYPE Type() const  {return type;}

		Plane*& plane() {return plane_;}
		Line*& line() {return line_;}
		Landmark*& ptrLandmark() {return ptr;}

		void setID(int i) 
		{
			if(std::to_string(i).size()==1) id=id+"00"+std::to_string(i);
			else if(std::to_string(i).size()==2) id=id+"0"+std::to_string(i);
			else if(std::to_string(i).size()==3) id=id+std::to_string(i);
		}

		Eigen::Vector3d getDirection() const 
		{
			if(type==PLANE) return plane_->direction();
			else if(type==LINE) return line_->direction();
		}

		Eigen::Vector3d getDistance() const 
		{
			if(type==PLANE) return plane_->distance();
			else if(type==LINE) return line_->distance();
		}

		double delta_dir(const Feature *f) const 
		{
			Eigen::Vector3d delta=getDirection().cross(f->getDirection());
			return delta.norm();
		}

		double delta_dist(const Feature *f) const 
		{
			Eigen::Vector3d delta=getDistance()-f->getDistance();
			return delta.norm();
		}

		void pushIndex(int i) {ptr_indices->push_back(i);}
		void setPtrPoints(ptrPointCloud p) {*ptr_ptr_points=p;}

		void vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, 
				 unsigned char r, unsigned char g, unsigned char b, std::string id_pre="", Transform Tcw=Transform::Identity());

	private:

		std::string id;
		FEATURE_TYPE type;
//		F *feature;
		Plane *plane_;
		Line *line_;
		Landmark *ptr;
		std::vector<int> *ptr_indices;
		ptrPointCloud *ptr_ptr_points;

	};

//	template<class L, class F> 
	class Landmark
	{
	public:

		// for simulation;
		Landmark(PlaneLM *p) : planelm_(p), type(PLANE), id(p->id) {}
		Landmark(LineLM *l) : linelm_(l), type(LINE), id(l->id) {}
		Landmark(Landmark *l) : type(l->Type()), id(l->ID()) 
		{
			if(l->Type()==PLANE) 
			{
				planelm_=new PlaneLM();
				*planelm_=*l->planelm();
			}
			else if(l->Type()==LINE) 
			{
				linelm_=new LineLM();
				*linelm_=*l->linelm();
			}
		} 
		
		// for slam;
		Landmark(Feature *f, Transform Tcg) 
		{
			if(f->Type()==PLANE) planelm_=new PlaneLM(f->plane(),Tcg);
			else if(f->Type()==LINE) linelm_=new LineLM(f->line(),Tcg);
//			landmark=new L(f->feature,Tcg);
			type=f->Type();
			id="map_"+f->ID().substr(0,6);
			f->ptrLandmark()=this;
//			landmark->ptr=this;
		}

		Landmark(std::string s, Eigen::Vector4d p) : id(s), planelm_(new PlaneLM(p)), type(PLANE) {}
		Landmark(std::string s, Vector6d l) : id(s), linelm_(new LineLM(l)), type(LINE) {}

		~Landmark()
		{
			if(type==PLANE) delete planelm_;
			else if(type==LINE) delete linelm_;
		}

		friend std::ostream & operator << (std::ostream &os, const Landmark *l)
		{
			os.precision(6); os<<std::fixed;
			if(l->type==PLANE) os<<l->id<<" "<<l->planelm_;
			else if(l->type==LINE) os<<l->id<<" "<<l->linelm_;

//			os<<" "<<l->observ_time.size()<<" ";
//			for(int i=0;i<l->observ_time.size();i++) os<<l->observ_time[i]<<" "<<l->observ_id[i]<<" ";
			os<<" "<<l->sizeObserv()<<" ";
			for(const_iterObserv it=l->observs.begin();it!=l->observs.end();it++)
			{
				for(int i=0;i<it->second.size();i++)
				{
					os<<it->first<<" "<<it->second[i]<<" ";
				}
			}

			return os;
		}

		std::string ID() const {return id;}
		FEATURE_TYPE Type() const {return type;}

		PlaneLM*& planelm() {return planelm_;}
		LineLM*& linelm() {return linelm_;}

		Eigen::Vector3d getDirection() 
		{
			if(type==PLANE) return planelm_->pi.block<3,1>(0,0);
			else if(type==LINE) return linelm_->L.block<3,1>(3,0);
		}

		void setID(int i)
		{
			if(std::to_string(i).size()==1) id=id+"00"+std::to_string(i);
			else if(std::to_string(i).size()==2) id=id+"0"+std::to_string(i);
			else if(std::to_string(i).size()==3) id=id+std::to_string(i);
		}

		void pushObserv(const double time, const std::string &n)//{observ_time.push_back(time);observ_id.push_back(n);}
		{
			iterObserv it=observs.find(time);
			if(it==observs.end())
			{
				observs.insert(std::pair<double,std::vector<std::string> >(time, std::vector<std::string>(1,n)));
			}
			else 
			{
				observs[time].push_back(n);
			}
		}
		std::vector<std::string> Observ(double t) 
		{
			iterObserv it=observs.find(t);
			if(it==observs.end()) return std::vector<std::string>();
			else return it->second;
		}

//		double observTime(int i) const {return observ_time[i];}
//		std::string observID(int i) const {return observ_id[i];}
		int sizeObserv() const
		{
			int num=0;
			for(const_iterObserv it=observs.begin();it!=observs.end();it++) 
			{ for(int i=0;i<it->second.size();i++) { num++; } }
			return num;
//			return observ_time.size();
		}
		iterObserv beginObserv() {return observs.begin();}
		iterObserv endObserv() {return observs.end();}

	private:
		
		std::string id;
		FEATURE_TYPE type;
//		L *landmark;
		PlaneLM *planelm_;
		LineLM *linelm_;
//		std::vector<double> observ_time;
//		std::vector<std::string> observ_id;
		std::map<double,std::vector<std::string> > observs;
	};

}

#endif
