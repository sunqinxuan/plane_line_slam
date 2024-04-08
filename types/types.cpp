/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-17 12:49
#
# Filename:		types.cpp
#
# Description: 
#
************************************************/
#include "types.h"

namespace ulysses
{
	void Transform::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, double time)
	{
		Eigen::Affine3f coords(getMatrix4f());
		v->removeCoordinateSystem(std::to_string(time)+"CoordsSys");
		v->addCoordinateSystem(0.1,coords,std::to_string(time)+"CoordsSys");
//		pcl::PointXYZRGBA pt1,pt2;
//		double scale=0.1;
//		char id[20];
//		Eigen::Vector3d x=R.block<3,1>(0,0);
//		Eigen::Vector3d y=R.block<3,1>(0,1);
//		Eigen::Vector3d z=R.block<3,1>(0,2);
//		pt1.x=t(0);
//		pt1.y=t(1);
//		pt1.z=t(2);
//		// x - red
//		pt2.x=pt1.x+x(0)*scale;
//		pt2.y=pt1.y+x(1)*scale;
//		pt2.z=pt1.z+x(2)*scale;
//		sprintf(id,"%fx",time);
//		v->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
//		// y - green
//		pt2.x=pt1.x+y(0)*scale;
//		pt2.y=pt1.y+y(1)*scale;
//		pt2.z=pt1.z+y(2)*scale;
//		sprintf(id,"%fy",time);
//		v->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
//		// z - green
//		pt2.x=pt1.x+z(0)*scale;
//		pt2.y=pt1.y+z(1)*scale;
//		pt2.z=pt1.z+z(2)*scale;
//		sprintf(id,"%fz",time);
//		v->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
	}

	Plane::Plane(const PlaneLM *lm, const Transform &Tcw, double sigma)
	{
		id=lm->id.substr(4,6);

		Eigen::Vector4d pi=lm->pi;
		pi/=pi.block<3,1>(0,0).norm();
		pi=Tcw.getPlaneTransform()*pi;
		pi.normalize();

		Eigen::Matrix4d H=pi*pi.transpose();
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(H);
		// the eigenvalues are sorted in increasing order;
		Eigen::Vector4d e1=es.eigenvectors().block<4,1>(0,0);
		Eigen::Vector4d e2=es.eigenvectors().block<4,1>(0,1);
		Eigen::Vector4d e3=es.eigenvectors().block<4,1>(0,2);

		unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(0,sigma);
		double x1=distribution(generator);
		double x2=distribution(generator);
		double x3=distribution(generator);
		Eigen::Vector4d noise=x1*e1+x2*e2+x3*e3; // noise for pi;
//		Vector3d noise_n=noise.block<3,1>(0,0);
//		double noise_d=noise(3);
		
		Eigen::Vector4d p=pi+noise;

		n=p.block<3,1>(0,0);
		d=p(3)/n.norm();
		n.normalize();
	}

	Plane::Plane(const PlaneLM *lm)
	{
		id=lm->id.substr(4,6);

		Eigen::Vector4d p=lm->pi;
		n=p.block<3,1>(0,0);
		d=p(3)/n.norm();
		n.normalize();
	}

	Plane::Plane(const PlaneLM *lm, const Transform &Tcw)
	{
		id=lm->id.substr(4,6);

		Eigen::Vector4d p=lm->pi;
		p/=p.block<3,1>(0,0).norm();
		p=Tcw.getPlaneTransform()*p;

		n=p.block<3,1>(0,0);
		d=p(3);
	}

	double Plane::dist2plane(Plane *p)
	{
		double distance=0;
		for(int i=0;i<p->indices.size();i++)
		{
			distance+=dist2point(Transform::point2eigen(p->ptr_points->at(p->indices[i])));
		}
		distance/=p->indices.size();
		return distance;
	}

	Line::Line(const LineLM *lm, const Transform &Tcw, double sigma)
	{
		id=lm->id.substr(4,6);

		Vector6d LL=Tcw.getLineTransform()*lm->L;
		LL.normalize();

		Vector6d L_dual;
		L_dual.block<3,1>(0,0)=LL.block<3,1>(3,0);
		L_dual.block<3,1>(3,0)=LL.block<3,1>(0,0);

		Matrix6d H=LL*LL.transpose()+L_dual*L_dual.transpose();
		Eigen::SelfAdjointEigenSolver<Matrix6d> es(H);
		// the eigenvalues are sorted in increasing order;
		Vector6d e1=es.eigenvectors().block<6,1>(0,0);
		Vector6d e2=es.eigenvectors().block<6,1>(0,1);
		Vector6d e3=es.eigenvectors().block<6,1>(0,2);
		Vector6d e4=es.eigenvectors().block<6,1>(0,3);

		unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(0,sigma);
		double x1=distribution(generator);
		double x2=distribution(generator);
		double x3=distribution(generator);
		double x4=distribution(generator);
		Vector6d noise=x1*e1+x2*e2+x3*e3+x4*e4; // noise for L;

//		Vector3d noise_v=noise.block<3,1>(3,0);
//		Vector3d noise_u=noise.block<3,1>(0,0);
//		double sd=sqrt(1.0+l->u.norm()*l->u.norm());
//		fp<<"|delta_v|="<<noise_v.norm()*sd*180.0/M_PI<<"\t|noise_u|="<<noise_u.norm()*sd<<endl;
		
		LL+=noise;
//		fp<<"L+noise="<<L.transpose()<<endl;
		LL.normalize();

		v=LL.block<3,1>(3,0);
		u=LL.block<3,1>(0,0)/v.norm();
		v.normalize();
	}

	Line::Line(const LineLM *lm)
	{
		id=lm->id.substr(4,6);

		Vector6d LL=lm->L;
		v=LL.block<3,1>(3,0);
		u=LL.block<3,1>(0,0);
	}

	Line::Line(const LineLM *lm, const Transform &Tcw)
	{
		id=lm->id.substr(4,6);

		Vector6d LL=Tcw.getLineTransform()*lm->L;
		v=LL.block<3,1>(3,0);
		u=LL.block<3,1>(0,0);
	}

	double Line::dist2line(Line *l)
	{
		double distance=0;
		for(int i=0;i<l->indices.size();i++)
		{
			distance+=dist2point(Transform::point2eigen(l->ptr_points->at(l->indices[i])));
		}
		distance/=l->indices.size();
		return distance;
	}

	Feature::Feature(Landmark *lm, const Transform &Tcw, double sigma)
	{
		if(lm->Type()==PLANE) plane_=new Plane(lm->planelm(),Tcw,sigma);
		else if(lm->Type()==LINE) line_=new Line(lm->linelm(),Tcw,sigma);
//			feature=new F(lm->landmark,sigma);
		id=lm->ID().substr(4,6);
		type=lm->Type();
		ptr=lm;
		if(type==PLANE)
		{
			ptr_indices=&plane_->indices;
			ptr_ptr_points=&plane_->ptr_points;
		}
		else
		{
			ptr_indices=&line_->indices;
			ptr_ptr_points=&line_->ptr_points;
		}
	}

	Feature::Feature(Landmark *lm)
	{
		if(lm->Type()==PLANE) plane_=new Plane(lm->planelm());
		else if(lm->Type()==LINE) line_=new Line(lm->linelm());
//			feature=new F(lm->landmark,sigma);
//		id=lm->ID().substr(4,6);
		id=lm->ID();
		type=lm->Type();
		ptr=lm;
		if(type==PLANE)
		{
			ptr_indices=&plane_->indices;
			ptr_ptr_points=&plane_->ptr_points;
		}
		else
		{
			ptr_indices=&line_->indices;
			ptr_ptr_points=&line_->ptr_points;
		}
	}

	Feature::Feature(Landmark *lm, const Transform &Tcw)
	{
		if(lm->Type()==PLANE) plane_=new Plane(lm->planelm(),Tcw);
		else if(lm->Type()==LINE) line_=new Line(lm->linelm(),Tcw);
//			feature=new F(lm->landmark,sigma);
//		id=lm->ID().substr(4,6);
		id=lm->ID();
		type=lm->Type();
		ptr=lm;
		if(type==PLANE)
		{
			ptr_indices=&plane_->indices;
			ptr_ptr_points=&plane_->ptr_points;
		}
		else
		{
			ptr_indices=&line_->indices;
			ptr_ptr_points=&line_->ptr_points;
		}
	}

	void Feature::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, 
					  unsigned char r, unsigned char g, unsigned char b, std::string id_pre, Transform Tcw)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(type==PLANE) pcl::copyPointCloud(*plane_->ptr_points,plane_->indices,*tmp);
		else if(type==LINE) pcl::copyPointCloud(*line_->ptr_points,line_->indices,*tmp);
//		{
//			for(std::list<EdgePoint*>::iterator it=line_->points.begin();it!=line_->points.end();it++)
//			{
//				pcl::PointXYZRGBA p;
//				p.x=(*it)->xyz(0); p.y=(*it)->xyz(1); p.z=(*it)->xyz(2);
//				tmp->push_back(p);
//			}
//		}
		
		pcl::transformPointCloud(*tmp,*tmp,Tcw.inv().getMatrix4f());

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(tmp,r,g,b);
		if (!v->updatePointCloud (tmp, color, id_pre+id)) v->addPointCloud (tmp, color, id_pre+id);
		if(type==LINE) v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id_pre+id);
	}

}
