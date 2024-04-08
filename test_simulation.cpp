/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-10-21 09:27
#
# Filename:		test.cpp
#
# Description: 
#
************************************************/
#include <sys/time.h>
#include <ctime>
#include <random>
#include <chrono>
#include "system_plane_line_shadow.h"
#include "system_extrinsic_calibration.h"
#include "relative_pose_error.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

ulysses::Transform getTranform(Eigen::Vector3d w, Eigen::Vector3d t)
{
	double theta=w.norm();
	Eigen::Vector3d axis=w.normalized();
	Eigen::Matrix3d R=cos(theta)*Eigen::Matrix3d::Identity()
					   + (1.0-cos(theta))*axis*axis.transpose()
					   + sin(theta)*ulysses::Transform::skew_sym(axis);
	ulysses::Transform T(R,t);
	return T;
}

Plane* generatePlane(double nx, double ny, double nz, double d)
{
	Plane *plane=new Plane;
	plane->n(0)=nx;
	plane->n(1)=ny;
	plane->n(2)=nz;
	plane->d=d;
	return plane;
}

Plane* generatePlane(Vector4d pi)
{
	Plane *plane=new Plane;
	Vector3d n=pi.block<3,1>(0,0);
	plane->n=n.normalized();
	plane->d=pi(3)/n.norm();
	return plane;
}

Vector4d getPlaneParam(Plane *p);
void addNoise2Plane(Plane *p, double sigma, ofstream &fp)
{
	Vector4d pi=getPlaneParam(p);
	pi.normalize();
	double d=sqrt(1.0+p->d*p->d);

	Matrix4d H=pi*pi.transpose();
	SelfAdjointEigenSolver<Matrix4d> es(H);
	// the eigenvalues are sorted in increasing order;
	Vector4d e1=es.eigenvectors().block<4,1>(0,0);
	Vector4d e2=es.eigenvectors().block<4,1>(0,1);
	Vector4d e3=es.eigenvectors().block<4,1>(0,2);

	unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::normal_distribution<double> distribution(0,sigma);
	double x1=distribution(generator);
	double x2=distribution(generator);
	double x3=distribution(generator);
	Vector4d noise=x1*e1+x2*e2+x3*e3; // noise for pi;
	Vector3d noise_n=noise.block<3,1>(0,0);
	double noise_d=noise(3);
	fp<<"|delta_n|="<<noise_n.norm()*d*180.0/M_PI<<"\t|noise_d|="<<fabs(noise_d)*d<<endl;
	
	pi+=noise;
	p->n=pi.block<3,1>(0,0);
	p->d=pi(3)/p->n.norm();
	p->n.normalize();
}

Plane* transformPlane(ulysses::Transform T, Plane *p)
{
	Plane *plane=new Plane;
	plane->n=T.R*p->n;
	plane->d=p->d-T.t.dot(plane->n);
	plane->id=p->id;
	return plane;
}

Plane* setPlaneParam(Vector4d pi)
{
	Plane *plane=new Plane;
	plane->n=pi.block<3,1>(0,0);
	plane->d=pi(3);
	return plane;
}

Vector4d getPlaneParam(Plane *p)
{
	Vector4d pi;
	pi.block<3,1>(0,0)=p->n;
	pi(3)=p->d;
	return pi;
}

Line* generateLine(Vector6d L)
{
	Line *line=new Line;
	Vector3d u=L.block<3,1>(0,0); Vector3d uu=u.normalized();
	Vector3d v=L.block<3,1>(3,0); Vector3d vv=v.normalized();
	Vector3d uu_vv=uu-vv; double d=uu_vv.norm(); double r=sqrt(1-0.25*d*d);
	Vector3d U=((d+2*r)*uu+(d-2*r)*vv)*u.norm()/(2*d);
	Vector3d V=((d-2*r)*uu+(d+2*r)*vv)*u.norm()/(2*d);
	L.block<3,1>(0,0)=U;
	L.block<3,1>(3,0)=V;
	L.normalize();

	u=L.block<3,1>(0,0);
	v=L.block<3,1>(3,0);
	line->u=u;
	line->v=v.normalized();
	return line;
}

Vector6d getLineParam(Line *l);
void addNoise2Line(Line *l, double sigma, ofstream &fp)
{
	fp<<"*****************addNoise2Line*************"<<endl;
	Vector6d L=getLineParam(l);
	fp<<"L="<<L.transpose()<<endl;
	L.normalize();
	fp<<"L.normalize()="<<L.transpose()<<endl;
	Vector6d L_dual;
	L_dual.block<3,1>(0,0)=L.block<3,1>(3,0);
	L_dual.block<3,1>(3,0)=L.block<3,1>(0,0);
	fp<<"L_dual="<<L_dual.transpose()<<endl;

	Matrix6d H=L*L.transpose()+L_dual*L_dual.transpose();
	SelfAdjointEigenSolver<Matrix6d> es(H);
	fp<<"H eigenvalues="<<es.eigenvalues().transpose()<<endl;
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
	fp<<"noise="<<noise.transpose()<<endl;

	Vector3d noise_v=noise.block<3,1>(3,0);
	Vector3d noise_u=noise.block<3,1>(0,0);
	double sd=sqrt(1.0+l->u.norm()*l->u.norm());
	fp<<"|delta_v|="<<noise_v.norm()*sd*180.0/M_PI<<"\t|noise_u|="<<noise_u.norm()*sd<<endl;
	
	L+=noise;
	fp<<"L+noise="<<L.transpose()<<endl;
//	Vector3d u=L.block<3,1>(0,0);
//	Vector3d uu=u.normalized();
//	Vector3d v=L.block<3,1>(3,0);
//	Vector3d vv=v.normalized();
//	fp<<"u.dot(v)="<<uu.dot(vv)<<endl;
//	Vector3d uu_vv=uu-vv;
//	double d=uu_vv.norm();
//	double r=sqrt(1-0.25*d*d);
//	Vector3d U=((d+2*r)*uu+(d-2*r)*vv)*u.norm()/(2*d);
//	Vector3d V=((d-2*r)*uu+(d+2*r)*vv)*u.norm()/(2*d);
//	fp<<"U.dot(V)="<<U.normalized().dot(V.normalized())<<endl;
//	L.block<3,1>(0,0)=U;
//	L.block<3,1>(3,0)=V;
	L.normalize();
	fp<<"L.normalize()="<<L.transpose()<<endl;

	l->v=L.block<3,1>(3,0);
	l->u=L.block<3,1>(0,0)/l->v.norm();
	l->v.normalize();
	L=getLineParam(l);
	fp<<"L="<<L.transpose()<<endl;
	fp<<"*******************************************"<<endl;
}

Line* transformLine(ulysses::Transform T, Line *l)
{
	Line *line=new Line;
	line->v=T.R*l->v;
	line->u=T.R*l->u+T.t.cross(line->v);
	line->id=l->id;
	return line;
}

Line* setLineParam(Vector6d L)
{
	Line *line=new Line;
	Vector3d u=L.block<3,1>(0,0);
	Vector3d v=L.block<3,1>(3,0);
	line->u=u;
	line->v=v;
	return line;
}

Vector6d getLineParam(Line *l)
{
	Vector6d L;
	L.block<3,1>(0,0)=l->u;
	L.block<3,1>(3,0)=l->v;
	return L;
}

void printNode(Node_InterpTree *n, ofstream &fp)
{
	fp<<"ref: "; n->feature_ref->print(fp);
	fp<<"cur: "; n->feature_cur->print(fp);
	fp<<std::endl;
}
	
//extern double THRES_RAD, THRES_DIST;

int main(int argc, char *argv[])
{
	vtkObject::GlobalWarningDisplayOff();

	bool loadSavedData=false, loadNoisyData=true;
	std::string filename_write, filename_read;
	THRES_RAD=5.0*M_PI/180.0;
	THRES_DIST=0.09;
	double sigma=0.001;
	int NumPlane=10, NumLine=10;

	cv::FileStorage settings("settings.yaml",cv::FileStorage::READ);
	settings["loadSavedData"]>>loadSavedData;
	settings["loadNoisyData"]>>loadNoisyData;
	settings["FileWrite"]>>filename_write;
	settings["FileRead"]>>filename_read;
	settings["THRES_RAD"]>>THRES_RAD; THRES_RAD*=(M_PI/180.0);
	settings["THRES_DIST"]>>THRES_DIST;
	settings["sigma"]>>sigma;

//	for(int i=1;i<argc;i++)
//	{
//		if(strcmp(argv[i],"-load")==0)
//		{
//			loadSavedData=true;
//			filename=argv[i+1];
//		}
//		else if(strcmp(argv[i],"-save")==0)
//		{
//			filename=argv[i+1];
//		}
//		else if(strcmp(argv[i],"-rad")==0)
//		{
//			THRES_RAD=atof(argv[i+1])*M_PI/180.0;
//		}
//		else if(strcmp(argv[i],"-dist")==0)
//		{
//			THRES_DIST=atof(argv[i+1]);
//		}
//		else if(strcmp(argv[i],"-sigma")==0)
//		{
//			loadNoisyData=false;
//			sigma=atof(argv[i+1]);
//		}
//	}

	ofstream fp;
	fp.open("test.txt",std::ios::out);
	remove("ProblemRotation.txt");
	remove("ProblemTranslation.txt");

	IntrinsicParam cam;
	Scan *scan=new Scan(1,cam,0);
	scan->scan_ref=new Scan(0,cam,0);
	InterpTree *tree=new InterpTree;

	cv::FileStorage fs_write, fs_read;

	Vector3d w,t;
	cv::Mat axis_angle, translation, plane_param, line_param;
	ulysses::Transform Tcr;
	Vector6d L; 
	Vector4d pi;
	string name;
	ostringstream os;

	if(loadSavedData)
	{
		if(loadNoisyData)
		{
			fs_read.open(filename_read,cv::FileStorage::READ);
			fs_read["axis_angle"]>>axis_angle; cv::cv2eigen(axis_angle,w);
			fs_read["translation"]>>translation; cv::cv2eigen(translation,t);
			fs_read["num_plane"]>>NumPlane;
			fs_read["num_line"]>>NumLine;

			Tcr=getTranform(w,t);

			for(int i=0;i<NumPlane;i++)
			{
				os<<"ref_plane_"<<i; name=os.str(); os.str("");
				fs_read[name]>>plane_param;
				cv::cv2eigen(plane_param,pi);
				Plane* plane=setPlaneParam(pi); plane->id=i;
				scan->scan_ref->observed_planes.push_back(plane);

				os<<"cur_plane_"<<i; name=os.str(); os.str("");
				fs_read[name]>>plane_param;
				cv::cv2eigen(plane_param,pi);
				plane=setPlaneParam(pi); plane->id=i;
				scan->observed_planes.push_back(plane);
			}
			for(int i=0;i<NumLine;i++)
			{
				os<<"ref_line_"<<i; name=os.str(); os.str("");
				fs_read[name]>>line_param;
				cv::cv2eigen(line_param,L);
				Line* line=setLineParam(L); line->id=i;
				scan->scan_ref->lines_occluding.push_back(line);

				os<<"cur_line_"<<i; name=os.str(); os.str("");
				fs_read[name]>>line_param;
				cv::cv2eigen(line_param,L);
				line=setLineParam(L); line->id=i;
				scan->lines_occluding.push_back(line);
			}
		}
		else 
		{
			fs_read.open(filename_read,cv::FileStorage::READ);
			fs_read["axis_angle"]>>axis_angle; cv::cv2eigen(axis_angle,w);
			fs_read["translation"]>>translation; cv::cv2eigen(translation,t);
			fs_read["num_plane"]>>NumPlane;
			fs_read["num_line"]>>NumLine;

			Tcr=getTranform(w,t);

			fs_write.open(filename_write,cv::FileStorage::WRITE);
			fs_write<<"axis_angle"<<axis_angle;
			fs_write<<"translation"<<translation;
			fs_write<<"num_plane"<<NumPlane;
			fs_write<<"num_line"<<NumLine;
			fs_write<<"sigma"<<sigma;

			for(int i=0;i<NumPlane;i++)
			{
				os<<"ref_plane_org_"<<i; name=os.str(); os.str("");
				fs_read[name]>>plane_param;
				fs_write<<name<<plane_param;
				cv::cv2eigen(plane_param,pi);
				Plane* plane=setPlaneParam(pi); plane->id=i;
				addNoise2Plane(plane,sigma,fp);
				scan->scan_ref->observed_planes.push_back(plane);

				pi=getPlaneParam(plane);
				cv::eigen2cv(pi,plane_param);
				os<<"ref_plane_"<<i; name=os.str(); os.str("");
				fs_write<<name<<plane_param;

				os<<"cur_plane_org_"<<i; name=os.str(); os.str("");
				fs_read[name]>>plane_param;
				fs_write<<name<<plane_param;
				cv::cv2eigen(plane_param,pi);
				plane=setPlaneParam(pi); plane->id=i;
				addNoise2Plane(plane,sigma,fp);
				scan->observed_planes.push_back(plane);

				pi=getPlaneParam(plane);
				cv::eigen2cv(pi,plane_param);
				os<<"cur_plane_"<<i; name=os.str(); os.str("");
				fs_write<<name<<plane_param;
			}
			for(int i=0;i<NumLine;i++)
			{
				os<<"ref_line_org_"<<i; name=os.str(); os.str("");
				fs_read[name]>>line_param;
				fs_write<<name<<line_param;
				cv::cv2eigen(line_param,L);
				Line* line=setLineParam(L); line->id=i;
				addNoise2Line(line,sigma,fp);
				scan->scan_ref->lines_occluding.push_back(line);

				L=getLineParam(line);
				cv::eigen2cv(L,line_param);
				os<<"ref_line_"<<i; name=os.str(); os.str("");
				fs_write<<name<<line_param;

				os<<"cur_line_org_"<<i; name=os.str(); os.str("");
				fs_read[name]>>line_param;
				fs_write<<name<<line_param;
				cv::cv2eigen(line_param,L);
				line=setLineParam(L); line->id=i;
				addNoise2Line(line,sigma,fp);
				scan->lines_occluding.push_back(line);

				L=getLineParam(line);
				cv::eigen2cv(L,line_param);
				os<<"cur_line_"<<i; name=os.str(); os.str("");
				fs_write<<name<<line_param;
			}
			
		}
	}
	else 
	{
		settings["NumPlane"]>>NumPlane;
		settings["NumLine"]>>NumLine;

		// camera motion generation;
		srand((unsigned)time(NULL));
		w=Eigen::Vector3d::Random();
		t=Eigen::Vector3d::Random();
		double angle=w.norm();
		if(angle>M_PI)
		{
			int k=int(angle/M_PI);
			angle=angle-k*M_PI;
		}
		w.normalize();
		w*=angle;

		Tcr=getTranform(w,t);

		fs_write.open(filename_write,cv::FileStorage::WRITE);
		cv::eigen2cv(w,axis_angle);
		cv::eigen2cv(t,translation);
		fs_write<<"axis_angle"<<axis_angle;
		fs_write<<"translation"<<translation;
		fs_write<<"num_plane"<<NumPlane;
		fs_write<<"num_line"<<NumLine;
		fs_write<<"sigma"<<sigma;

		// plane observation generation;
		for(int i=0;i<NumPlane;i++)
		{
			Plane* plane=generatePlane(Vector4d::Random()); plane->id=i;
			Plane* plane_trans=transformPlane(Tcr,plane);

			pi=getPlaneParam(plane);
			cv::eigen2cv(pi,plane_param);
			os<<"ref_plane_org_"<<i; name=os.str(); os.str("");
			fs_write<<name<<plane_param;

			pi=getPlaneParam(plane_trans);
			cv::eigen2cv(pi,plane_param);
			os<<"cur_plane_org_"<<i; name=os.str(); os.str("");
			fs_write<<name<<plane_param;

			addNoise2Plane(plane,sigma,fp);
			addNoise2Plane(plane_trans,sigma,fp);
			scan->scan_ref->observed_planes.push_back(plane);
			scan->observed_planes.push_back(plane_trans);

			pi=getPlaneParam(plane);
			cv::eigen2cv(pi,plane_param);
			os<<"ref_plane_"<<i; name=os.str(); os.str("");
			fs_write<<name<<plane_param;

			pi=getPlaneParam(plane_trans);
			cv::eigen2cv(pi,plane_param);
			os<<"cur_plane_"<<i; name=os.str(); os.str("");
			fs_write<<name<<plane_param;

		}
		// line observation generation;
		for(int i=0;i<NumLine;i++)
		{
			Line* line=generateLine(Vector6d::Random()); line->id=i;
			Line* line_trans=transformLine(Tcr,line);

			{
				fp<<"--------------------generate"<<endl;
				Vector3d tv=Tcr.R*line->v;
				Vector3d dv=line_trans->v-tv;
				fp<<line_trans->v.transpose()<<"\t"<<tv.transpose()<<"\t"
				  <<dv.transpose()<<"\t"<<dv.norm()<<std::endl;

				Vector3d tu=Tcr.R*line->u-ulysses::Transform::skew_sym(line_trans->v)*Tcr.t;
				Vector3d du=line_trans->u-tu;
				fp<<line_trans->u.transpose()<<"\t"<<tu.transpose()<<"\t"
				  <<du.transpose()<<"\t"<<du.norm()<<std::endl;

				tu=Tcr.R*line->u+Tcr.t.cross(line_trans->v);
				du=line_trans->u-tu;
				fp<<line_trans->u.transpose()<<"\t"<<tu.transpose()<<"\t"
				  <<du.transpose()<<"\t"<<du.norm()<<std::endl;

				fp<<std::endl;
			}

			L=getLineParam(line);
			cv::eigen2cv(L,line_param);
			os<<"ref_line_org_"<<i; name=os.str(); os.str("");
			fs_write<<name<<line_param;

			L=getLineParam(line_trans);
			cv::eigen2cv(L,line_param);
			os<<"cur_line_org_"<<i; name=os.str(); os.str("");
			fs_write<<name<<line_param;

			addNoise2Line(line,sigma,fp);
			addNoise2Line(line_trans,sigma,fp);

			{
				fp<<"--------------------addNoise"<<endl;
				Vector3d tv=Tcr.R*line->v;
				Vector3d dv=line_trans->v-tv;
				fp<<line_trans->v.transpose()<<"\t"<<tv.transpose()<<"\t"
				  <<dv.transpose()<<"\t"<<dv.norm()<<std::endl;

				Vector3d tu=Tcr.R*line->u-ulysses::Transform::skew_sym(line_trans->v)*Tcr.t;
				Vector3d du=line_trans->u-tu;
				fp<<line_trans->u.transpose()<<"\t"<<tu.transpose()<<"\t"
				  <<du.transpose()<<"\t"<<du.norm()<<std::endl;

				tu=Tcr.R*line->u+Tcr.t.cross(line_trans->v);
				du=line_trans->u-tu;
				fp<<line_trans->u.transpose()<<"\t"<<tu.transpose()<<"\t"
				  <<du.transpose()<<"\t"<<du.norm()<<std::endl;

				fp<<std::endl;
			}

			scan->scan_ref->lines_occluding.push_back(line);
			scan->lines_occluding.push_back(line_trans);

			L=getLineParam(line);
			cv::eigen2cv(L,line_param);
			os<<"ref_line_"<<i; name=os.str(); os.str("");
			fs_write<<name<<line_param;

			L=getLineParam(line_trans);
			cv::eigen2cv(L,line_param);
			os<<"cur_line_"<<i; name=os.str(); os.str("");
			fs_write<<name<<line_param;
		}
	}
	fs_read.release();
	fs_write.release();
	settings.release();

	fp<<"rotation axis-angle: "<<w.transpose()<<endl;
	fp<<"rotation axis: "<<w.normalized().transpose()<<endl;
	fp<<"rotation angle: "<<w.norm()*180.0/M_PI<<" deg"<<endl;
	fp<<"translation: "<<t.transpose()<<endl<<endl;


	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0.1, 0.1, 0.1);
	vis->initCameraParameters ();

	scan->Rotation_PCA.col(0)=w;
	tree->construct(scan);
	tree->printNodes(fp);

	fp.close();
	return 0;
}
