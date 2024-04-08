/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-21 08:43
#
# Filename:		geometric_feature_matching.cpp
#
# Description: 
#
===============================================*/
#include "features/geometric_feature_matching.h"

namespace ulysses
{
	bool dbg;
	std::ofstream fp_IT;
	double THRES_RAD, THRES_DIST;
	Eigen::Vector3d axis_angle;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis ;

	Node_InterpTree::~Node_InterpTree() 
	{ 
		if(!null) delete consistent_model;
	}

	void Node_InterpTree::initConsistentModel() 
	{
		consistent_model=new ModelConsistency;
		consistent_model->initializeModel(this);
	}

	void Node_InterpTree::deriveConsistentModel(ModelConsistency* model) 
	{
		consistent_model=new ModelConsistency;
		consistent_model->deriveFrom(model);
	}

	bool Node_InterpTree::updateConsistentModel(ModelConsistencyBinary *model) {return consistent_model->update(model);}
	double Node_InterpTree::error() { return consistent_model->transError()+consistent_model->rotError(); }

//	bool Node_InterpTree::updateConsistentModel(ModelConsistencyBinary *model) 
//	{ 
////		bool updated=false;
////		if(!model->IsConsistent()) 
////		{
////			updated=false;
////		}
////		if(consistent_model->IsEmpty() && model->IsEmpty())
////		{
////			updated=false;
////		}
////		else if(!consistent_model->IsEmpty() && model->IsEmpty())
////		{
////			updated=true;
////		}
////		else if(consistent_model->IsEmpty() && !model->IsEmpty())
////		{
////			consistent_model->copyFrom(model);
//////			delete consistent_model;
//////			consistent_model=model;
////			updated=true;
////		}
////		else 
////		{
////			updated=consistent_model->update(model); 
////		}
////		fp_IT<<std::endl<<"updateConsistentModel() => "<<updated<<std::endl;
////		return updated;
//		return true;
//	}
//
//	bool Node_InterpTree::updateConsistentModel(Node_InterpTree *node) 
//	{ 
////		return updateConsistentModel(node->consistent_model);
//		return true;
//	}


//	bool Node_InterpTree::modelConsistent(Node_InterpTree* node)
//	{}
//
//	bool Node_InterpTree::spatialConsistent(Node_InterpTree* node)
//	{}

	bool ProblemRotation::solve()
	{
		if(dbg) 
		{
			fp_IT<<std::endl<<"-----------------------ProblemRotation::solve()-----------------------"<<std::endl;
			fp_IT<<"nri(vri) = "<<nri.transpose()<<std::endl;
			fp_IT<<"nrj(vrj) = "<<nrj.transpose()<<std::endl;
			fp_IT<<"nci(vci) = "<<nci.transpose()<<std::endl;
			fp_IT<<"ncj(vcj) = "<<ncj.transpose()<<std::endl;
		}

		isConsistent=false;
		hasDOF=false;
		bool isConsistent_local=false;
		Eigen::Vector3d axis_local=Eigen::Vector3d::Zero();
		double angle_local=0, theta_i=0, theta_j=0;
		double error_local;

		double delta_cur=included_angle(nci,ncj), delta_ref=included_angle(nri,nrj);
		if(dbg) fp_IT<<"included angle in current is "<<delta_cur*180.0/M_PI<<std::endl
				  <<"included angle in reference is "<<delta_ref*180.0/M_PI<<std::endl;
		if(fabs(delta_cur-delta_ref)>THRES_RAD)
		{
			if(dbg) fp_IT<<"included angle NOT equal"<<std::endl;
			return false;
		}
		else if(delta_cur<THRES_RAD && delta_ref<THRES_RAD)
		{
			if(dbg) fp_IT<<"delta_cur<THRES_RAD && delta_ref<THRES_RAD"<<std::endl;
//			if()
			{
				isConsistent_local=solveRotation(axis_local,angle_local,error_local);
			}
			if(!isConsistent_local || error_local>0.01)
			{
				if(dbg) fp_IT<<"included angles are both zero => consistent, hasDOF"<<std::endl;
				// nci and ncj are parallel planes;
				// infinite valid rotations;
				// 1-DoF undefined;
				isConsistent_local=true;
				hasDOF=true;
	//			normal=(nci+ncj)-(nri+nrj);
	//			normal.normalize();
	//			fp_IT<<"normal="<<normal.transpose()<<std::endl;
				nc=nci+ncj; nc.normalize();
				nr=nri+nrj; nr.normalize();
				axis_local=nr.cross(nc);
				axis_local.normalize();
				angle_local=rotation_angle(axis_local,nci,nri);
				if(dbg) fp_IT<<"axis=nrixnci="<<axis_local.transpose()<<std::endl
						  <<"angle="<<angle_local*180.0/M_PI<<std::endl;
				
//				Eigen::Vector3d dvi=nci-rotation_matrix(angle_local*axis_local)*nri;
//				Eigen::Vector3d dvj=ncj-rotation_matrix(angle_local*axis_local)*nrj;
//				fp_IT<<"|nci-R*nri|="<<dvi.norm()<<std::endl;
//				fp_IT<<"|ncj-R*nrj|="<<dvj.norm()<<std::endl;
				error_local=residual(axis_local,angle_local);
				if(dbg) fp_IT<<"rotation error="<<error_local<<std::endl;
			}

			// the axis-angle is returned by the function rot_axis_angle_1dof(phi);
//			fp_IT.close();
//			return true;
		}
		else 
		{
			isConsistent_local=solveRotation(axis_local,angle_local,error_local);
		}
		
		if(isConsistent_local)
		{
			if(angle_local<0)
			{
				angle_local=-angle_local;
				axis_local=-axis_local;
			}
		}

		isConsistent=isConsistent_local;
		axis=axis_local;
		angle=angle_local;
//		double error=residual(axis,angle);
//		if(isConsistent) fp_IT<<"average residual for rotation is "<<error<<std::endl<<std::endl;

		return isConsistent;
	}

	bool ProblemRotation::solveRotation(Eigen::Vector3d &axis_local, double &angle_local, double &error_local)
	{
		bool isConsistent_local=false;
		Eigen::Vector3d delta_i=nci-nri, delta_j=ncj-nrj;
		if(dbg) fp_IT<<"nci-nri (<nci,nri> in deg) - "<<delta_i.transpose()<<" ("<<included_angle(nri,nci)*180.0/M_PI<<")"<<std::endl
				  <<"ncj-nrj (<ncj,nrj> in deg) - "<<delta_j.transpose()<<" ("<<included_angle(nrj,ncj)*180.0/M_PI<<")"<<std::endl;
		double delta_i_norm=included_angle(nri,nci);//delta_i.norm();
		double delta_j_norm=included_angle(nrj,ncj);//delta_j.norm();
		delta_i.normalize();
		delta_j.normalize();
		axis_local=delta_i.cross(delta_j);
		if(dbg) fp_IT<<"|(nci-nri)x(ncj-nrj)|="<<axis_local.norm()<<std::endl;

		if(delta_i_norm>THRES_RAD && delta_j_norm>THRES_RAD)
		{
			if(dbg) fp_IT<<"<nci,nri> >THRES_RAD && <ncj,nrj> >THRES_RAD"<<std::endl;
			isConsistent_local=CaseI_CaseII(axis_local,angle_local,error_local);
		}
		else if(delta_i_norm<=THRES_RAD && delta_j_norm>THRES_RAD)
		{
			if(dbg) fp_IT<<"<nci,nri> <=THRES_RAD && <ncj,nrj> >THRES_RAD"<<std::endl;
			isConsistent_local=CaseI_CaseII(axis_local,angle_local,error_local);
			if(!isConsistent_local)
			{
				if(dbg) fp_IT<<"Case III: (nci-nri)=0 => rotation consistent"<<std::endl;
				isConsistent_local=true;
				axis_local=nci+nri;
				axis_local.normalize();
				if(dbg) fp_IT<<"rotation axis_local - "<<axis_local.transpose()<<std::endl;
				angle_local=rotation_angle(axis_local,ncj,nrj);
				if(angle_local>M_PI) angle_local=-(2.0*M_PI-angle_local);
				if(dbg) fp_IT<<"rotation angle_local - "<<angle_local*180/M_PI<<std::endl;
				error_local=residual(axis_local,angle_local);
				if(dbg) fp_IT<<"rotation error="<<error_local<<std::endl;
			}
//			else 
//			{
//				angle_local=theta_j;
//				fp_IT<<"rotation angle is "<<angle_local*180.0/M_PI<<std::endl;
//			}
		}
		else if(delta_i_norm>THRES_RAD && delta_j_norm<=THRES_RAD)
		{
			if(dbg) fp_IT<<"<nci,nri> >THRES_RAD && <ncj,nrj> <=THRES_RAD"<<std::endl;
			isConsistent_local=CaseI_CaseII(axis_local,angle_local,error_local);
			if(!isConsistent_local)
			{
				if(dbg) fp_IT<<"Case III: (ncj-nrj)=0"<<std::endl
						  <<"-------------------"<<std::endl
						  <<"rotation consistent"<<std::endl;
				isConsistent_local=true;
				axis_local=ncj+nrj;
				axis_local.normalize();
				if(dbg) fp_IT<<"rotation axis_local - "<<axis_local.transpose()<<std::endl;
				angle_local=rotation_angle(axis_local,nci,nri);
				if(angle_local>M_PI) angle_local=-(2.0*M_PI-angle_local);
				if(dbg) fp_IT<<"rotation angle_local - "<<angle_local*180/M_PI<<std::endl;
				error_local=residual(axis_local,angle_local);
				if(dbg) fp_IT<<"rotation error="<<error_local<<std::endl;
			}
//			else 
//			{
//				angle_local=theta_i;
//				fp_IT<<"rotation angle is "<<angle_local*180.0/M_PI<<std::endl;
//			}
		}
		else if(delta_i_norm<=THRES_RAD && delta_j_norm<=THRES_RAD)
		{
			if(dbg) fp_IT<<"<nci,nri> <=THRES_RAD && <ncj,nrj> <=THRES_RAD"<<std::endl;
			isConsistent_local=CaseI_CaseII(axis_local,angle_local,error_local);
			if(!isConsistent_local)
			{
				if(dbg) fp_IT<<"Case III: (nci-nri)=0 && (ncj-nrj)=0"<<std::endl
						  <<"-------------------"<<std::endl
						  <<"rotation consistent"<<std::endl;
				isConsistent_local=true;
				axis_local.setZero();
				angle_local=0;
				if(dbg) fp_IT<<"rotation = I3"<<std::endl;
				error_local=residual(axis_local,angle_local);
				if(dbg) fp_IT<<"rotation error="<<error_local<<std::endl;
			}
		}
		return isConsistent_local;
	}

	bool ProblemRotation::CaseI_CaseII(Eigen::Vector3d &axis_local, double &angle_local, double &error_local)
	{
		bool isConsistent_local;
		if(axis_local.norm()>THRES_RAD)
		{
			isConsistent_local=CaseI(axis_local,angle_local,error_local);
		}
		else //if(axis_local.norm()<=THRES_RAD)
		{
			Eigen::Vector3d axis1=axis_local, axis2=axis_local;
			double angle1, angle2, error1, error2;
			bool consist1=false, consist2=false;
			// <nci-nri,ncj-nrj> > 0.57deg, run CaseI;
			if(axis_local.norm()>0.01) consist1=CaseI (axis1,angle1,error1);
			consist2=CaseII(axis2,angle2,error2);
			if(consist1 && consist2)
			{
				if(dbg) fp_IT<<"CaseI and CaseII both return true"<<std::endl
						  <<"error1="<<error1<<std::endl
						  <<"error2="<<error2<<std::endl;
//				if(fabs(thetai1-thetaj1)<fabs(thetai2-thetaj2))
				if(error1<error2)
				{
					if(dbg) fp_IT<<"choose CaseI"<<std::endl;
					isConsistent_local=consist1;
					axis_local=axis1; angle_local=angle1;
					error_local=error1;
				}
				else
				{
					if(dbg) fp_IT<<"choose CaseII"<<std::endl;
					isConsistent_local=consist2;
					axis_local=axis2; angle_local=angle2;
					error_local=error2;
				}
			}
			else if(consist1)
			{
				if(dbg) fp_IT<<"CaseI returns true"<<std::endl;
				isConsistent_local=consist1;
				axis_local=axis1; angle_local=angle1;
				error_local=error1;
			}
			else if(consist2)
			{
				if(dbg) fp_IT<<"CaseII returns true"<<std::endl;
				isConsistent_local=consist2;
				axis_local=axis2; angle_local=angle2;
				error_local=error2;
			}
			else 
			{
				if(dbg) fp_IT<<"CaseI and CaseII both return false"<<std::endl;
				isConsistent_local=false;
				error_local=DBL_MAX;
			}
		}
		if(dbg) fp_IT<<"rotation error="<<error_local<<std::endl;
		return isConsistent_local;
	}

	bool ProblemRotation::CaseI(Eigen::Vector3d &rot_axis, double &rot_angle, double &rot_error)
	{
		if(rot_axis.norm()<0.01) return false;
		if(dbg) fp_IT<<"Case I: (nci-nri) and (ncj-nrj) are NOT (anti-)parallel"<<std::endl;
		rot_axis.normalize();
		if(dbg) fp_IT<<"axis="<<rot_axis.transpose()<<std::endl;

		double theta_i=rotation_angle(rot_axis,nci,nri);
		double theta_j=rotation_angle(rot_axis,ncj,nrj);

		if(dbg) fp_IT<<"rotation angles in deg - theta_i="<<theta_i*180.0/M_PI<<", theta_j="<<theta_j*180.0/M_PI<<std::endl
				  <<"|theta_i-theta_j|="<<fabs(theta_i-theta_j)*180.0/M_PI<<std::endl;
		if(fabs(theta_i-theta_j)<THRES_RAD)
		{
			rot_angle=(theta_i+theta_j)*0.5;
			if(rot_angle>M_PI) rot_angle=-(2.0*M_PI-rot_angle);
			rot_error=residual(rot_axis,rot_angle);
			if(dbg) 
			{
				fp_IT<<"theta_i=theta_j => rotation consistent"<<std::endl;
				fp_IT<<"rotation axis - "<<rot_axis.transpose()<<std::endl;
				fp_IT<<"rotation angle - "<<rot_angle*180/M_PI<<std::endl;
				fp_IT<<"rotation error="<<rot_error<<std::endl;
			}
			return true;
		}
		else
		{
			if(dbg) fp_IT<<"theta_i!=theta_j => rotation NOT consistent"<<std::endl;
			rot_error=DBL_MAX;
			return false;
		}
	}

	bool ProblemRotation::CaseII(Eigen::Vector3d &rot_axis, double &rot_angle, double &rot_error)
	{
		Eigen::Vector3d cross_i=nri.cross(nci); cross_i.normalize();
		Eigen::Vector3d cross_j=nrj.cross(ncj); cross_j.normalize();
		Eigen::Vector3d plus_i=nri+nci; plus_i.normalize();
		Eigen::Vector3d plus_j=nrj+ncj; plus_j.normalize();

		double delta=atan2(cross_j.dot(plus_i),cross_j.dot(cross_i));
		if(fabs(delta)<0.01) return false;

		if(dbg) fp_IT<<"Case II: (nci-nri) and (ncj-nrj) are (anti-)parallel"<<std::endl
				  <<" => rotation consistent"<<std::endl;
		double alpha_i=0.5*included_angle(nri,nci);
		double alpha_j=0.5*included_angle(nrj,ncj);
		if(dbg) fp_IT<<"2*alpha_i<nri,nci>="<<2*alpha_i*180.0/M_PI<<", 2*alpha_j<nrj,ncj>="<<2*alpha_j*180.0/M_PI<<std::endl;

		double delta1=atan2(plus_j.dot(plus_i),plus_j.dot(cross_i));
		if(dbg) fp_IT<<"delta<nrixnci,nrjxncj>="<<delta*180.0/M_PI<<std::endl;
		if(dbg) fp_IT<<"delta'<nrixnci,nrj+ncj>="<<delta1*180.0/M_PI<<std::endl;
		double gamma_i=atan((tan(alpha_j)/tan(alpha_i)-cos(delta))/sin(delta));
		double gamma_j=atan(-(tan(alpha_i)/tan(alpha_j)-cos(delta))/sin(delta));
		if(dbg) fp_IT<<"gamma_i="<<gamma_i*180.0/M_PI<<", gamma_j="<<gamma_j*180.0/M_PI<<std::endl<<std::endl;
		// rot_axis=cross_i*(sin(gamma_j)/sin(delta))+cross_j*(sin(gamma_i)/sin(delta));///////////////
		rot_axis=cross_i*cos(gamma_i)+plus_i*sin(gamma_i); rot_axis.normalize();
		Eigen::Vector3d axis1=cross_j*cos(gamma_j)+plus_j*cos(delta1-delta-gamma_j); axis1.normalize();
		double thetai=rotation_angle(rot_axis,nci,nri);
		double thetaj=rotation_angle(rot_axis,ncj,nrj);
		double thetai1=rotation_angle(axis1,nci,nri);
		double thetaj1=rotation_angle(axis1,ncj,nrj);
		double theta_i, theta_j;
		if(fabs(thetai-thetaj)<fabs(thetai1-thetaj1))
		{
			rot_angle=(thetai+thetaj)/2;
			if(dbg) fp_IT<<"theta_i="<<thetai*180.0/M_PI<<", theta_j="<<thetaj*180.0/M_PI<<std::endl;
			theta_i=thetai;
			theta_j=thetaj;
		}
		else 
		{
			rot_axis=axis1;
			rot_angle=(thetai1+thetaj1)/2;
			if(dbg) fp_IT<<"theta_i'="<<thetai1*180.0/M_PI<<", theta_j'="<<thetaj1*180.0/M_PI<<std::endl;
			theta_i=thetai1;
			theta_j=thetaj1;
		}
		rot_error=residual(rot_axis,rot_angle);
		if(dbg) 
		{
			fp_IT<<"rotation axis - "<<rot_axis.transpose()<<std::endl;
			fp_IT<<"rotation angle' - "<<rot_angle*180/M_PI<<std::endl<<std::endl;
			fp_IT<<"rotation error="<<rot_error<<std::endl;
		}

		return true;

//				// **********************************************
//				char id[20];
//				axis_angle.normalize();
//				vis->removeAllShapes();
//				pcl::PointXYZRGBA pt1,pt2;
//				pt1.x=0; pt1.y=0; pt1.z=0;
//				pt2.x=cross_i(0); pt2.y=cross_i(1); pt2.z=cross_i(2);
//				sprintf(id,"cross_i");
//				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
//				pt2.x=cross_j(0); pt2.y=cross_j(1); pt2.z=cross_j(2);
//				sprintf(id,"cross_j");
//				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
//				pt2.x=rot_axis(0); pt2.y=rot_axis(1); pt2.z=rot_axis(2);
//				sprintf(id,"r");
//				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);
//				axis_angle.normalize();
//				pt2.x=axis_angle(0); pt2.y=axis_angle(1); pt2.z=axis_angle(2);
//				sprintf(id,"rot_axis");
//				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
//				vis->spin();
	}


	bool ProblemRotation::fuseRotation(const ProblemRotation *rotation)
	{
//		std::ofstream fp_IT;
//		fp_IT.open("ProblemRotation.txt",std::ios::app);
		fp_IT<<std::endl<<"-----------------------ProblemRotation::fuseRotation()-----------------------"<<std::endl;

		fp_IT<<"isConsistent: "<<isConsistent<<", "<<rotation->IsConsistent()<<std::endl;
		if(!isConsistent || !rotation->IsConsistent()) 
		{
//			fp_IT.close();
			return false;
		}

		bool equal=false;
		if(hasDOF && rotation->HasDOF())
		{
			fp_IT<<"both HasDOF"<<std::endl;
			ProblemRotation problem(get_nr(),rotation->get_nr(),get_nc(),rotation->get_nc());
			equal=problem.solve();
			if(equal) 
			{
				*this=problem;
				fp_IT<<"updated this rotation_consistency"<<std::endl;
			}
//
//			Eigen::Vector3d nu1=normal;
//			Eigen::Vector3d nu2=rotation->getNormal();
//			fp_IT<<"nu1="<<nu1.transpose()<<std::endl;
//			fp_IT<<"nu2="<<nu2.transpose()<<std::endl;
////			fp_IT<<"nu1xnu2="<<nu1.cross(nu2).transpose()<<std::endl;
////			fp_IT<<"|nu1xnu2|="<<nu1.cross(nu2).norm()<<std::endl;
////			fp_IT<<"THRES_RAD="<<THRES_RAD<<std::endl;
//			if(nu1.cross(nu2).norm()<THRES_RAD)
//			{
//				fp_IT<<"nu1xnu2=0 => hasDOF"<<std::endl;
//				normal=nu1;
//				fp_IT<<"normal="<<normal.transpose()<<std::endl;
//				equal=true;
//			}
//			else 
//			{
//				fp_IT<<"nu1!=nu2"<<std::endl;
//				Eigen::Vector3d r=nu1.cross(nu2);
//				r.normalize();
//				fp_IT<<"r="<<r.transpose()<<std::endl;
//				double theta1=rotation_angle(r,0.5*(nci+ncj),0.5*(nri+nrj));
//				double theta2=rotation_angle(r,rotation->get_nc(),rotation->get_nr());
//				fp_IT<<"theta1="<<theta1<<", theta2="<<theta2<<std::endl;
//				if(fabs(theta1-theta2)<THRES_RAD)
//				{
//					fp_IT<<"theta1=theta2 => CONSTRAINED"<<std::endl;
//					axis=r;
//					angle=0.5*(theta1+theta2);
//					equal=true;
//				}
//				else 
//				{
//					fp_IT<<"theta1!=theta2 => NONE"<<std::endl;
//					equal=false;
//				}
//			}
		}

		else if(!hasDOF && rotation->HasDOF())
		{
			fp_IT<<"constrained & hasDoF"<<std::endl;
			Eigen::Vector3d normal2=rotation->getNormal();
			if(fabs(normal2.dot(axis))<THRES_RAD)
			{
				fp_IT<<"r.dot(nu)=0"<<std::endl;
				double angle2=rotation_angle(axis,rotation->get_nc(),rotation->get_nr());
				fp_IT<<"theta1="<<angle<<", theta2="<<angle2<<std::endl;
				if(fabs(angle-angle2)<THRES_RAD) 
				{
					fp_IT<<"theta1=theta2 => CONSTRAINED"<<std::endl;
					fp_IT<<"updated this rotation_consistency"<<std::endl;
					angle=0.5*(angle+angle2);
					equal=true;
				}
				else 
				{
					fp_IT<<"theta1!=theta2 => NONE"<<std::endl;
					equal=false;
				}
			}
			else 
			{
				fp_IT<<"r.dot(nu)!=0 => NONE"<<std::endl;
				equal=false;
			}
		}

		else if(hasDOF && !rotation->HasDOF())
		{
			fp_IT<<"constrained & hasDoF"<<std::endl;
			Eigen::Vector3d r=rotation->getAxisAngle();
			double theta=r.norm();
			r.normalize();
			Eigen::Vector3d normal=getNormal();
			if(fabs(normal.dot(r))<THRES_RAD)
			{
				fp_IT<<"r.dot(nu)=0"<<std::endl;
				double theta2=rotation_angle(r,0.5*(nci+ncj),0.5*(nri+nrj));
				fp_IT<<"theta1="<<theta<<", theta2="<<theta2<<std::endl;
				if(fabs(theta-theta2)<THRES_RAD)
				{
					fp_IT<<"theta1=theta2 => CONSTRAINED"<<std::endl;
					fp_IT<<"updated this rotation_consistency"<<std::endl;
					axis=r;
					angle=0.5*(theta+theta2);
					hasDOF=false;
					equal=true;
				}
				else 
				{
					fp_IT<<"theta1!=theta2 => NONE"<<std::endl;
					equal=false;
				}
			}
			else 
			{
				fp_IT<<"r.dot(nu)!=0 => NONE"<<std::endl;
				equal=false;
			}
		}

		else if(!hasDOF && !rotation->HasDOF())
		{
			fp_IT<<"both constrained"<<std::endl;
			Eigen::Vector3d axis_angle=rotation->getAxisAngle();
			double angle2=axis_angle.norm();
			Eigen::Vector3d axis2=axis_angle.normalized();
			Eigen::Vector3d delta=axis-axis2;
			fp_IT<<"r1="<<axis.transpose()<<std::endl;
			fp_IT<<"r2="<<axis2.transpose()<<std::endl;
			fp_IT<<"theta1="<<angle<<", theta2="<<angle2<<std::endl;
			if(fabs(angle-angle2)<THRES_RAD && fabs(delta.norm())<THRES_RAD) 
			{
				fp_IT<<"R1=R2 => CONSTRAINED"<<std::endl;
				fp_IT<<"updated this rotation_consistency"<<std::endl;
				axis=0.5*(axis+axis2);
				angle=0.5*(angle+angle2);
				equal=true;
			}
			else 
			{
				fp_IT<<"R1!=R2 => NONE"<<std::endl;
				equal=false;
			}
		}

//		fp_IT.close();
		return equal;
	}

	bool ProblemTranslation::solve()
	{
		ModelType();

//		std::ofstream fp_IT;
//		fp_IT.open("ProblemTranslation.txt",std::ios::app);
		if(dbg) fp_IT<<std::endl<<"-----------------------ProblemTranslation::solve()-----------------------"<<std::endl;

//		THRES_RAD=5.0*M_PI/180.0;
//		fp_IT<<"theshold for angle is "<<THRES_RAD*180.0/M_PI<<" deg"<<std::endl;
//		THRES_DIST=0.05;
//		fp_IT<<"theshold for distance is "<<THRES_DIST<<" m"<<std::endl;

		isConsistent=false;
		Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
		if(dbg) fp_IT<<"R="<<std::endl<<R<<std::endl;

		// plane - n;
		// line - v;
		Eigen::Vector3d nci = node_i->feature_cur->getDirection();
		Eigen::Vector3d nri = node_i->feature_ref->getDirection();
		Eigen::Vector3d ncj = node_j->feature_cur->getDirection();
		Eigen::Vector3d nrj = node_j->feature_ref->getDirection();

		// plane - d=u(0);
		// line - u;
		Eigen::Vector3d uci = node_i->feature_cur->getDistance();
		Eigen::Vector3d uri = node_i->feature_ref->getDistance();
		Eigen::Vector3d ucj = node_j->feature_cur->getDistance();
		Eigen::Vector3d urj = node_j->feature_ref->getDistance();

		if(dbg) 
		{
			fp_IT<<"nri(vri) = "<<nri.transpose()<<std::endl;
			fp_IT<<"nrj(vrj) = "<<nrj.transpose()<<std::endl;
			fp_IT<<"nci(vci) = "<<nci.transpose()<<std::endl;
			fp_IT<<"ncj(vcj) = "<<ncj.transpose()<<std::endl;

			fp_IT<<"dri(uri) = "<<uri.transpose()<<std::endl;
			fp_IT<<"drj(urj) = "<<urj.transpose()<<std::endl;
			fp_IT<<"dci(uci) = "<<uci.transpose()<<std::endl;
			fp_IT<<"dcj(ucj) = "<<ucj.transpose()<<std::endl;
		}

		Eigen::Vector3d translation_local, direction_local;
		SOLUTION_CASE solution_case_local;
		double translation_error=0;

		switch(type)
		{
		case PLANE_PLANE :
		{
			if(dbg) fp_IT<<"Type PLANE_PLANE"<<std::endl;
			double dci=uci(0);
			double dri=uri(0);
			double dcj=ucj(0);
			double drj=urj(0);
			if(included_angle(nci,ncj)<THRES_RAD)
			{
				SOLUTION_CASE case1, case2;
				Eigen::Vector3d trans1, trans2, dir1, dir2;
				double error1, error2;
				case1=plane_plane_parallel(nci,nri,dci,dri,ncj,nrj,dcj,drj,trans1,dir1,error1);
				case2=plane_plane_nonparallel(nci,nri,dci,dri,ncj,nrj,dcj,drj,trans2,dir2,error2);
				if(case1!=NONE)
				{
					if(error1<error2)
					{
						solution_case_local=case1;//LIN2DOF
						translation_local=trans1;
						direction_local=dir1;
						translation_error=error1;
					}
				}
				else 
				{
					solution_case_local=case2;//LIN1DOF or NONE
					translation_local=trans2;
					direction_local=dir2;
					translation_error=error2;
				}
			}
			else 
			{
				solution_case_local=plane_plane_nonparallel(nci,nri,dci,dri,ncj,nrj,dcj,drj,
									translation_local,direction_local,translation_error);
			}
			break;
		}

		case LINE_LINE :
		{
			if(dbg) fp_IT<<"Type LINE_LINE"<<std::endl;
			if(included_angle(nci,ncj)<THRES_RAD)
			{
				SOLUTION_CASE case1, case2;
				Eigen::Vector3d trans1, trans2, dir1;
				double error1, error2;
				ProblemRotation *problem=new ProblemRotation;
				case1=line_line_parallel(nci,nri,uci,uri,ncj,nrj,ucj,urj,trans1,dir1,error1,problem);
				case2=line_line_nonparallel(nci,nri,uci,uri,ncj,nrj,ucj,urj,trans2,error2);
				if(case1!=NONE)
				{
					if(error1<error2)
					{
						solution_case_local=case1;//LIN1DOF
						translation_local=trans1;
						direction_local=dir1;
						translation_error=error1;
						*rotation_consistency=*problem;
					}
				}
				else 
				{
					solution_case_local=case2;//CONSTRAINED or NONE
					translation_local=trans2;
					direction_local.setZero();
					translation_error=error2;
				}
				delete problem;
			}
			else 
			{
				solution_case_local=line_line_nonparallel(nci,nri,uci,uri,ncj,nrj,ucj,urj,translation_local,translation_error);
				direction_local.setZero();
			}
			break;
		}

		case PLANE_LINE :
		{
			if(dbg) fp_IT<<"Type PLANE_LINE"<<std::endl;
			Eigen::Vector3d ur,uc,vr,vc;
			Eigen::Vector3d nr,nc; double dr,dc;
			if(node_i->type==PLANE)
			{
				nr=nri; nc=nci;
				dr=uri(0); dc=uci(0);
				vr=nrj; vc=ncj;
				ur=urj; uc=ucj;
			}
			else 
			{
				vr=nri; vc=nci;
				ur=uri; uc=uci;
				nr=nrj; nc=ncj;
				dr=urj(0); dc=ucj(0);
			}
			if(dbg) fp_IT<<"vc.dot(nc)="<<fabs(vc.dot(nc))<<std::endl;
			if(fabs(vc.dot(nc))<THRES_RAD)
			{
				SOLUTION_CASE case1, case2;
				Eigen::Vector3d trans1, trans2, dir1, dir2;
				double error1, error2;
				case1=plane_line_parallel(nc,nr,dc,dr,vc,vr,uc,ur,trans1,dir1,error1);
				case2=plane_line_intersect(nc,nr,dc,dr,vc,vr,uc,ur,trans2,dir2,error2);
				if(case1==LIN1DOF)
				{
					if(error1<error2)
					{
						solution_case_local=case1;//LIN1DOF
						translation_local=trans1;
						direction_local=dir1;
						translation_error=error1;
					}
				}
				else 
				{
					solution_case_local=case2;//CONSTRAINED
					translation_local=trans2;
					direction_local=dir2;
					translation_error=error2;
				}
			}
			else //(fabs(vc.dot(nc))>=THRES_RAD)
			{
				solution_case_local=plane_line_intersect(nc,nr,dc,dr,vc,vr,uc,ur,translation_local,direction_local,translation_error);
			}
			break;
		}

		default : break;
		}

		solution_case=solution_case_local;
		translation=translation_local;
		direction=direction_local;

		if(solution_case!=NONE) isConsistent=true;
		return isConsistent;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::plane_plane_parallel(
			Eigen::Vector3d nci, Eigen::Vector3d nri, double dci, double dri,
			Eigen::Vector3d ncj, Eigen::Vector3d nrj, double dcj, double drj,
			Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error)
	{
		if(dbg) fp_IT<<"nci=ncj => parallel planes, rank(A)=1"<<std::endl;
		SOLUTION_CASE solution_case;
		if(fabs((dri-dci)-(drj-dcj))<THRES_DIST)
		{
			if(dbg) fp_IT<<"dri-dci=drj-dcj => consistent"<<std::endl
					  <<"solution_case: LIN2DOF"<<std::endl;
			solution_case=LIN2DOF;
			direction_local=0.5*(nci+ncj);
			direction_local.normalize();
			if(dbg) fp_IT<<"direction="<<direction_local.transpose()<<std::endl;
			Eigen::Matrix<double,5,3> A;
			Eigen::Matrix<double,5,1> b;
			A.row(0)=nci.transpose();
			A.row(1)=ncj.transpose();
			A.block<3,3>(2,0)=Transform::skew_sym(direction_local);
			b(0)=dri-dci;
			b(1)=drj-dcj;
			b.block<3,1>(2,0).setZero();
			translation_local=A.fullPivLu().solve(b);
			if(dbg) 
			{
				fp_IT<<"A="<<std::endl<<A<<std::endl;
				fp_IT<<"b="<<b.transpose()<<std::endl;
				fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
			}

			Eigen::Matrix<double,5,1> res=A*translation_local-b;
			translation_error=res.norm()/sqrt(2.0);
			if(dbg) fp_IT<<"translation error="<<translation_error<<std::endl;
		}
		else 
		{
			if(dbg) fp_IT<<"dri-dci!=drj-dcj => inconsistent"<<std::endl;
			solution_case=NONE;
			translation_error=DBL_MAX;
		}
		return solution_case;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::plane_plane_nonparallel(
			Eigen::Vector3d nci, Eigen::Vector3d nri, double dci, double dri,
			Eigen::Vector3d ncj, Eigen::Vector3d nrj, double dcj, double drj,
			Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error)
	{
		if(dbg) fp_IT<<"nci!=ncj => rank(A)=2 => consistent"<<std::endl
				  <<"solution_case: LIN1DOF"<<std::endl;
		SOLUTION_CASE solution_case;
		solution_case=LIN1DOF;
		direction_local=nci.cross(ncj);
		direction_local.normalize();
		if(dbg) fp_IT<<"direction="<<direction_local.transpose()<<std::endl;
		Eigen::Matrix3d A;
		Eigen::Vector3d b;
		A.row(0)=nci.transpose();
		A.row(1)=ncj.transpose();
		A.row(2)=direction_local.transpose();
		b(0)=dri-dci;
		b(1)=drj-dcj;
		b(2)=0;
		translation_local=A.fullPivLu().solve(b);
		if(dbg)
		{
			fp_IT<<"A="<<std::endl<<A<<std::endl;
			fp_IT<<"b="<<b.transpose()<<std::endl;
			fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
		}

		Eigen::Vector3d res=A*translation_local-b;
		translation_error=res.norm()/sqrt(2.0);
		if(dbg) fp_IT<<"translation error="<<translation_error<<std::endl;

		return solution_case;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::line_line_parallel(
			Eigen::Vector3d nci, Eigen::Vector3d nri, Eigen::Vector3d uci, Eigen::Vector3d uri,
			Eigen::Vector3d ncj, Eigen::Vector3d nrj, Eigen::Vector3d ucj, Eigen::Vector3d urj,
			Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error, 
			ProblemRotation* problem)
	{
		if(dbg) fp_IT<<"<vci,vcj>="<<included_angle(nci,ncj)*180/M_PI<<std::endl
				  <<"vci=vcj => parallel lines"<<std::endl;
		Eigen::Vector3d ur=uri-urj;
		Eigen::Vector3d uc=uci-ucj;
		double dr=ur.norm();
		double dc=uc.norm();
		if(dbg) fp_IT<<"|uri-urj|="<<dr<<std::endl
				  <<"|uci-ucj|="<<dc<<std::endl;
		SOLUTION_CASE solution_case;
		if(fabs(dr-dc)<THRES_DIST)
		{
			if(dbg) fp_IT<<"|uri-urj|=|uci-ucj|"<<std::endl;
			ur.normalize();
			uc.normalize();
			Eigen::Vector3d vr=0.5*(nri+nrj);
			Eigen::Vector3d vc=0.5*(nci+ncj);
			if(dbg)
			{
				fp_IT<<"ur="<<ur.transpose()<<std::endl;
				fp_IT<<"vr="<<vr.transpose()<<std::endl;
				fp_IT<<"uc="<<uc.transpose()<<std::endl;
				fp_IT<<"vc="<<vc.transpose()<<std::endl;
				fp_IT<<"************************************"<<std::endl;
			}
			problem->input(ur,vr,uc,vc);
			bool isConsistent=problem->solve();
			Eigen::Matrix3d R=problem->getRotationMatrix();
			if(dbg) fp_IT<<"problem->R="<<std::endl<<R<<std::endl
					  <<"************************************"<<std::endl;
			if(isConsistent)
			{
				if(dbg) fp_IT<<"exists R s.t. R*uri-uci=R*urj-rcj, R*vri=vci => consistent"<<std::endl
						  <<"solution_case: LIN1DOF"<<std::endl;
				solution_case=LIN1DOF;
				direction_local=0.5*(nci+ncj);
				direction_local.normalize();
				if(dbg) fp_IT<<"direction="<<direction_local.transpose()<<std::endl;
				Eigen::Matrix<double,7,3> A;
				Eigen::Matrix<double,7,1> b;
				A.block<3,3>(0,0)=Transform::skew_sym(nci);
				A.block<3,3>(3,0)=Transform::skew_sym(ncj);
				A.row(6)=direction_local.transpose();
				b.block<3,1>(0,0)=R*uri-uci;
				b.block<3,1>(3,0)=R*urj-ucj;
				b(6)=0;
				translation_local=A.fullPivLu().solve(b);
				if(dbg)
				{
					fp_IT<<"A="<<std::endl<<A<<std::endl;
					fp_IT<<"b="<<b.transpose()<<std::endl;
					fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
				}

				Eigen::Matrix<double,7,1> res=A*translation_local-b;
				translation_error=res.norm()/sqrt(2.0);
				if(dbg) fp_IT<<"translation error="<<translation_error<<std::endl;
			}
			else 
			{
				if(dbg) fp_IT<<"cannot find R s.t. R*uri-uci=R*urj-rcj, R*vri=vci => inconsistent"<<std::endl;
				solution_case=NONE;
				translation_error=DBL_MAX;
			}
		}
		else
		{
			solution_case=NONE;
			if(dbg) fp_IT<<"|uri-urj|!=|uci-ucj| => inconsistent"<<std::endl;
			translation_error=DBL_MAX;
		}
		return solution_case;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::line_line_nonparallel(
			Eigen::Vector3d nci, Eigen::Vector3d nri, Eigen::Vector3d uci, Eigen::Vector3d uri,
			Eigen::Vector3d ncj, Eigen::Vector3d nrj, Eigen::Vector3d ucj, Eigen::Vector3d urj,
			Eigen::Vector3d &translation_local, double &translation_error)
	{
		if(dbg) fp_IT<<"vci!=vcj => non-parallel lines"<<std::endl;
		SOLUTION_CASE solution_case;
		Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
		Eigen::Vector3d ui=R*uri-uci;
		Eigen::Vector3d uj=R*urj-ucj;
		double lr_lc=ui.dot(ncj)+uj.dot(nci);
		if(dbg) fp_IT<<"lr-lc="<<lr_lc<<std::endl;
		if(fabs(lr_lc)<THRES_DIST)
		{
			if(dbg) fp_IT<<"lr-lc=0 => consistent"<<std::endl
					  <<"solution_case: CONSTRAINED"<<std::endl;
			solution_case=CONSTRAINED;
			Eigen::Matrix<double,6,3> A;
			Eigen::Matrix<double,6,1> b;
			A.block<3,3>(0,0)=Transform::skew_sym(R*nri);
			A.block<3,3>(3,0)=Transform::skew_sym(R*nrj);
			b.block<3,1>(0,0)=ui;
			b.block<3,1>(3,0)=uj;
			translation_local=A.fullPivLu().solve(b);
			if(dbg) 
			{
				fp_IT<<"A="<<std::endl<<A<<std::endl;
				fp_IT<<"b="<<b.transpose()<<std::endl;
				fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
			}

			Eigen::Matrix<double,6,1> res=A*translation_local-b;
			translation_error=res.norm()/sqrt(2.0);
			if(dbg) fp_IT<<"translation error="<<translation_error<<std::endl;
		}
		else 
		{
			if(dbg) fp_IT<<"lr-lc!=0 => inconsistent"<<std::endl;
			solution_case=NONE;
			translation_error=DBL_MAX;
		}
		return solution_case;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::plane_line_parallel(Eigen::Vector3d nc, Eigen::Vector3d nr, double dc, double dr, 
			Eigen::Vector3d vc, Eigen::Vector3d vr, Eigen::Vector3d uc, Eigen::Vector3d ur, 
			Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error)
	{
		if(dbg) fp_IT<<"vc.dot(nc)=0 => plane and line are parallel"<<std::endl;
		Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
		if(dbg) fp_IT<<"lr-lc="<<fabs(nc.dot(vc.cross(R*ur-uc))+(dr-dc))<<std::endl;
		SOLUTION_CASE solution_case_local;
		if(fabs(nc.dot(vc.cross(R*ur-uc))+(dr-dc))<THRES_DIST)
		{
			if(dbg) fp_IT<<"lr-lc=0 => consistent"<<std::endl
					  <<"solution_case: LIN1DOF"<<std::endl;
//				isConsistent=true;
			solution_case_local=LIN1DOF;
			direction_local=vc;
			direction_local.normalize();
			if(dbg) fp_IT<<"direction="<<direction_local.transpose()<<std::endl;
			Eigen::Matrix<double,5,3> A;
			Eigen::Matrix<double,5,1> b;
			A.row(0)=nc.transpose();
			A.block<3,3>(1,0)=Transform::skew_sym(vc);
			A.row(4)=direction_local.transpose();
			b(0)=dr-dc;
			b.block<3,1>(1,0)=R*ur-uc;
			b(4)=0;
			translation_local=A.fullPivLu().solve(b);
			if(dbg) 
			{
				fp_IT<<"A="<<std::endl<<A<<std::endl;
				fp_IT<<"b="<<b.transpose()<<std::endl;
				fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
			}

			Eigen::Matrix<double,5,1> res=A*translation_local-b;
			translation_error=res.norm()/sqrt(2.0);
//			Eigen::Vector3d res_line=uc-R*ur-translation_local.cross(R*vr);
//			fp_IT<<"line residual=uc-R*ur-t.cross(R*ur)="<<res_line.transpose()<<std::endl;
//			double res_plane=dc-dr+translation_local.dot(R*nr);
//			fp_IT<<"plane residual=dc-dr+t.dot(R*nr)="<<res_plane<<std::endl;
//			translation_error=0.5*(res_line.norm()+res_plane);
			if(dbg) fp_IT<<"translation error="<<translation_error<<std::endl;
		}
		else 
		{
			if(dbg) fp_IT<<"lr-lc!=0 => inconsistent"<<std::endl;
//				isConsistent=false;
			solution_case_local=NONE;
			translation_error=DBL_MAX;
		}
		return solution_case_local;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::plane_line_intersect(Eigen::Vector3d nc, Eigen::Vector3d nr, double dc, double dr, 
			Eigen::Vector3d vc, Eigen::Vector3d vr, Eigen::Vector3d uc, Eigen::Vector3d ur, 
			Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error)
	{
		if(dbg) fp_IT<<"vc.dot(nc)!=0 => plane and line intersect"<<std::endl
				  <<" => consistent"<<std::endl
				  <<"vc.cross(nc)="<<vc.cross(nc).norm()<<std::endl;
		Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
		Eigen::Vector3d t;
		SOLUTION_CASE solution_case_local;
//			if(vc.cross(nc).norm()<THRES_RAD)
		if(rotation_consistency->HasDOF())
		{
			if(dbg) fp_IT<<"solution_case: ROT1DOF"<<std::endl;
			solution_case_local=ROT1DOF;
			translation_local=(dr-dc)*nc+vc.cross(uc);
			direction_local=ur.cross(vr);
			t=translation_local+R*direction_local;
			if(dbg)
			{
				fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
				fp_IT<<"direction="<<direction_local.transpose()<<std::endl;
				fp_IT<<"translation+R*direction="<<t.transpose()<<std::endl;
			}
		}
		else 
		{
			if(dbg) fp_IT<<"solution_case: CONSTRAINED"<<std::endl;
			solution_case_local=CONSTRAINED;
			Eigen::Matrix<double,4,3> A;
			Eigen::Matrix<double,4,1> b;
			A.block<1,3>(0,0)=nc.transpose();
			A.block<3,3>(1,0)=Transform::skew_sym(vc);
			b(0)=dr-dc;
			b.block<3,1>(1,0)=R*ur-uc;
			translation_local=A.fullPivLu().solve(b);
			if(dbg)
			{
				fp_IT<<"A="<<std::endl<<A<<std::endl;
				fp_IT<<"b="<<b.transpose()<<std::endl;
				fp_IT<<"translation="<<translation_local.transpose()<<std::endl;
			}
			t=translation_local;
		}
		Eigen::Vector3d res_line=uc-R*ur-t.cross(R*vr);
		double res_plane=dc-dr+t.dot(R*nr);
		translation_error=0.5*(res_line.norm()+res_plane);
		if(dbg)
		{
			fp_IT<<"line residual=uc-R*ur-t.cross(R*ur)="<<res_line.transpose()<<std::endl;
			fp_IT<<"plane residual=dc-dr+t.dot(R*nr)="<<res_plane<<std::endl;
			fp_IT<<"translation error="<<translation_error<<std::endl;
		}
		return solution_case_local;
	}

	bool ProblemTranslation::fuseTranslation(const ProblemTranslation *trans) 
	{
//		std::ofstream fp_IT;
//		fp_IT.open("ProblemTranslation.txt",std::ios::app);
		fp_IT<<std::endl<<"-----------------------ProblemTranslation::fuseTranslation()-----------------------"<<std::endl;

		fp_IT<<"isConsistent: "<<isConsistent<<", "<<trans->IsConsistent()<<std::endl;
		if(!isConsistent || !trans->IsConsistent()) 
		{
//			fp_IT.close();
			return false;
		}

		constrainRotDOF();
		fp_IT<<"constrainRotDOF: translation="<<translation.transpose()<<std::endl;
		fp_IT<<"solution_case:"<<std::endl;
		fp_IT<<solution_case<<", "<<trans->solution_case<<std::endl;

		bool equal=false;
		SOLUTION_CASE tmp_case=NONE;
		switch(solution_case)
		{
		case CONSTRAINED :
			switch(trans->solution_case)
			{
			case CONSTRAINED :
				tmp_case=fuse_Constrained_Constrained(translation,trans->getTranslation(),translation); break;
			case LIN1DOF :
				tmp_case=fuse_Constrained_Lin1DOF(translation,trans->getTranslation(),trans->getDirection(),translation); break;
			case LIN2DOF :
				tmp_case=fuse_Constrained_Lin2DOF(translation,trans->getTranslation(),trans->getDirection(),translation); break;
			case ROT1DOF :
				tmp_case=CONSTRAINED;
				break;
			default : break;
			}
			break;

		case LIN1DOF :
			switch(trans->solution_case)
			{
			case CONSTRAINED :
				tmp_case=fuse_Constrained_Lin1DOF(trans->getTranslation(),translation,direction,translation); break;
			case LIN1DOF :
				tmp_case=fuse_Lin1DOF_Lin1DOF(translation,direction,trans->getTranslation(),trans->getDirection(),translation,direction); break;
			case LIN2DOF :
				tmp_case=fuse_Lin1DOF_Lin2DOF(translation,direction,trans->getTranslation(),trans->getDirection(),translation,direction); break;
			case ROT1DOF :
				tmp_case=CONSTRAINED;
				translation=trans->getTranslation()+rotation_consistency->getRotationMatrix()*trans->getDirection();
				break;
			default : break;
			}
			break;

		case LIN2DOF :
			switch(trans->solution_case)
			{
			case CONSTRAINED :
				tmp_case=fuse_Constrained_Lin2DOF(trans->getTranslation(),translation,direction,translation); break;
			case LIN1DOF :
				tmp_case=fuse_Lin1DOF_Lin2DOF(trans->getTranslation(),trans->getDirection(),translation,direction,translation,direction); break;
			case LIN2DOF :
				tmp_case=fuse_Lin2DOF_Lin2DOF(trans->getTranslation(),trans->getDirection(),translation,direction,translation,direction); break;
			case ROT1DOF :
				tmp_case=fuse_Rot1DOF_Lin2DOF(trans->getTranslation(),trans->getDirection(),translation,direction,translation,direction); break;
			default : break;
			}
			break;

		case ROT1DOF :
			switch(trans->solution_case)
			{
			case LIN2DOF :
				tmp_case=fuse_Rot1DOF_Lin2DOF(translation,direction,trans->getTranslation(),trans->getDirection(),translation,direction); break;
			case ROT1DOF :
				tmp_case=fuse_Rot1DOF_Rot1DOF(translation,direction,trans->getTranslation(),trans->getDirection(),translation,direction); break;
			default : break;
			}
			break;

		default : break;
		}

		if(tmp_case==NONE) equal=false;
		else 
		{
			equal=true;
			solution_case=tmp_case;
		}

//		fp_IT.close();
		return equal;
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Constrained_Constrained
			(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d &t)
	{
		fp_IT<<std::endl<<"both CONSTRAINED"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2 => CONSTRAINED"<<std::endl;
			t=0.5*(t1+t2);
			fp_IT<<"t="<<t.transpose()<<std::endl;
			return CONSTRAINED;
		}
		else 
		{
			fp_IT<<"t1!=t2 => NONE"<<std::endl;
			return NONE;
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Constrained_Lin1DOF
											(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t)
	{
		fp_IT<<std::endl<<"CONSTRAINED & LIN1DOF"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"t1-t2="<<delta.transpose()<<std::endl;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2 => CONSTRAINED"<<std::endl;
//			t=0.5*(t1+t2);
			t=t1;
			fp_IT<<"t="<<t.transpose()<<std::endl;
			return CONSTRAINED;
		}
		else 
		{
			fp_IT<<"t1!=t2"<<std::endl;
			delta.normalize();
			v2.normalize();
			fp_IT<<"|(t1-t2)xv2|="<<delta.cross(v2).norm()<<std::endl;
			if(delta.cross(v2).norm()<THRES_RAD*2)
			{
				fp_IT<<"(t1-t2)xv2=0 => CONSTRAINED"<<std::endl;
				t=t1;
				fp_IT<<"t=t1="<<t.transpose()<<std::endl;
				return CONSTRAINED;
			}
			else 
			{
				fp_IT<<"(t1-t2)xv2!=0 => NONE"<<std::endl;
				return NONE;
			}
			
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Constrained_Lin2DOF
											(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t)
	{
		fp_IT<<std::endl<<"CONSTRAINED & LIN2DOF"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"t1-t2="<<delta.transpose()<<std::endl;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2 => CONSTRAINED"<<std::endl;
//			t=0.5*(t1+t2);
			t=t1;
			fp_IT<<"t="<<t.transpose()<<std::endl;
			return CONSTRAINED;
		}
		else 
		{
			fp_IT<<"t1!=t2"<<std::endl;
			delta.normalize();
			v2.normalize();
			if(fabs(delta.dot(v2))<THRES_RAD*2)
			{
				fp_IT<<"(t1-t2).dot(v2)=0 => CONSTRAINED"<<std::endl;
				t=t1;
				fp_IT<<"t="<<t.transpose()<<std::endl;
				return CONSTRAINED;
			}
			else 
			{
				fp_IT<<"(t1-t2).dot(v2)!=0 => NONE"<<std::endl;
				return NONE;
			}
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Lin1DOF_Lin1DOF
										(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
										Eigen::Vector3d &t, Eigen::Vector3d &v)
	{
		fp_IT<<std::endl<<"both LIN1DOF"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v1="<<v1.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"t1-t2="<<delta.transpose()<<std::endl;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		v1.normalize(); v2.normalize();
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2"<<std::endl;
			if(v1.cross(v2).norm()<THRES_RAD*2)
			{
				fp_IT<<"v1xv2=0 => LIN1DOF"<<std::endl;
				t=t1; v=v1;
				fp_IT<<"t="<<t.transpose()<<std::endl;
				fp_IT<<"v="<<v.transpose()<<std::endl;
				return LIN1DOF;
			}
			else 
			{
				fp_IT<<"v1xv2!=0 => CONSTRAINED"<<std::endl;
				t=0.5*(t1+t2);
				fp_IT<<"t="<<t.transpose()<<std::endl;
				return CONSTRAINED;
			}
		}
		else 
		{
			delta.normalize();
			fp_IT<<"t1!=t2"<<std::endl;
			if(v1.cross(v2).norm()<THRES_RAD*2)
			{
				fp_IT<<"v1xv2=0"<<std::endl;
				if(delta.cross(v1).norm()<THRES_RAD*2)
				{
					fp_IT<<"(t1-t2)xv1=0 => LIN1DOF"<<std::endl;
					t=t1; v=v1;
					fp_IT<<"t="<<t.transpose()<<std::endl;
					fp_IT<<"v="<<v.transpose()<<std::endl;
					return LIN1DOF;
				}
				else 
				{
					fp_IT<<"(t1-t2)xv1!=0 => NONE"<<std::endl;
					return NONE;
				}
			}
			else 
			{
				fp_IT<<"v1xv2!=0"<<std::endl;
				if(fabs(delta.dot(v1.cross(v2)))<THRES_RAD*2)
				{
					fp_IT<<"(t1-t1).dot(v1xv2)=0 => CONSTRAINED"<<std::endl;
					Eigen::Matrix<double,3,2> A;
					Eigen::Vector3d b=t1-t2;
					A.col(0)=-v1;
					A.col(1)=v2;
					Eigen::Vector2d eta=A.fullPivLu().solve(b);
					t=t1+eta(0)*v1;
					fp_IT<<"t="<<t.transpose()<<std::endl;
					return CONSTRAINED;
				}
				else 
				{
					fp_IT<<"(t1-t1).dot(v1xv2)!=0 => NONE"<<std::endl;
					return NONE;
				}
			}
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Lin1DOF_Lin2DOF
										(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
										Eigen::Vector3d &t, Eigen::Vector3d &v)
	{
		fp_IT<<std::endl<<"LIN1DOF & LIN2DOF"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v1="<<v1.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"t1-t2="<<delta.transpose()<<std::endl;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		v1.normalize(); v2.normalize();
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2"<<std::endl;
			if(fabs(v1.dot(v2))<THRES_RAD*2)
			{
				fp_IT<<"v1.dot(v2)=0 => LIN1DOF"<<std::endl;
				t=t1; v=v1;
				fp_IT<<"t="<<t.transpose()<<std::endl;
				fp_IT<<"v="<<v.transpose()<<std::endl;
				return LIN1DOF;
			}
			else 
			{
				fp_IT<<"v1.dot(v2)!=0 => CONSTRAINED"<<std::endl;
				t=0.5*(t1+t2);
				return CONSTRAINED;
			}
		}
		else 
		{
			delta.normalize();
			fp_IT<<"t1!=t2"<<std::endl;
			if(fabs(v1.dot(v2))<THRES_RAD*2)
			{
				fp_IT<<"v1.dot(v2)=0"<<std::endl;
				if(fabs(delta.dot(v2))<THRES_RAD*2)
				{
					fp_IT<<"(t1-t2).dot(v2)=0 => LIN1DOF"<<std::endl;
					t=t1; v=v1;
					fp_IT<<"t="<<t.transpose()<<std::endl;
					fp_IT<<"v="<<v.transpose()<<std::endl;
					return LIN1DOF;
				}
				else 
				{
					fp_IT<<"(t1-t2).dot(v2)!=0 => NONE"<<std::endl;
					return NONE;
				}
			}
			else 
			{
				fp_IT<<"v1.dot(v2)!=0 => CONSTRAINED"<<std::endl;
				Eigen::Matrix<double,3,4> A;
				Eigen::Vector3d b=t1-t2;
				A.col(0)=-v1;
				A.block<3,3>(0,1)=Transform::skew_sym(v2);
				Eigen::Vector4d eta=A.fullPivLu().solve(b);
				t=t1+eta(0)*v1;
				fp_IT<<"t="<<t.transpose()<<std::endl;
				return CONSTRAINED;
			}
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Lin2DOF_Lin2DOF
										(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
										Eigen::Vector3d &t, Eigen::Vector3d &v)
	{
		fp_IT<<std::endl<<"LIN2DOF & LIN2DOF"<<std::endl;
		fp_IT<<"t1="<<t1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v1="<<v1.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta=t1-t2;
		fp_IT<<"t1-t2="<<delta.transpose()<<std::endl;
		fp_IT<<"|t1-t2|="<<delta.norm()<<std::endl;
		v1.normalize(); v2.normalize();
		if(delta.norm()<THRES_DIST)
		{
			fp_IT<<"t1=t2"<<std::endl;
			if(v1.cross(v2).norm()<THRES_RAD*2)
			{
				fp_IT<<"v1xv2=0 => LIN2DOF"<<std::endl;
				t=t1; v=v1;
				fp_IT<<"t="<<t.transpose()<<std::endl;
				fp_IT<<"v="<<v.transpose()<<std::endl;
				return LIN2DOF;
			}
			else 
			{
				fp_IT<<"v1xv2!=0 => LIN1DOF"<<std::endl;
				t=0.5*(t1+t2); v=v1.cross(v2);
				fp_IT<<"t="<<t.transpose()<<std::endl;
				fp_IT<<"v="<<v.transpose()<<std::endl;
				return LIN1DOF;
			}
		}
		else 
		{
			delta.normalize();
			fp_IT<<"t1!=t2"<<std::endl;
			if(v1.cross(v2).norm()<THRES_RAD*2)
			{
				fp_IT<<"v1xv2=0"<<std::endl;
				if(fabs(delta.dot(v1))<THRES_RAD*2)
				{
					fp_IT<<"(t1-t2).dot(v1)=0 => LIN2DOF"<<std::endl;
					t=t1; v=v1;
					fp_IT<<"t="<<t.transpose()<<std::endl;
					fp_IT<<"v="<<v.transpose()<<std::endl;
					return LIN2DOF;
				}
				else 
				{
					fp_IT<<"(t1-t2).dot(v1)!=0 => NONE"<<std::endl;
					return NONE;
				}
			}
			else 
			{
				fp_IT<<"v1xv2!=0 => LIN1DOF"<<std::endl;
				Eigen::Matrix<double,3,6> A;
				Eigen::Vector3d b=t1-t2;
				A.block<3,3>(0,0)=-Transform::skew_sym(v1);
				A.block<3,3>(0,3)=Transform::skew_sym(v2);
				Eigen::Matrix<double,6,1> eta=A.fullPivLu().solve(b);
				t=t1+Transform::skew_sym(v1)*eta.block<3,1>(0,0);
				v=v1.cross(v2);
				fp_IT<<"t="<<t.transpose()<<std::endl;
				fp_IT<<"v="<<v.transpose()<<std::endl;
				return LIN1DOF;
			}
		}
	}


	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Rot1DOF_Lin2DOF
										(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
										Eigen::Vector3d &t, Eigen::Vector3d &v)
	{
		fp_IT<<std::endl<<"ROT1DOF & LIN2DOF"<<std::endl;
		fp_IT<<"t1 [(dr-dc)*nc+vcxuc] ="<<t1.transpose()<<std::endl;
		fp_IT<<"v1 [urxvr] ="<<v1.transpose()<<std::endl;
		fp_IT<<"t2="<<t2.transpose()<<std::endl;
		fp_IT<<"v2="<<v2.transpose()<<std::endl;
		if(fabs(t2.dot(v2)-t1.dot(v2))<THRES_DIST)
		{
			fp_IT<<"t2.dot(v)=t1.dot(v) => ROT1DOF"<<std::endl;
			t=t1; v=v1;
			return ROT1DOF;
		}
		else 
		{
			fp_IT<<"t2.dot(v)!=t1.dot(v) => NONE"<<std::endl;
			return NONE;
		}
	}

	ProblemTranslation::SOLUTION_CASE ProblemTranslation::fuse_Rot1DOF_Rot1DOF
										(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
										Eigen::Vector3d &t, Eigen::Vector3d &v)
	{
		fp_IT<<std::endl<<"ROT1DOF & ROT1DOF"<<std::endl;
		fp_IT<<"t1 [(dr-dc)*nc+vcxuc] ="<<t1.transpose()<<std::endl;
		fp_IT<<"v1 [urxvr] ="<<v1.transpose()<<std::endl;
		fp_IT<<"t2 [(dr-dc)*nc+vcxuc] ="<<t2.transpose()<<std::endl;
		fp_IT<<"v2 [urxvr] ="<<v2.transpose()<<std::endl;
		Eigen::Vector3d delta_v=v1-v2;
		Eigen::Vector3d delta_t=t2-t1;
		if(delta_v.norm()<THRES_DIST && delta_t.norm()<THRES_DIST)
		{
			t=0.5*(t1+t2);
			v=0.5*(v1+v2);
			return ROT1DOF;
		}
		else if(fabs(delta_v.norm()-delta_t.norm())<THRES_DIST)
		{
			fp_IT<<"|v1-v2|=|t2-t1|"<<std::endl;
			delta_v.normalize(); delta_t.normalize();
			Eigen::Vector3d vr=rotation_consistency->get_nr();
			Eigen::Vector3d vc=rotation_consistency->get_nc();
			rotation_consistency->input(vr,delta_v,vc,delta_t);
			if(rotation_consistency->solve())
			{
				fp_IT<<"rotation consistent => CONSTRAINED"<<std::endl;
				Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
				t=0.5*(t1+t2+R*(v1+v2));
				return CONSTRAINED;
			}
			else 
			{
				fp_IT<<"rotation inconsistent => NONE"<<std::endl;
				return NONE;
			}
		}
		else 
		{
			fp_IT<<"|v1-v2|!=|t2-t1| => NONE"<<std::endl;
			return NONE;
		}
	}

	bool ModelConsistency::computeTransform(Node_InterpTree* node)
	{
		if(dbg) fp_IT<<std::endl<<"-----------------------ModelConsistency::computeTransform()-----------------------"<<std::endl;
//		THRES_RAD=10.0*M_PI/180.0; THRES_DIST=0.1;

		Eigen::Vector3d nc,nr,res;
		std::vector<Eigen::Vector3d> nci,nri,uci,uri;
//		nci.resize(nodes.size()+1); nri.resize(nodes.size()+1);
//		uci.resize(nodes.size()+1); uri.resize(nodes.size()+1);

		int count=0;
		for(int i=0;i<nodes.size();i++)
		{
			if(nodes[i]->null) continue;
			nci.push_back(nodes[i]->feature_cur->getDirection());
			nri.push_back(nodes[i]->feature_ref->getDirection());
			uci.push_back(nodes[i]->feature_cur->getDistance());
			uri.push_back(nodes[i]->feature_ref->getDistance());
			count++;
		}
		if(count==0) return true;
		nci.push_back(node->feature_cur->getDirection());
		nri.push_back(node->feature_ref->getDirection());
		uci.push_back(node->feature_cur->getDistance());
		uri.push_back(node->feature_ref->getDistance());

		nc.setZero(); nr.setZero();
		for(int i=0;i<nci.size();i++) { nc+=nci[i]; nr+=nri[i]; }
		nc/=nci.size(); nr/=nri.size();
		double dc=0, dr=0;
		for(int i=0;i<nci.size();i++)
		{ res=nci[i]-nc; dc+=res.norm(); 
		  res=nri[i]-nr; dr+=res.norm(); }
		dc/=nci.size(); dr/=nri.size();

		if(dc<THRES_RAD && dr<THRES_RAD)
		{
			if(dbg) fp_IT<<"there are infinitely many solutions for the rotation."<<std::endl;
		}
		else 
		{
			Eigen::Matrix3d H; H.setZero();
			for(int i=0;i<nci.size();i++) { H+=nri[i]*nci[i].transpose(); }
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d H_svd_U=svd.matrixU();
			Eigen::Matrix3d H_svd_V=svd.matrixV();
			Eigen::Vector3d H_singularValues=svd.singularValues();
			if(dbg) fp_IT<<"H_singularValues: "<<H_singularValues.transpose()<<std::endl;
			Tcr.R=H_svd_V*H_svd_U.transpose();
			if(dbg) fp_IT<<"R.det()="<<Tcr.R.determinant()<<std::endl;
			if(Tcr.R.determinant()<0) 
			{
//				std::cerr<<"aligning failure !"<<std::endl;
				H_svd_V.col(2)=-H_svd_V.col(2);
				Tcr.R=H_svd_V*H_svd_U.transpose();
				if(dbg) fp_IT<<"R.det()="<<Tcr.R.determinant()<<std::endl;
			}

			rotation_error=0;
			for(int i=0;i<nci.size();i++)
			{
				res=nci[i]-Tcr.R*nri[i];
				if(dbg) fp_IT<<i<<" - "<<res.norm()<<std::endl;
				rotation_error+=res.norm()*res.norm();
			}
			rotation_error/=nci.size();
			rotation_error=sqrt(rotation_error);
			if(dbg) fp_IT<<"average roation error (Sum{|nc-R*nr|^2}/N) is "<<rotation_error<<std::endl;
			if(rotation_error>THRES_RAD) 
			{
				if(dbg) fp_IT<<"aligning error for rotation is too large => no consistent rotation found"<<std::endl;
				return false;
			}
			else 
			{
				double angle=acos((Tcr.R.trace()-1.0)/2.0);
				if(dbg) fp_IT<<"rotation angle = "<<angle*180.0/M_PI<<std::endl;
				Eigen::Vector3d axis;
				axis(0)=Tcr.R(2,1)-Tcr.R(1,2);
				axis(1)=Tcr.R(0,2)-Tcr.R(2,0);
				axis(2)=Tcr.R(1,0)-Tcr.R(0,1);
				axis/=2.0*sin(angle);
				if(dbg) fp_IT<<"rotation axis = "<<axis.transpose()<<std::endl;
			}
		}

		const int N=uci.size();
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N*3,3);
		Eigen::VectorXd b = Eigen::VectorXd::Zero(N*3);
		Eigen::VectorXd res_trans = Eigen::VectorXd::Zero(N*3);
		for(int i=0;i<uci.size();i++)
		{
			if(fabs(uci[i](1))<1e-10 && fabs(uci[i](2))<1e-10)
			{
				A.block<1,3>(3*i,0)=nci[i].transpose();
				b(3*i)=uri[i](0)-uci[i](0);
			}
			else 
			{
				A.block<3,3>(3*i,0)=ulysses::Transform::skew_sym(nci[i]);
				b.block<3,1>(3*i,0)=Tcr.R*uri[i]-uci[i];
			}
		}
		Tcr.t=A.fullPivLu().solve(b);
		if(dbg) fp_IT<<"translation = "<<Tcr.t.transpose()<<std::endl;

		res_trans=A*Tcr.t-b;
		translation_error=sqrt(res_trans.norm()*res_trans.norm()/uci.size());
		if(dbg)
		{
			fp_IT<<"A="<<std::endl<<A<<std::endl;
			fp_IT<<"b="<<b.transpose()<<std::endl;
			fp_IT<<"translation residual= "<<std::endl<<res_trans<<std::endl;
			fp_IT<<"average translation error is "<<translation_error<<std::endl;
		}
		if(translation_error>THRES_DIST) 
		{
			if(dbg) fp_IT<<"aligning error for translation is too large => no consistent translation found"<<std::endl;
			return false;
		}
		return true;
	}

//	bool updateModel(const ModelConsistencyBinary* model)
//	{
//		empty=false;
//		if(!rotation_consistency->fuse(model->rotationConsistency())) return false;
//		// here rotation_consistency has been updated;
//		if(rotation_consistency->HasDOF())
//		{}
//	}

	bool InterpTree::construct(const std::vector<Feature*> &features_cur, const std::vector<Feature*> &features_ref)
	{
//		axis_angle=scan->Rotation_PCA.col(0);

		if(debug) 
		{
			fp_IT.open("interp_tree.txt",std::ios::out);
			fp_IT<<"THRES_RAD="<<THRES_RAD<<std::endl
				 <<"THRES_DIST="<<THRES_DIST<<std::endl;
		}
//		if(debug) fp_IT<<"**************************************************************"<<std::endl;

		// destruct the existing interpretation tree
//		release();
//		fp_IT<<"released"<<std::endl;


//		std::random_device rd;
//		std::mt19937 g(rd());
//		std::shuffle(features_cur.begin(),features_cur.end(),g);
//		std::shuffle(features_ref.begin(),features_ref.end(),g);

//		for(int i=0;i<features_cur.size();i++)
//		{
//			fp_IT<<features_cur[i]<<std::endl;
//		}
//		for(int i=0;i<features_ref.size();i++)
//		{
//			fp_IT<<features_ref[i]<<std::endl;
//		}
//		{ features_cur[i]=scan->features[i]; }
//		for(std::list<Line*>::iterator it=scan->lines_occluding.begin();it!=scan->lines_occluding.end();it++)
//		{ features_cur[idx++]=new Feature(*it); }
		// fill features_ref
//		idx=0;
//		for(int i=0;i<scan->scan_ref->features.size();i++)
//		{ features_ref[i]=scan->scan_ref->features[i]; }
//		for(std::list<Line*>::iterator it=scan->scan_ref->lines_occluding.begin();it!=scan->scan_ref->lines_occluding.end();it++)
//		{ features_ref[idx++]=new Feature(*it); }

		// root of the IT;
		root=new Node_InterpTree;
		// number of levels of the IT;
		level_nodes.resize(features_cur.size());
		level_max_quality.resize(features_cur.size(),0);

		// construct the i-th level of the IT;
//		cout<<"size "<<features_cur.size()<<", "<<features_ref.size()<<endl;
		for(int i=0;i<features_cur.size();i++)
		{
			if(debug) fp_IT<<std::endl<<"filling level "<<i<<"========================================================="<<std::endl;
			// filling level[0] - insert nodes to the root node in level[-1];
			// insert the feature pairs - <cur-0, ref-i>;
			if(i==0)
			{
				for(int j=0;j<features_ref.size();j++)
				{
//					fp_IT<<features_cur[0]->Type()<<", "<<features_ref[j]->Type()<<std::endl;
					if(features_cur[0]->Type()!=features_ref[j]->Type()) continue;
//					fp_IT<<features_cur[0]<<std::endl;
					Node_InterpTree *node=new Node_InterpTree(features_cur[0],features_ref[j]);
//					fp_IT<<features_ref[j]<<std::endl;
					insertNode(root,node);
//					fp_IT<<node<<std::endl;
					node->initConsistentModel();
					if(debug) fp_IT<<"add "<<node<<" to level 0"<<std::endl;
				}
				Node_InterpTree *null_node=new Node_InterpTree(features_cur[0]); // null_node->null=true;
				insertNode(root,null_node);
				if(debug) fp_IT<<"add "<<null_node<<" to level 0"<<std::endl;
//				level_max_quality[0]=1;
				continue;
			}

//			if(debug) fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
//			if(debug) fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl
//						<<"nodes in level "<<i-1<<std::endl;
//			if(debug) 
//			for(std::list<Node_InterpTree*>::iterator it=level_nodes[i-1].begin();it!=level_nodes[i-1].end();it++)
//			{
//				printPath(*it);
//			}
//			if(debug) fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
//			if(debug) fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;

			int count=0;
//			fp_IT<<"size: "<<level_nodes[i-1].size()*level_nodes[i-1].size()<<std::endl;
			for(std::list<Node_InterpTree*>::iterator it=level_nodes[i-1].begin();it!=level_nodes[i-1].end();it++)
			{
				if((*it)->quality<level_max_quality[i-1]/2)
				{
					delete *it;
					it=level_nodes[i-1].erase(it); it--;
				}
			}

			if(level_nodes[i-1].size()>max_size) break;

			timeval start, end;
			double timeused;
			gettimeofday(&start,NULL);
			for(std::list<Node_InterpTree*>::iterator it1=level_nodes[i-1].begin();it1!=level_nodes[i-1].end();it1++)
			for(std::list<Node_InterpTree*>::iterator it2=level_nodes[i-1].begin();it2!=level_nodes[i-1].end();it2++)
			{
//				cout<<"compare "<<*it1<<" and "<<*it2<<endl;
				bool comp=comparePaths(it1,it2);
//				if(comp && debug) 
//				{
//					fp_IT<<std::endl<<"deleted a node ----- "<<std::endl;
//					printPath(*it1,fp_IT);
//					printPath(*it2,fp_IT);
//					fp_IT<<std::endl;
//				}
				count++;
			}
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//			cout<<" --- "<<level_nodes[i-1].size()<<" nodes - "<<timeused<<endl;

			if(debug) 
			fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl
						<<"nodes in level "<<i-1<<std::endl;
//			int count_null=0, count_node=0;
			if(debug) 
			for(std::list<Node_InterpTree*>::iterator it=level_nodes[i-1].begin();it!=level_nodes[i-1].end();it++)
			{
				printPath(*it,fp_IT);
//				Node_InterpTree *n=*it;
////				count_null=0;
////				count_node=0;
//				while(n!=root)
//				{
//					if(debug) fp_IT<<n<<" - ";
////					if(n->null) count_null++;
////					else count_node++;
//					n=n->parent;
//				}
//				fp_IT<<std::endl;
////				if(count_null>1 && count_null>count_node)
////				{
////					delete *it;
////					it=level_nodes[i-1].erase(it);
////					it--;
////				}
			}
			if(debug) fp_IT<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;

			// for each node in the higher level - level[i-1];
			for(std::list<Node_InterpTree*>::iterator it=level_nodes[i-1].begin();it!=level_nodes[i-1].end();it++)
			{
				// for each feature pair - <cur-i, ref-j>
				for(int j=0;j<features_ref.size();j++)
				{
					if(features_cur[i]->Type()!=features_ref[j]->Type()) continue;
					// now the node pair - node (current level) and *it (previous level);
					Node_InterpTree *node=new Node_InterpTree(features_cur[i],features_ref[j]);
//					cout<<"testing "<<node<<" and "<<*it<<" in level "<<i-1<<std::endl;
					if(debug)
					{
						fp_IT<<std::endl<<"************************************************************************"<<std::endl;
						fp_IT<<"testing "<<node<<" and "<<*it<<" in level "<<i-1<<std::endl;
						printPath(*it,fp_IT);
					}
					bool path_null=true;
					Node_InterpTree *node_interp=*it;
					while(node_interp!=root)
					{
						if(!node_interp->null) {path_null=false; break;}
						node_interp=node_interp->parent;
					}
					if(path_null) 
					{
						insertNode(*it,node); 
						node->initConsistentModel();
						if(debug) fp_IT<<"attach "<<node<<" to "<<*it<<std::endl
									<<"add "<<node<<" to level "<<i<<std::endl;
						continue;
					}
					if(debug) fp_IT<<"derive from "<<node_interp<<std::endl;
					node->deriveConsistentModel(node_interp->consistent_model); // M(N_i) = M(N_{i-1});
					bool attachNode=true;
					while(node_interp!=root)
					{
						if(node_interp->null) 
						{ 
							node_interp=node_interp->parent; 
//							attachNode=true; 
							continue; 
						}
						ModelConsistencyBinary *model=new ModelConsistencyBinary(node, node_interp);
						if(debug) fp_IT<<std::endl<<"--------------------------------------------------------"<<std::endl
									<<model<<std::endl;
//						if(debug) std::cout<<model<<std::endl;
						if(model->intraConsistent())
						{
//							if(node->updateConsistentModel(model))
							if(!node->consistent_model->compute(node))
							{
								attachNode=false;
							}
						}
						else attachNode=false;
						delete model;
						if(!attachNode) break;
						node_interp=node_interp->parent;
					}
					if(attachNode)
					{
//						fp_IT<<std::endl<<"--------------------------------------------------------"<<std::endl;
//						node->consistent_model->compute(node);

//						fp_IT<<std::endl<<"--------------------------------------------------------"<<std::endl;
//						fp_IT<<"fuse the model of "; (*it)->print(fp_IT); fp_IT<<"to "; node->print(fp_IT); fp_IT<<std::endl;
//						if(node->updateConsistentModel((*it)))
						{
							if(debug) fp_IT<<std::endl<<"--------------------------------------------------------"<<std::endl
										<<"attach "<<node<<" to "<<*it<<std::endl
										<<"add "<<node<<" to level "<<i<<std::endl;
							// attach node to *it;
//							(*it)->insertChild(node);
							insertNode(*it,node); 
							node->consistent_model->pushNode(node);
//							level_nodes[i].push_back(node);
//							node->consistent_model->printModel(fp_IT);
							
						}
					}
					else delete node;
				}
				Node_InterpTree *null_node=new Node_InterpTree(features_cur[i]); // null_node->null=true;
				insertNode((*it),null_node);
				if(debug) fp_IT<<"add "<<null_node<<" to level "<<i<<std::endl;
//				cout<<"add "<<null_node<<" to level "<<i<<std::endl;
			}
//			fp_IT<<"####################################################################"<<std::endl;
//			fp_IT<<"################# level "<<i<<" ### printTree() ###########################"<<std::endl;
//			printTree(fp_IT);
//			fp_IT<<"####################################################################"<<std::endl;
//			fp_IT<<"####################################################################"<<std::endl;
//			cout<<"i="<<i<<endl;
		}

		int leaf_level=level_nodes.size()-1; //std::cout<<"levels: "<<leaf_level<<std::endl;
		while(level_nodes[leaf_level].size()==0)
		{
			leaf_level--;
//			cout<<leaf_level<<" "<<level_nodes[leaf_level].size()<<endl;
		}
//		int path_length=0, max_length=0;
//		leaf_max_interp=0;
//		std::cout<<"leaf level: "<<level_nodes[leaf_level].size()<<std::endl;
//		cout<<"leaf_level="<<leaf_level<<endl;
		for(std::list<Node_InterpTree*>::iterator it=level_nodes[leaf_level].begin();
			it!=level_nodes[leaf_level].end();it++)
		{
			qualities.push_back((*it)->quality);
//			path_length=0;
//			Node_InterpTree* ptr=*it;
//			while(ptr!=root)
//			{
//				if(!ptr->null) path_length++;
//				ptr=ptr->parent;
//			}
//			if(path_length>max_length)
//			{
//				max_length=path_length;
//				leaf_max_interp=*it;
//			}
//			printPath(*it,std::cout);
//			std::cout<<"path length="<<path_length<<std::endl;
		}
		std::sort(qualities.begin(),qualities.end());
		std::reverse(qualities.begin(),qualities.end());
		
//		std::cout<<"longest path is "<<std::endl;
//		printPath(leaf_max_interp,std::cout);

//		printNodes(fp_IT);
		
		if(debug) fp_IT.close();
		return true;
	}

	void InterpTree::printTree(std::ostream &os)
	{
		for(int i=0;i<level_nodes.size();i++)
		{
			os<<std::endl<<"level "<<i<<std::endl;
			printLevel(i,os);
//			for(std::list<Node_InterpTree*>::iterator it=level_nodes[i].begin();it!=level_nodes[i].end();it++)
//			{
//				printPath(*it,os);
//			}
		}
	}

	void InterpTree::printLevel(int i, std::ostream &os)
	{
		for(std::list<Node_InterpTree*>::iterator it=level_nodes[i].begin();it!=level_nodes[i].end();it++)
		{
			printPath(*it,os);
		}
	}


	void InterpTree::printPath(Node_InterpTree *node, std::ostream &os)
	{
		Node_InterpTree *ptr=node;
		while(ptr!=root)
		{
			os<<ptr<<" - ";
			ptr=ptr->parent;
		}
		os<<std::endl;
	}

	double GeometricFeatureMatching::match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("geometric_feature_matching.txt",std::ios::app);
			fp<<std::endl<<"****************************GeometricFeatureMatching::match()*******************************"<<std::endl;
			fp<<"scan.timestamp = "<<std::fixed<<scan->time()<<std::endl;
		}

		std::vector<Feature*> features_cur, features_ref;
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{ features_cur.push_back(it->second); }
		for(iterFeature it=scan->ref()->beginFeature();it!=scan->ref()->endFeature();it++)
		{ features_ref.push_back(it->second); }

		timeval start, end;
		double timeused;
		gettimeofday(&start,NULL);
		interp_tree->construct(features_cur,features_ref);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

		if(debug) interp_tree->printTree(fp);

//		fp<<"leaf nodes: "<<std::endl; interp_tree->printLeaves(fp); fp<<std::endl;

//		std::vector<Node_InterpTree*> leaves=interp_tree->getMaxLeaves();
//		for(int i=0;i<leaves.size();i++)
//		{
//			std::vector<Node_InterpTree*> path=interp_tree->getInterp(leaves[i]);
//			fp<<"-------------------------------------"<<std::endl;
//			fp<<i<<"-th max interp:"<<std::endl;
//			std::cout<<std::endl<<i<<"-th max interp:"<<std::endl;
//			for(auto j:path) fp<<j<<" "; fp<<std::endl;
//			scan->Tcr()=interp_tree->Tcr(leaves[i]);
//			fp<<"Tcr = "<<scan->Tcr()<<std::endl;
//			std::cout<<"Tcr = "<<scan->Tcr()<<std::endl;
//			Node_InterpTree* n=leaves[i];
//			while(true)
//			{
//				if(!n->null)
//				{
//					std::cout<<n->error()<<std::endl;
//					break;
//				}
//				n=n->parent;
//			}
//
////			Transform Tcr_true=scan->Tcw()*scan->ref()->Tcw().inv();
////			Transform T_res=scan->Tcr()*Tcr_true.inv();
////			Vector6d xi_res=T_res.getMotionVector();
//			Vector6d xi=scan->Tcr().getMotionVector();
//			double error_translation=xi.block<3,1>(0,0).norm();
//			double error_rotation=xi.block<3,1>(3,0).norm();
//			std::cout<<"error_trns="<<error_translation<<endl;
//			std::cout<<"error_rot ="<<error_rotation<<endl;
//
//			FeatureAssociation *association=new FeatureAssociation;
//			for(int i=0;i<path.size();i++)
//			{
//				if(path[i]->null) continue;
//				association->insert(path[i]->feature_cur,path[i]->feature_ref,scan->ref()->Tcg());
//			}
//			fp<<"association: "<<std::endl; association->print(fp);
//
//			association->vis(scan,vis);
//			delete association;
//
////			vis->removeAllPointClouds();
////			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
////			pcl::transformPointCloud(*scan->ref()->points(),*cloud,scan->Tcr().getMatrix4f());
////			vis->addPointCloud (cloud,std::to_string(scan->ref()->time()));
////			vis->addPointCloud (scan->points(),std::to_string(scan->time()));
////			vis->spin();
//			fp<<"-------------------------------------"<<std::endl;
//		}


		Node_InterpTree* max_leaf=interp_tree->getMaxLeaf();
		std::vector<Node_InterpTree*> path=interp_tree->getInterp(max_leaf);
		if(debug) for(auto j:path) fp<<j<<" "; fp<<std::endl;
		scan->Tcr()=interp_tree->Tcr(max_leaf);
		scan->localize();
		if(debug) fp<<"Tcr = "<<scan->Tcr()<<std::endl;

		scan->association()=new FeatureAssociation;
		for(int i=0;i<path.size();i++)
		{
			if(path[i]->null) continue;
			scan->association()->insert(path[i]->feature_cur,path[i]->feature_ref,scan->ref()->Tcg());
		}
		if(debug) 
		{
			fp<<"association: "<<std::endl; scan->association()->print(fp);
			fp<<"********************************************************************************************"<<std::endl<<std::endl;
			fp.close();
		}
		interp_tree->release();
		return timeused;
	}

	double GeometricFeatureMatching::match(Scan *scan, Map* map)
	{
		if(debug)
		{
			fp.open("geometric_feature_matching.txt",std::ios::app);
			fp<<std::endl<<"****************************GeometricFeatureMatching::match(map)*******************************"<<std::endl;
			fp<<"scan.timestamp = "<<std::fixed<<scan->time()<<std::endl;
		}

		std::vector<Feature*> features_cur, features_ref;
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{ features_cur.push_back(it->second); }
//		for(iterFeature it=scan->ref()->beginFeature();it!=scan->ref()->endFeature();it++)
//		{ features_ref.push_back(it->second); }
		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
		{
			Feature *feat=new Feature(it->second);
			features_ref.push_back(feat);
		}
		cout<<"size: "<<features_cur.size()<<", "<<features_ref.size()<<endl;

		timeval start, end;
		double timeused;
		gettimeofday(&start,NULL);
		interp_tree->construct(features_cur,features_ref);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

		if(debug) interp_tree->printTree(fp);

//		fp<<"leaf nodes: "<<std::endl; interp_tree->printLeaves(fp); fp<<std::endl;

		Node_InterpTree* max_leaf=interp_tree->getMaxLeaf_relocalize();
		cout<<max_leaf<<endl;
		std::vector<Node_InterpTree*> path=interp_tree->getInterp(max_leaf);
		if(debug) for(auto j:path) fp<<j<<" "; fp<<std::endl;
		scan->Tcg()=interp_tree->Tcr(max_leaf);
		if(debug) fp<<"Tcg = "<<scan->Tcg()<<std::endl;

		scan->association_map()=new LandmarkAssociation;
		for(int i=0;i<path.size();i++)
		{
			if(path[i]->null) continue;
			scan->association_map()->insert(path[i]->feature_cur,path[i]->feature_ref->ptrLandmark());
		}
		if(debug)
		{
			fp<<"association_map: "<<std::endl; scan->association_map()->print(fp);
			fp<<"********************************************************************************************"<<std::endl<<std::endl;
			fp.close();
		}
		
		for(int i=0;i<features_ref.size();i++)
		{ delete features_ref[i]; }

		interp_tree->release();
		return timeused;
	}


	using namespace std;
	double GeometricFeatureMatching::match(Scan *scan, Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("geometric_feature_matching.txt",std::ios::app);
			fp<<std::endl<<"****************************GeometricFeatureMatching::match()*******************************"<<std::endl;
			fp<<"scan.timestamp = "<<std::fixed<<scan->time()<<std::endl;
		}

		std::vector<Feature*> features_cur, features_ref;
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{ 
			features_cur.push_back(it->second); 
		}
		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
		{
			Feature *feat=new Feature(it->second,scan->Tcw());
			features_ref.push_back(feat);
		}

		std::vector<double> dist_min; dist_min.resize(features_cur.size(),DBL_MAX);
		std::vector<int> idx; idx.resize(features_cur.size(),-1);
//		double dir_min=DBL_MAX, dist_min=DBL_MAX;
//		int idx=-1;
		for(int j=0;j<features_ref.size();j++)
		{
			for(int i=0;i<features_cur.size();i++)
			{
				if(features_cur[i]->Type()!=features_ref[j]->Type()) continue;
				double dist=features_ref[j]->dist(features_cur[i]);
				double dir=features_ref[j]->delta_dir(features_cur[i]);
				if(debug)
				{
					fp<<endl;
					fp<<"cur "<<features_cur[i]<<endl;
					fp<<"ref "<<features_ref[j]<<endl;
					fp<<"dist="<<dist<<endl;
					fp<<"dir="<<dir<<endl;
				}
//				cout<<"dist="<<dist<<endl;
//				vis->removeAllPointClouds();
//				vis->removeAllCoordinateSystems();
//				scan->vis(vis);
//				features_cur->vis(vis);
				if(dist<dist_min[i] && dir<THRES_RAD)
				{
					dist_min[i]=dist;
					idx[i]=j;
				}
			}
		}
//		for(int i=0;i<features_cur.size();i++)
//		{
//			for(int j=0;j<features_ref.size();j++)
//			{
//				if(features_cur[i]->Type()!=features_ref[j]->Type()) continue;
////				fp<<endl;
////				fp<<"cur "<<features_cur[i]<<endl;
////				fp<<"ref "<<features_ref[j]<<endl;
////				fp<<"dir "<<features_cur[i]->delta_dir(features_ref[j])*180.0/M_PI<<endl;
////				fp<<"dist "<<features_cur[i]->delta_dist(features_ref[j])<<endl;
//				if(features_cur[i]->delta_dir(features_ref[j])<THRES_RAD)
//				{
//					double dist=features_cur[i]->delta_dist(features_ref[j]);
//					if(dist<dist_min[i])
//					{
//						dist_min[i]=dist;
//						idx[i]=j;
//					}
//				}
//			}
//		}

		if(debug) fp<<endl;
		scan->association_map()=new LandmarkAssociation;
		for(int i=0;i<features_cur.size();i++)
		{
//			cout<<features_cur[i]->ID()<<" "<<idx[i]<<" "<<dist_min[i]<<endl;
			if(idx[i]!=-1 && dist_min[i]<THRES_DIST) 
			{
				scan->association_map()->insert(features_cur[i],features_ref[idx[i]]->ptrLandmark());
			}
			else 
			{
				Landmark* lm=new Landmark(features_cur[i],scan->Tcw());
//				map->addLandmark(lm);
				scan->association_map()->insert(features_cur[i],lm);
				if(debug) fp<<"new landmark "<<features_cur[i]<<endl;
			}
		}
		map->addScan(scan,false,true);
		if(debug) {fp<<"association_map: "<<std::endl; scan->association_map()->print(fp);}
		if(debug) fp<<"********************************************************************************************"<<std::endl<<std::endl;
		
		for(int i=0;i<features_ref.size();i++)
		{ delete features_ref[i]; }

		if(debug) fp.close();
		return 0;
	}

//	timeval start, end;
//	double timeused;
//		gettimeofday(&start,NULL);
//		gettimeofday(&end,NULL);
//		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

	// segmented planar regions;
	void GeometricFeatureMatching::visMatchedFeatures(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
//		scan->association()->vis(vis);
//
//		std::cout<<"\nvisualize matched planes"<<std::endl;
//		for(int i=0;i<scan->plane_matches.size();i++)
//		{
//			std::cout<<"\t"<<i<<"cur - red"<<std::endl;
//			sprintf(id,"cur_plane");
//			plane->resize(scan->plane_matches[i].cur->inliers.indices.size());
//			for(int j=0;j<scan->plane_matches[i].cur->inliers.indices.size();j++)
//			{
//				int idx=scan->plane_matches[i].cur->inliers.indices[j];
//				plane->at(j).x=scan->point_cloud->at(idx).x;
//				plane->at(j).y=scan->point_cloud->at(idx).y;
//				plane->at(j).z=scan->point_cloud->at(idx).z;
//				plane->at(j).r=255;
//                plane->at(j).g=0;
//                plane->at(j).b=0;
//			}
//			if (!vis->updatePointCloud (plane, id))
//				vis->addPointCloud (plane, id);
//
//			std::cout<<"\t"<<i<<"ref - blue"<<std::endl;
//			sprintf(id,"ref_plane");
//			plane->resize(scan->plane_matches[i].ref->inliers.indices.size());
//			for(int j=0;j<scan->plane_matches[i].ref->inliers.indices.size();j++)
//			{
//				int idx=scan->plane_matches[i].ref->inliers.indices[j];
//				plane->at(j).x=scan->scan_ref->point_cloud->at(idx).x;
//				plane->at(j).y=scan->scan_ref->point_cloud->at(idx).y;
//				plane->at(j).z=scan->scan_ref->point_cloud->at(idx).z;
//				plane->at(j).r=0;
//                plane->at(j).g=0;
//                plane->at(j).b=255;
//			}
//			if (!vis->updatePointCloud (plane, id))
//				vis->addPointCloud (plane, id);
//
//			vis->spin();
//		}

	}


}


