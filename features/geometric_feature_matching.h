/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-21 08:50
#
# Filename:		geometric_feature_matching.h
#
# Description: 
#
===============================================*/
#ifndef _GEO_FEATURE_MATCH_H_
#define _GEO_FEATURE_MATCH_H_

#include "types/types.h"
#include "types/map.h"
#include <random>

namespace ulysses
{
	extern bool dbg;
	extern std::ofstream fp_IT;
	extern double THRES_RAD, THRES_DIST;

	class ModelConsistencyBinary;
	class ModelConsistency;
	struct Node_InterpTree;

	struct Node_Quality
	{
		Node_Quality(int i, Node_InterpTree *p) : value(i), ptr(p) {}
		int value;
		Node_InterpTree *ptr;
		bool operator == (Node_Quality q)
		{
			if(value==q.value) return true;
			else return false;
		}
		bool operator < (Node_Quality q)
		{
			if(value<q.value) return true;
			else return false;
		}
		bool operator < (int i)
		{
			if(value<i) return true;
			else return false;
		}
		bool operator > (int i)
		{
			if(value>i) return true;
			else return false;
		}
		friend std::ostream & operator << (std::ostream &os, const Node_Quality q)
		{
			os<<q.value;
			return os;
		}
		void increase(Node_Quality q) {value=q.value+1;}
		void copy(Node_Quality q) {value=q.value;}
	};

	struct Node_InterpTree
	{
		FEATURE_TYPE type;

		Node_InterpTree() : id("Node<root>"), null(true), level(-1), quality(Node_Quality(0,this)) {}
		Node_InterpTree(Feature *cur) : feature_cur(cur), type(cur->Type()), 
						id("Node<"+cur->ID()+"-null>"), null(true), level(0), quality(Node_Quality(0,this)) {}
		Node_InterpTree(Feature *cur, Feature *ref) : feature_cur(cur), feature_ref(ref), type(cur->Type()), 
						id("Node<"+cur->ID()+"-"+ref->ID()+">"), null(false), level(0), quality(Node_Quality(0,this)) {}

//		// delete it after testing 
//		Node_InterpTree(Plane *pc, Plane *pr) : null(false)
//		{
//			feature_cur=new Feature(pc);
//			feature_ref=new Feature(pr);
//		}
//		Node_InterpTree(Line *lc, Line *lr) : null(false)
//		{
//			feature_cur=new Feature(lc);
//			feature_ref=new Feature(lr);
//		}

		~Node_InterpTree();// { if(!null) delete consistent_model; }

		Feature *feature_ref, *feature_cur;
		std::string id;
		int level;
		bool null;
		Node_Quality quality;

		Node_InterpTree *parent;
		std::vector<Node_InterpTree*> children;

		ModelConsistency *consistent_model;
		void initConsistentModel();// {consistent_model->initializeModel(node);}
		void deriveConsistentModel(ModelConsistency* model);// {consistent_model->deriveFrom(model);}
		bool updateConsistentModel(ModelConsistencyBinary *model);// {consistent_model->update(model);}
//		bool updateConsistentModel(Node_InterpTree *node) {consistent_model->update(node->consistent_model);}
		double error();// { return consistent_model->transError()+consistent_model->rotError(); }

//		void setParent(Node_InterpTree* node)
//		{
//			parent=node;
//			node->children.push_back(this);
//			this->level=node->level+1;
//		}

		void insertChild(Node_InterpTree* node)
		{
			children.push_back(node);
			node->parent=this;
			node->level=this->level+1;
//			*node->consistent_model=*consistent_model;
//			node->consistent_model->pushNode(this);
		}

//		bool modelConsistent(Node_InterpTree* node);
//
//		bool spatialConsistent(Node_InterpTree* node);

		friend std::ostream & operator << (std::ostream &os, const Node_InterpTree *n)
		{
			os<<n->id<<"("<<n->quality<<")";
			return os;
		}

	};

	class ProblemRotation
	{
	public:
		ProblemRotation() {}
		ProblemRotation(const ProblemRotation* r) { *this=*r; }
		ProblemRotation(Eigen::Vector3d nri_, Eigen::Vector3d nrj_, Eigen::Vector3d nci_, Eigen::Vector3d ncj_)
		: nri(nri_),  nrj(nrj_), nci(nci_), ncj(ncj_) {}
		
		void input(Eigen::Vector3d nri_, Eigen::Vector3d nrj_, Eigen::Vector3d nci_, Eigen::Vector3d ncj_)
		{ nri=nri_; nrj=nrj_; nci=nci_; ncj=ncj_; }

		bool solve();
		bool fuse(const ProblemRotation *r) {return fuseRotation(r);}

		Eigen::Vector3d getAxisAngle(double phi=0) const 
		{
			if(hasDOF) return constrain1DOF(phi);
			else return angle*axis;
		}

		Eigen::Matrix3d getRotationMatrix(double phi=0) const
		{ 
//			std::cout<<"-----getRotationMatrix-----"<<std::endl;
			if(!isConsistent) return Eigen::Matrix3d::Zero();
			if(hasDOF) return rotation_matrix(constrain1DOF(phi));
			if(axis.norm()<1e-10) return Eigen::Matrix3d::Identity();
			return rotation_matrix(angle*axis); 
		}

		bool HasDOF() const { return hasDOF; }
		bool IsConsistent() const { return isConsistent; }

		Eigen::Vector3d get_nc() const {return nc;}
		Eigen::Vector3d get_nr() const {return nr;}
		Eigen::Vector3d getNormal() const {Eigen::Vector3d n=nc-nr; return n;}

	private:

		Eigen::Vector3d nri, nrj, nci, ncj;

		Eigen::Vector3d axis;
		double angle;
		bool isConsistent;

		// if hasDOF
		bool hasDOF;
		Eigen::Vector3d nc,nr;

//		double THRES_RAD;

		bool fuseRotation(const ProblemRotation *rotation);

		bool solveRotation(Eigen::Vector3d &axis_local, double &angle_local, double &error_local);
		bool CaseI_CaseII(Eigen::Vector3d &axis_local, double &angle_local, double &error_local);
		bool CaseI(Eigen::Vector3d &rot_axis, double &rot_angle, double &rot_error);
		bool CaseII(Eigen::Vector3d &rot_axis, double &rot_angle, double &rot_error);

		double residual(Eigen::Vector3d rot_axis, double rot_angle)
		{
			Eigen::Matrix3d R;
			if(rot_axis.norm()<1e-10) R.setIdentity(); 
			else R=rotation_matrix(rot_angle*rot_axis);
			Eigen::Vector3d ri=nci-R*nri;
			Eigen::Vector3d rj=ncj-R*nrj;
			double error=0.5*(ri.norm()+rj.norm());
			return error;
		}

		double cos_theta(Eigen::Vector3d r, Eigen::Vector3d nc, Eigen::Vector3d nr) const
		{
			double den=1.0-r.dot(nc)*r.dot(nr);
			double result=1.0-(1.0-nc.dot(nr))/den;
			return result;
		}

		double sin_theta(Eigen::Vector3d r, Eigen::Vector3d nc, Eigen::Vector3d nr) const
		{
			double den=1.0-r.dot(nc)*r.dot(nr);
			double result=nc.dot(r.cross(nr))/den;
			return result;
		}

		// compute the rotation angle from nr to nc;
		// rotation axis is r;
		double rotation_angle(Eigen::Vector3d r, Eigen::Vector3d nc, Eigen::Vector3d nr) const 
		{
			nc.normalize();
			nr.normalize();
			double theta=atan2(sin_theta(r,nc,nr),cos_theta(r,nc,nr));
			if(theta<0) theta+=(2*M_PI);
			return theta; // in rad;
		}

		double included_angle(Eigen::Vector3d nc, Eigen::Vector3d nr) const
		{
			nc.normalize();
			nr.normalize();
			double ncTnr=nc.dot(nr);
			if(ncTnr>=0.99999) ncTnr=0.99999;
			if(ncTnr<=-0.99999) ncTnr=-0.99999;
			return acos(ncTnr); // in rad;
		}

		Eigen::Matrix3d rotation_matrix(Eigen::Vector3d w) const
		{
//			std::cout<<"axis_angle="<<w.transpose()<<std::endl;
			double theta=w.norm();
//			std::cout<<"theta="<<theta<<std::endl;
			Eigen::Vector3d r=w.normalized();
//			std::cout<<"r="<<r.transpose()<<std::endl;
			Eigen::Matrix3d R= cos(theta)*Eigen::Matrix3d::Identity()
				   + (1.0-cos(theta))*r*r.transpose()
				   + sin(theta)*Transform::skew_sym(r);
			return R;
		}

		Eigen::Vector3d constrain1DOF(double phi) const 
		{
			Eigen::Vector3d nc=nci+ncj, nr=nri+nrj;
//			std::cout<<"nc="<<nc.transpose()<<std::endl;
//			std::cout<<"nr="<<nr.transpose()<<std::endl;
			Eigen::Vector3d rx=-nc.cross(nc+nr); rx.normalize();
			Eigen::Vector3d ry=nc+nr; ry.normalize();
//			std::cout<<"rx="<<rx.transpose()<<std::endl;
//			std::cout<<"ry="<<ry.transpose()<<std::endl;

			Eigen::Vector3d r=rx*cos(phi)+ry*sin(phi);
//			std::cout<<"r="<<r.transpose()<<std::endl;
			double theta=rotation_angle(r,nc,nr);
//			std::cout<<"theta="<<theta<<std::endl;
			return theta*r;
		}

		bool inRotation1DoF(Eigen::Vector3d axis_angle) const
		{
			double theta=axis_angle.norm();
			Eigen::Vector3d r=axis_angle.normalized();
			Eigen::Vector3d v=getNormal();
			if(fabs(v.dot(r))<THRES_RAD)
			{
				double theta2=rotation_angle(r,0.5*(nci+ncj),0.5*(nri+nrj));
				if(fabs(theta-theta2)<THRES_RAD) return true;
				else return false;
			}
			else return false;
		}

	};

	class ProblemTranslation
	{
	public:
		enum TYPE {PLANE_PLANE, LINE_LINE, PLANE_LINE};
		TYPE type;

		enum SOLUTION_CASE {CONSTRAINED, LIN1DOF, LIN2DOF, ROT1DOF, NONE};
		SOLUTION_CASE solution_case;

		ProblemTranslation() {}
		ProblemTranslation(ProblemRotation* r) 
		{
			rotation_consistency=r;
		}
		ProblemTranslation(const ProblemTranslation* t, ProblemRotation* r) 
		{
			*this=*t;
			rotation_consistency=r;
		}
		ProblemTranslation(Node_InterpTree* ni, Node_InterpTree *nj, ProblemRotation *rot) 
			: node_i(ni), node_j(nj), rotation_consistency(rot) {}

		~ProblemTranslation() {}

		void input(Node_InterpTree* ni, Node_InterpTree *nj, ProblemRotation *rot) 
		{ node_i=ni; node_j=nj; rotation_consistency=rot; }

		bool solve();
		bool fuse(const ProblemTranslation *t) {return fuseTranslation(t);}

		bool IsConsistent() const { return isConsistent; }

		Eigen::Vector3d getTranslation() const {return translation;}
		Eigen::Vector3d getDirection() const {return direction;}

		void setRotationConsistency(ProblemRotation *r) {rotation_consistency=r;}

	private:

		Node_InterpTree *node_i, *node_j;
		ProblemRotation *rotation_consistency;

//		Eigen::MatrixXd A,b;

		bool isConsistent;
		Eigen::Vector3d translation, direction;
		// if solution_case=ROT1DOF 
		// u0=translation;
		// v0=direction;
		// t0=distance;
		// t=t0*mu+mu.cross(u0)*cos(phi)+u0*sin(phi);
//		double distance;

//		double THRES_RAD, THRES_DIST;

		// plane and plane;
		SOLUTION_CASE plane_plane_parallel(Eigen::Vector3d nci, Eigen::Vector3d nri, double dci, double dri,
				Eigen::Vector3d ncj, Eigen::Vector3d nrj, double dcj, double drj,
				Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error);
		SOLUTION_CASE plane_plane_nonparallel(Eigen::Vector3d nci, Eigen::Vector3d nri, double dci, double dri,
				Eigen::Vector3d ncj, Eigen::Vector3d nrj, double dcj, double drj,
				Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error);

		// line and line;
		SOLUTION_CASE line_line_parallel(Eigen::Vector3d nci, Eigen::Vector3d nri, Eigen::Vector3d uci, Eigen::Vector3d uri,
				Eigen::Vector3d ncj, Eigen::Vector3d nrj, Eigen::Vector3d ucj, Eigen::Vector3d urj,
				Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error, 
				ProblemRotation* problem);
		SOLUTION_CASE line_line_nonparallel(Eigen::Vector3d nci, Eigen::Vector3d nri, Eigen::Vector3d uci, Eigen::Vector3d uri,
				Eigen::Vector3d ncj, Eigen::Vector3d nrj, Eigen::Vector3d ucj, Eigen::Vector3d urj,
				Eigen::Vector3d &translation_local, double &translation_error);

		// plane and line;
		SOLUTION_CASE plane_line_parallel(Eigen::Vector3d nc, Eigen::Vector3d nr, double dc, double dr, 
				Eigen::Vector3d vc, Eigen::Vector3d vr, Eigen::Vector3d uc, Eigen::Vector3d ur, 
				Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error);
		SOLUTION_CASE plane_line_intersect(Eigen::Vector3d nc, Eigen::Vector3d nr, double dc, double dr, 
				Eigen::Vector3d vc, Eigen::Vector3d vr, Eigen::Vector3d uc, Eigen::Vector3d ur, 
				Eigen::Vector3d &translation_local, Eigen::Vector3d &direction_local, double &translation_error);

		bool constrainRotDOF()
		{
			if(solution_case==ROT1DOF && !rotation_consistency->HasDOF())
			{
				Eigen::Matrix3d R=rotation_consistency->getRotationMatrix();
				translation=translation+R*direction;
				solution_case=CONSTRAINED;
				return true;
			}
			else return false;
		}

		// if return true, the current model is updated;
		bool fuseTranslation(const ProblemTranslation *translation);

		void ModelType()
		{
			if(node_i->type==PLANE && node_j->type==PLANE) { type=PLANE_PLANE; }
			else if(node_i->type==LINE && node_j->type==LINE) { type=LINE_LINE; }
			else if(node_i->type==PLANE && node_j->type==LINE) { type=PLANE_LINE; }
			else if(node_i->type==LINE && node_j->type==PLANE) { type=PLANE_LINE; }
		}

		double included_angle(Eigen::Vector3d nc, Eigen::Vector3d nr) const
		{
			nc.normalize();
			nr.normalize();
			double ncTnr=nc.dot(nr);
			if(ncTnr>=0.99999) ncTnr=0.99999;
			if(ncTnr<=-0.99999) ncTnr=-0.99999;
			return acos(ncTnr); // in rad;
		}

		SOLUTION_CASE fuse_Constrained_Constrained(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d &t);
		SOLUTION_CASE fuse_Constrained_Lin1DOF(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
												Eigen::Vector3d &t);
		SOLUTION_CASE fuse_Constrained_Lin2DOF(Eigen::Vector3d t1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
												Eigen::Vector3d &t);
		SOLUTION_CASE fuse_Lin1DOF_Lin1DOF(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t, Eigen::Vector3d &v);
		SOLUTION_CASE fuse_Lin1DOF_Lin2DOF(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t, Eigen::Vector3d &v);
		SOLUTION_CASE fuse_Lin2DOF_Lin2DOF(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t, Eigen::Vector3d &v);
		SOLUTION_CASE fuse_Rot1DOF_Lin2DOF(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t, Eigen::Vector3d &v);
		SOLUTION_CASE fuse_Rot1DOF_Rot1DOF(Eigen::Vector3d t1, Eigen::Vector3d v1, Eigen::Vector3d t2, Eigen::Vector3d v2, 
											Eigen::Vector3d &t, Eigen::Vector3d &v);
	};

	class ModelConsistencyBinary
	{
	public:
		enum TYPE {CONSTRAINED, TRANS1DOF, ROT1DOF, HAS3DOF};
		TYPE type;

		ModelConsistencyBinary()
		{ 
			rotation_consistency=new ProblemRotation; 
			translation_consistency=new ProblemTranslation; 
			isConsistent=false;
			empty=true;
		}

		ModelConsistencyBinary(Node_InterpTree* ni, Node_InterpTree *nj) : node_i(ni), node_j(nj) 
		{ 
			rotation_consistency=new ProblemRotation;
			translation_consistency=new ProblemTranslation; 
			isConsistent=false;
			empty=true;
		}

		~ModelConsistencyBinary() 
		{
			delete rotation_consistency;
			delete translation_consistency;
		}

		void input(Node_InterpTree* ni, Node_InterpTree *nj) { node_i=ni; node_j=nj; }

		// return whetheer this model is self-consistent;
		bool intraConsistent()
		{
			empty=false;
			isConsistent=false;

			Eigen::Vector3d nci = node_i->feature_cur->getDirection();
			Eigen::Vector3d nri = node_i->feature_ref->getDirection();
			Eigen::Vector3d ncj = node_j->feature_cur->getDirection();
			Eigen::Vector3d nrj = node_j->feature_ref->getDirection();

			rotation_consistency->input(nri,nrj,nci,ncj);
			translation_consistency->input(node_i,node_j,rotation_consistency);

			isConsistent = rotation_consistency->solve() && translation_consistency->solve();
			ModelType();

			if(dbg) fp_IT<<std::endl<<"intraConsistent() => "<<isConsistent<<std::endl;

			return isConsistent;
		}

//		// return whether this model is consistent with the input model;
//		// i.e., whether they represent the same transform;
//		bool interConsistent(ModelConsistencyBinary *model) const 
//		{
//			bool inter_consistent;
//			inter_consistent = this->rotation_consistency->equal(model->rotationConsistency())
//						    && this->translation_consistency->equal(model->translationConsistency());
//			return inter_consistent;
//		}

		const ProblemRotation* rotationConsistency() const {return rotation_consistency;}
		const ProblemTranslation* translationConsistency() const {return translation_consistency;}
		bool IsEmpty() const { return empty; }
		bool IsConsistent() const { return isConsistent; }
		Node_InterpTree* getNode_i() const { return node_i; }
		Node_InterpTree* getNode_j() const { return node_j; }

		// update "this" with the input "model";
		// return true if "this" is consistent with "model";
		bool update(const ModelConsistencyBinary* model)
		{
			empty=false;
			if(!rotation_consistency->fuse(model->rotationConsistency())) return false;
			return  translation_consistency->fuse(model->translationConsistency());
		}
		
		friend std::ostream & operator << (std::ostream &os, const ModelConsistencyBinary *m)
		{
			os<<"Model - "<<m->node_i<<" & "<<m->node_j;
			return os;
		}

	private:

		bool isConsistent, empty;
		Node_InterpTree *node_i, *node_j;

		ProblemRotation *rotation_consistency;
		ProblemTranslation *translation_consistency;

//		// update "this" with the input "model";
//		// return true if "this" is consistent with "model";
//		bool updateModel(const ModelConsistencyBinary* model);

		void ModelType()
		{
			if(rotation_consistency->HasDOF())
			{
				switch(translation_consistency->solution_case)
				{
				case ProblemTranslation::LIN2DOF :
					type=HAS3DOF;
					break;
				case ProblemTranslation::ROT1DOF :
					type=ROT1DOF;
					break;
				default : break;
				}
			}
			else 
			{
				switch(translation_consistency->solution_case)
				{
				case ProblemTranslation::LIN1DOF :
					type=TRANS1DOF;
					break;
				case ProblemTranslation::CONSTRAINED :
					type=CONSTRAINED;
					break;
				default : break;
				}
			}
		}

	};

	class ModelConsistency
	{
	public:
		ModelConsistency() 
		{
			rotation=new ProblemRotation;
			translation=new ProblemTranslation;
		}

		~ModelConsistency() 
		{
			delete rotation; delete translation;
			for(int i=0;i<models.size();i++) delete models[i];
		}

		int sizeNodes() const {return nodes.size();}
		Node_InterpTree* Node(int i) const {return nodes[i];}
		Transform getTcr() const {return Tcr;}
		void pushNode(Node_InterpTree* node) {nodes.push_back(node);}
		void pushModel(ModelConsistencyBinary* model) {models.push_back(model);}

		const ProblemRotation* rotationConsistency() const {return rotation;}
		const ProblemTranslation* translationConsistency() const {return translation;}

		// node is on the 0st level (child of the root);
		void initializeModel(Node_InterpTree* node)
		{
			if(dbg) fp_IT<<std::endl<<"-----------------------ModelConsistency::initializeModel()-----------------------"<<std::endl;
			models.clear();
//			if(node->null)
//			{
//				rotation=new ProblemRotation;
//				translation=new ProblemTranslation(rotation);
//			}
//			else 
			{
				pushNode(node);
				Eigen::Vector3d nr=node->feature_ref->getDirection();
				Eigen::Vector3d nc=node->feature_cur->getDirection();
				rotation->input(nr,nr,nc,nc); rotation->solve();
				translation->input(node,node,rotation); translation->solve();
			}
		}

		void deriveFrom(const ModelConsistency* model) 
		{
			models.clear();
			for(int i=0;i<model->sizeNodes();i++) pushNode(model->Node(i));
			*rotation=*model->rotationConsistency();
			*translation=*model->translationConsistency();
			translation->setRotationConsistency(rotation);
		}

//		bool isConsistent(const ModelConsistencyBinary* model) const;
		
		// update this model with the input binary model;
		// if return true - this model is updated;
		// if return false - this model is unchanged;
		bool update(ModelConsistencyBinary *model)
		{
//			models.push_back(model_binary);
			ProblemRotation rot=*rotation;
			if(rot.fuse(model->rotationConsistency())) 
			{
				*rotation=rot;
				if(translation->fuse(model->translationConsistency()))
				{
					pushModel(model);
					return true;
				}
				else return false;
			}
			else return false;
		}
		
		bool compute(Node_InterpTree* node) {return computeTransform(node);}

		double rotError() const {return rotation_error;}
		double transError() const {return translation_error;}

	private:
		// cannot be deleted in this class!
		std::vector<Node_InterpTree*> nodes;

		// each model - <node, nodes[i]>
		std::vector<ModelConsistencyBinary*> models;

		ProblemRotation *rotation;
		ProblemTranslation *translation;

		double rotation_error, translation_error;

//		double THRES_RAD, THRES_DIST;
		ulysses::Transform Tcr;
		bool computeTransform(Node_InterpTree* node);

	};

	// layers: numbers of geometric features in referece frame
	// children per node: numbers of geometric features in current frame
	class InterpTree
	{
	public:

		InterpTree(int s):max_size(s)
		{
			root=new Node_InterpTree;
			remove("geometric_feature_matching.txt");
		}

		~InterpTree() {}

		void release()
		{
			delete root;
			for(int i=0;i<level_nodes.size();i++)
			{
				for(std::list<Node_InterpTree*>::iterator it=level_nodes[i].begin();
					it!=level_nodes[i].end();it++)
				{
					delete *it;
				}
				std::list<Node_InterpTree*>().swap(level_nodes[i]);
			}
			std::vector<std::list<Node_InterpTree*> >().swap(level_nodes);
			std::vector<Node_Quality>().swap(qualities);
//			for(std::vector<Node_InterpTree*>::iterator it=nodes.begin();it!=nodes.end();it++)
//			{
//				delete *it;
//			}
//			std::vector<Node_InterpTree*>().swap(nodes);
//			leaf_max_interp.clear();
		}

		void setDebug(bool d) {debug=d;}

		Node_InterpTree* getRoot() {return root;}
//		Node_InterpTree* getMaxInterp() {return leaf_max_interp;}

		std::vector<Node_InterpTree*> getMaxLeaves() 
		{
			std::vector<Node_InterpTree*> result;
			for(int i=0;i<qualities.size();i++)
			{
				if(qualities[i]==qualities[0]) result.push_back(qualities[i].ptr);
				else break;
			}
			return result;
//			return qualities[0].ptr; 
		}

		Node_InterpTree* getMaxLeaf()
		{
			std::vector<Node_InterpTree*> leaves=getMaxLeaves();
			double min=DBL_MAX; int idx=-1;
			for(int i=0;i<leaves.size();i++)
			{
				Vector6d xi=Tcr(leaves[i]).getMotionVector();
				double error=xi.norm();
				if(error<min)
				{
					min=error;
					idx=i;
				}
//				if(leaves[i]->error()<min)
//				{
//					min=leaves[i]->error();
//					idx=i;
//				}
			}
			return leaves[idx];
		}

		Node_InterpTree* getMaxLeaf_relocalize()
		{
			std::vector<Node_InterpTree*> leaves=getMaxLeaves();
			double min=DBL_MAX; int idx=-1;
			for(int i=0;i<leaves.size();i++)
			{
//				cout<<i<<", "<<leaves[i]<<endl;
				Node_InterpTree* ptr=leaves[i];
				while(ptr!=root)
				{
					if(!ptr->null)
					{
						if(ptr->error()<min)
						{
							min=ptr->error();
							idx=i;
						}
						break;
					}
					ptr=ptr->parent;
				}
			}
			return leaves[idx];
		}

		std::vector<Node_InterpTree*> getInterp(Node_InterpTree* leaf)
		{
			std::vector<Node_InterpTree*> path;
			Node_InterpTree* ptr=leaf;
			for(int i=0;i<qualities.size();i++)
			{
				if(qualities[i].ptr->null) continue;
//				std::cout<<qualities[i].ptr<<" "
//						 <<qualities[i].ptr->consistent_model->rotError()<<" "
//						 <<qualities[i].ptr->consistent_model->transError()<<std::endl;
			}
			while(ptr!=root)
			{
				if(!ptr->null) path.push_back(ptr);
				ptr=ptr->parent;
			}
			return path;
		}

		Transform Tcr(Node_InterpTree* leaf) const 
		{
			Transform Tcr;
//			Node_InterpTree* ptr=getLeaf();
			Node_InterpTree* ptr=leaf;
			while(ptr!=root)
			{
				if(!ptr->null)
				{
					Tcr=ptr->consistent_model->getTcr();
					break;
				}
				ptr=ptr->parent;
			}
			return Tcr;
		}

		void printTree(std::ostream &os);
		void printPath(Node_InterpTree *node, std::ostream &os);
		void printLevel(int i, std::ostream &os);
		void printLeaves(std::ostream &os) { printLevel(level_nodes.size()-1,os); }

		bool construct(const std::vector<Feature*> &features_cur, const std::vector<Feature*> &features_ref);
		
	private:

		bool debug;
		int max_size;

		Node_InterpTree *root;
//		std::vector<Node_InterpTree*> nodes;
//		Node_InterpTree* leaf_max_interp;
		std::vector<Node_Quality> qualities;

		std::vector<std::list<Node_InterpTree*> > level_nodes;
		std::vector<int> level_max_quality;

//		double thres_color; // thres for consistent_1
//		double thres_delta_angle; // (consistent_2) if delta_normal_angle<thres then the planes are parallel
//		double thres_delta_d; // (consistent_2) if delta_d<thres then the plane pairs are coincident
//		double THRES_RAD, THRES_DIST;


		void insertNode(Node_InterpTree *insert_to, Node_InterpTree *node)
		{
			insert_to->insertChild(node);
			level_nodes[node->level].push_back(node);
			if(node->null) node->quality.copy(insert_to->quality);
			else node->quality.increase(insert_to->quality);
			if(node->quality>level_max_quality[node->level])
			{
				level_max_quality[node->level]=node->quality.value;
			}
			
//			Node_InterpTree *ptr=insert_to;
//			while(ptr!=root)
//			{
//				if(!ptr->null)
//				{
//					node->quality=ptr->quality+1;
//					break;
//				}
//				ptr=ptr->parent;
//			}
		}

		bool comparePaths(std::list<Node_InterpTree*>::iterator &it1, std::list<Node_InterpTree*>::iterator &it2) 
		{
			if(*it1==*it2 || (*it1)->level!=(*it2)->level) return false;
			Node_InterpTree *n1=(*it1);
			Node_InterpTree *n2=(*it2);
//			int num_level=(*it1)->level+1;
//			int num_equal=0, num_null_1=0, num_null_2=0, num_null_both=0;
			bool null_path=true;
			int equal_steps=0;
			while(n1!=root && n2!=root)
			{
				if(n1->id.compare(n2->id)==0) 
				{
					equal_steps++;
//					if(!n1->null || !n2->null) null_path=false;
				}
				else break;
				n1=n1->parent;
				n2=n2->parent;
			}
//			for(int i=0;i<3;i++)
//			{
//				if(!n1->null || !n2->null) null_path=false;
//				if(n1->id.compare(n2->id)!=0) 
//				{
//					equal_in_steps=false;
//					break;
//				}
//				n1=n1->parent;
//				n2=n2->parent;
//				if(n1==root || n2==root) 
//				{
//					equal_in_steps=false;
//					break;
//				}
//			}
			bool sure_to_delete=true;
			if(equal_steps>=3)
			{
				if(n1->null || n2->null)
				{
					Node_InterpTree *nn1=n1->parent, *nn2=n2->parent;
					while(nn1!=root && nn2!=root)
					{
						if(nn1->id.compare(nn2->id)!=0) 
						{ sure_to_delete=false; break; }
						nn1=nn1->parent; nn2=nn2->parent;
					}
				}
				if(sure_to_delete)
				{
//					if(n1->null)
//					{
//						int l=(*it1)->level;
//						if(debug)
//						{
//							fp_IT<<std::endl<<"deleted a node: "<<std::endl;
//							fp_IT<<"equal_steps="<<equal_steps<<std::endl;
//							fp_IT<<"n1="<<n1<<std::endl;
//							printPath(*it1,fp_IT);
//							fp_IT<<"n2="<<n2<<std::endl;
//							printPath(*it2,fp_IT);
//						}
//						delete *it1;
//						it1=level_nodes[l].erase(it1);
////						if(it1!=level_nodes[l].begin()) 
//						it1--;
//					}
//					else 
					if(n2->null)
					{
						int l=(*it2)->level;
						if(debug)
						{
							fp_IT<<std::endl<<"deleted a node: "<<std::endl;
							fp_IT<<"equal_steps="<<equal_steps<<std::endl;
							fp_IT<<"n2="<<n2<<std::endl;
							printPath(*it2,fp_IT);
							fp_IT<<"n1="<<n1<<std::endl;
							printPath(*it1,fp_IT);
						}
						delete *it2;
						it2=level_nodes[l].erase(it2);
//						if(it2!=level_nodes[l].begin()) 
						it2--;
					}
					else sure_to_delete=false;
				}
			}
			return sure_to_delete;
		}

	};

	class GeometricFeatureMatching
	{
	public:

		GeometricFeatureMatching(const std::string &settingFile)
		{
			remove("geometric_feature_matching.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			debug=(int)settings["debug"]; 
			settings["debug"]>>debug; 
			int max_size; settings["max_size"]>>max_size; 
			settings.release();
			interp_tree = new InterpTree(max_size);
			interp_tree->setDebug(debug); dbg=debug;
		}

		~GeometricFeatureMatching() { delete interp_tree; }

		void setDebug(bool d) {debug=d; interp_tree->setDebug(d); dbg=d;}

		double match(Scan* scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double match(Scan* scan, Map* map);

		double match(Scan* scan, Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void visMatchedFeatures(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		//void depthFirstSearch();
		//void breadthFirstTravel();
		
	private:

		bool debug;
		InterpTree *interp_tree;

		std::ofstream fp;
	};
}

#endif
