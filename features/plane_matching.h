/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-06-16 13:28
#
# Filename:		plane_matching.h
#
# Description: 
#
===============================================*/

#include "types/types.h"

namespace ulysses
{
	void visMatchedPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	struct Node_InterpTree_Plane
	{
		Node_InterpTree_Plane() {}
		Node_InterpTree_Plane(Plane *plane_ref_, Plane *plane_cur_)
		{
			plane_ref=plane_ref_;
			plane_cur=plane_cur_;
		}

		Plane *plane_ref,*plane_cur;

		Node_InterpTree_Plane *parent;
		std::vector<Node_InterpTree_Plane*> children;

		void setParent(Node_InterpTree_Plane* node)
		{
			parent=node;
			node->children.push_back(this);
			this->layer=node->layer+1;
		}

		void insertChild(Node_InterpTree_Plane* node)
		{
			children.push_back(node);
			node->parent=this;
			node->layer=this->layer+1;
		}

		int layer;
	};

	// layers: numbers of planes in referece frame
	// children per node: numbers of planes in current frame
	class InterpTree_Plane
	{
	public:

		InterpTree_Plane()
		{
			root=new Node_InterpTree_Plane;
			thres_color=1.0;
			thres_delta_angle=0.2; // 0.2rad~=11.5deg
			thres_delta_d=0.07; // 10cm
			debug=false;
		}


		InterpTree_Plane(bool db, double angle, double d, double color)
		{
			root=new Node_InterpTree_Plane;
			thres_delta_angle=angle*M_PI/180.0; // angle~deg
			thres_delta_d=d;
			thres_color=color;
			debug=db;
		}
		~InterpTree_Plane() {Release();}

		void Release()
		{
			delete root;
			for(std::vector<Node_InterpTree_Plane*>::iterator it=nodes.begin();it!=nodes.end();it++)
			{
				delete *it;
			}
			std::vector<Node_InterpTree_Plane*>().swap(nodes);
			leaf_max_interp.clear();
		}

		void setDebug(bool d) {debug=d;}

		int getNodeNum() {return nodes.size();}

		Node_InterpTree_Plane* getRoot() {return root;}

		std::vector<Node_InterpTree_Plane*> getMaxInterp() 
		{
			return leaf_max_interp;
		}

		bool Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_);
		
	private:

		bool debug;

		std::ofstream fp;

		Node_InterpTree_Plane *root;
		std::vector<Node_InterpTree_Plane*> nodes;
		std::vector<Node_InterpTree_Plane*> leaf_max_interp;

		double thres_color; // thres for consistent_1
		double thres_delta_angle; // (consistent_2) if delta_normal_angle<thres then the planes are parallel
		double thres_delta_d; // (consistent_2) if delta_d<thres then the plane pairs are coincident

		bool consistent_1(Node_InterpTree_Plane *node);
		bool consistent_2(Node_InterpTree_Plane *node1, Node_InterpTree_Plane *node2);
		bool consistent_3(Node_InterpTree_Plane *node1, Node_InterpTree_Plane *node2, Node_InterpTree_Plane *node3);
		bool isLeaf(Node_InterpTree_Plane *node);
	};

	class PlaneMatching
	{
	public:

		PlaneMatching() {}

		PlaneMatching(const std::string &settingFile)
		{ 
			remove("plane_matching.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			debug=(int)settings["debug"]; 
			interp_tree = new InterpTree_Plane(debug,
				(double)settings["PlaneMatching.thres_delta_angle"],
				(double)settings["PlaneMatching.thres_delta_d"],
				(double)settings["PlaneMatching.thres_color"]);
		}

		~PlaneMatching() {delete interp_tree;}

		void setDebug(bool d) {debug=d; interp_tree->setDebug(d);}

		bool match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		//void depthFirstSearch();
		//void breadthFirstTravel();
		
	private:

		bool debug;
		bool visual;

		InterpTree_Plane *interp_tree;
	};
}
