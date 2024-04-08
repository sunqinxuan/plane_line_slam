/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-06-16 13:37
#
# Filename:		line_matching.h
#
# Description: 
#
===============================================*/

#ifndef _LINE_MATCH_
#define _LINE_MATCH_

#include "types/types.h"

namespace ulysses
{
	void visMatchedLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	struct Node_InterpTree_Line
	{
		Node_InterpTree_Line() {}

		Node_InterpTree_Line(Line *line_ref_, Line *line_cur_)
		{
			line_ref=line_ref_;
			line_cur=line_cur_;
		}

		Line *line_ref,*line_cur;

		Node_InterpTree_Line *parent;
		std::vector<Node_InterpTree_Line*> children;

		void setParent(Node_InterpTree_Line* node)
		{
			parent=node;
			node->children.push_back(this);
			this->layer=node->layer+1;
		}

		void insertChild(Node_InterpTree_Line* node)
		{
			children.push_back(node);
			node->parent=this;
			node->layer=this->layer+1;
		}

		int layer;
	};

	class InterpTree_Line
	{
	public:

		InterpTree_Line()
		{
			root=new Node_InterpTree_Line;
			thres_dir=10.0;
			thres_normal=5.0;
			thres_dist=0.1; // 10cm
			debug=false;
			thres_delta_dir=5.0;
			thres_delta_normal=1.0;
			thres_delta_dist=0.05;
		}

		InterpTree_Line(bool db, double dir, double normal, double dist, double delta_dir, double delta_normal, double delta_dist)
		{
			debug=db;
			root=new Node_InterpTree_Line;
			thres_dir=dir;
			thres_normal=normal;
			thres_dist=dist;
			thres_delta_dir=delta_dir;
			thres_delta_normal=delta_normal;
			thres_delta_dist=delta_dist;
		}

		~InterpTree_Line() {Release();}

		void Release()
		{
			delete root;
			for(std::vector<Node_InterpTree_Line*>::iterator it=nodes.begin();it!=nodes.end();it++)
			{
				delete *it;
			}
			std::vector<Node_InterpTree_Line*>().swap(nodes);
			leaf_max_interp.clear();
		}

		void setDebug(bool d) {debug=d;}

		int getNodeNum() {return nodes.size();}

		Node_InterpTree_Line* getRoot() {return root;}

		std::vector<Node_InterpTree_Line*> getMaxInterp() 
		{
			return leaf_max_interp;
		}

		bool Construct(std::list<Line*> &lines_ref_,std::list<Line*> &lines_cur_,
					   boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		
	private:

		bool debug;

		std::ofstream fp;

		Node_InterpTree_Line *root;
		std::vector<Node_InterpTree_Line*> nodes;
		std::vector<Node_InterpTree_Line*> leaf_max_interp;

		double thres_dir,thres_normal,thres_dist; // thres for consistent_1;
		double thres_delta_dir,thres_delta_normal,thres_delta_dist; // thres for consistent_2;

		bool consistent_1(Node_InterpTree_Line *node);
		bool consistent_2(Node_InterpTree_Line *node1, Node_InterpTree_Line *node2);
		bool consistent_3(Node_InterpTree_Line *node1, Node_InterpTree_Line *node2, Node_InterpTree_Line *node3);
		bool isLeaf(Node_InterpTree_Line *node);
	};

	class LineMatching
	{
	public:

		LineMatching() {}

		LineMatching(const std::string &settingFile)
		{ 
			remove("line_matching.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			debug=(int)settings["debug"]; 
			interp_tree = new InterpTree_Line(debug,
				(double)settings["LineMatching.thres_dir"],
				(double)settings["LineMatching.thres_normal"],
				(double)settings["LineMatching.thres_dist"],
				(double)settings["LineMatching.thres_delta_dir"],
				(double)settings["LineMatching.thres_delta_normal"],
				(double)settings["LineMatching.thres_delta_dist"]);
		}

		~LineMatching() {delete interp_tree;}

		void setDebug(bool d) {debug=d; interp_tree->setDebug(d);}

		bool match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		//void depthFirstSearch();
		//void breadthFirstTravel();
		
	private:

		bool debug;
		bool visual;

		InterpTree_Line *interp_tree;

//		double thres_line2shadow;
	};
}

#endif
