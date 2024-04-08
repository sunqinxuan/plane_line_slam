/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-06-16 13:37
#
# Filename:		line_matching.cpp
#
# Description: 
#
===============================================*/

#include "features/line_matching.h"

namespace ulysses
{
	// construct interpretation tree from two line set
	// each path down to the leaf represent a hypothesis
	// *************************************************
	// consider: the circumstances returning false
	bool InterpTree_Line::Construct(std::list<Line*> &lines_ref_,std::list<Line*> &lines_cur_,
									boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		fp.open("line_matching.txt",std::ios::app);
		if(debug)
			fp<<"**************************************************************"<<std::endl;
		// destruct the existing interpretation tree
		Release();
		root=new Node_InterpTree_Line;
		Node_InterpTree_Line *tmp_node, *insert_node, *node_attach, *interp_node2, *interp_node3;
		std::vector<Node_InterpTree_Line*> nodes_tmp;
		bool flag_consistent=true;
		tmp_node=new Node_InterpTree_Line();
		int interp_length_max=0, interp_length;
//		leaf_max_interp=root;
		//fp<<"nodes:"<<nodes.size()<<std::endl;
		if(debug)
		{
			fp<<"line_cur:"<<lines_cur_.size()<<std::endl;
			for(std::list<Line*>::iterator it_cur=lines_cur_.begin();it_cur!=lines_cur_.end();it_cur++)
			{
				fp<<(*it_cur)->id<<", "<<(*it_cur)->u.transpose()<<", "<<(*it_cur)->v.transpose()<<std::endl;
			}
			fp<<"line_ref:"<<lines_ref_.size()<<std::endl;
			for(std::list<Line*>::iterator it_ref=lines_ref_.begin();it_ref!=lines_ref_.end();it_ref++)
			{
				fp<<(*it_ref)->id<<", "<<(*it_ref)->u.transpose()<<", "<<(*it_ref)->v.transpose()<<std::endl;
			}
		}
		// for any two lines out of lines_cur_ and lines_ref_
		for(std::list<Line*>::iterator it_cur=lines_cur_.begin();it_cur!=lines_cur_.end();it_cur++)
		{
			for(std::list<Line*>::iterator it_ref=lines_ref_.begin();it_ref!=lines_ref_.end();it_ref++)
			{
				tmp_node->line_cur=*it_cur;
				tmp_node->line_ref=*it_ref;
				if(debug)
				{
					fp<<std::endl<<"now in: <"<<tmp_node->line_cur->id<<","<<tmp_node->line_ref->id<<"> "<<std::endl;
				}
				if(!consistent_1(tmp_node))
					continue;
				// it has to be ensured that the line id 0 in lines_cur_ must have a correspondence in lines_ref_
				// and there consistent_1 must be satisfied
				// otherwise there will be no node insert to the root
				//if(it_cur==lines_cur_.begin())
				{
					insert_node=new Node_InterpTree_Line(*it_ref,*it_cur);
					root->insertChild(insert_node);
					nodes.push_back(insert_node);
					if(debug)
					{
						fp<<"insert <"<<insert_node->line_cur->id<<","<<insert_node->line_ref->id<<"> to root"<<std::endl;
//						std::cout<<"insert <"<<insert_node->line_cur->id<<","<<insert_node->line_ref->id<<"> to root"<<std::endl;
					}
					//continue;
				}
				if(debug)
				{
					fp<<"nodes in the tree: ";
					for(std::vector<Node_InterpTree_Line*>::iterator it=nodes.begin();it!=nodes.end();it++)
					{
						fp<<"<"<<(*it)->line_cur->id<<","<<(*it)->line_ref->id<<">,";
					}
					fp<<std::endl;

//					std::cout<<"nodes in the tree: ";
//					for(std::vector<Node_InterpTree_Line*>::iterator it=nodes.begin();it!=nodes.end();it++)
//					{
//						std::cout<<"<"<<(*it)->line_cur->id<<","<<(*it)->line_ref->id<<">,";
//					}
//					std::cout<<std::endl;

//					displayLine(insert_node->line_cur,insert_node->line_ref,vis);
//					vis->spin();
//
//					char ch;
//					std::cout<<"Press enter to continue...\n"<<std::endl;
//					ch=std::cin.get();
				}
				for(int i_node=nodes.size()-1;i_node>=0;i_node--)
				{
					flag_consistent=true;
					if(debug)
					{
						fp<<"node: "<<nodes[i_node]->line_cur->id<<","<<nodes[i_node]->line_ref->id<<std::endl;
					}

					node_attach=nodes[i_node];
					interp_node2=node_attach;
					
					interp_length=0;
					while(interp_node2!=root)
					{
						if(!flag_consistent)
							break;
						//interp_node2=*it_interp;
						if(interp_node2->line_ref==tmp_node->line_ref)
						{
							flag_consistent=false;
							break;
						}
						if(!consistent_2(tmp_node,interp_node2))
						{
							flag_consistent=false;
							break;
						}
						else
						{
							interp_node3=node_attach;
							while(interp_node3!=root)
							//for(std::vector<Node_InterpTree_Line*>::iterator ite_interp=Interp.begin();ite_interp!=Interp.end();ite_interp++)
							{
								if(interp_node3==interp_node2)
								{
									interp_node3=interp_node3->parent;
									continue;
								}
								//interp_node3=*ite_interp;
								if(!consistent_3(tmp_node,interp_node2,interp_node3))
								{
									flag_consistent=false;
									break;
								}
								interp_node3=interp_node3->parent;
							}
						}
						interp_length++;
						interp_node2=interp_node2->parent;
					}//end while interp_node2;
					if(flag_consistent)
					{
						if(debug)
						{
							fp<<"insert <"<<tmp_node->line_cur->id<<","<<tmp_node->line_ref->id<<"> to <"<<node_attach->line_cur->id<<","<<node_attach->line_ref->id<<">"<<std::endl;
						}
						insert_node=new Node_InterpTree_Line(*it_ref,*it_cur);
						node_attach->insertChild(insert_node);
						nodes_tmp.push_back(insert_node);
						if(debug)
						{
//							displayLineNode(insert_node,vis);
//							vis->spin();
//							removeLineNode(insert_node,vis);
						}
						//Interp.push_back(insert_node);
						if(interp_length>interp_length_max)
						{
							interp_length_max=interp_length;
							leaf_max_interp.resize(1);
							leaf_max_interp[0]=insert_node;
						}
						else if(interp_length==interp_length_max)
						{
							leaf_max_interp.push_back(insert_node);
						}
					}
				}// end for i_node;
				for(int i_node=0;i_node<nodes_tmp.size();i_node++)
				{
					nodes.push_back(nodes_tmp[i_node]);
				}
				nodes_tmp.clear();
			}// end for it_ref;
		}// end for it_cur;
		if(leaf_max_interp.size()==0)
		{
			for(size_t i=0;i<nodes.size();i++)
			{
				leaf_max_interp.push_back(nodes[i]);
			}
		}
		if(debug)
		{
			fp<<"final interp:"<<std::endl;
			for(size_t i=0;i<leaf_max_interp.size();i++)
			{
				fp<<i<<" - ";
				node_attach=leaf_max_interp[i];

//				if(false)
//				{
//					displayLineNode(node_attach,vis);
//					vis->spin();
//					removeLineNode(node_attach,vis);
//				}

				while(node_attach!=root)
				{
					fp<<"<"<<node_attach->line_cur->id<<","<<node_attach->line_ref->id<<">,";
					node_attach=node_attach->parent;
				}
				fp<<std::endl;
			}
		}
		delete tmp_node;
		fp.close();
		return true;
	}

	bool InterpTree_Line::isLeaf(Node_InterpTree_Line *node)
	{
		if(node->parent!=NULL && node->children.size()==0)
			return true;
		else
			return false;
	}

	bool InterpTree_Line::consistent_1(Node_InterpTree_Line *node)
	{
		if(debug)
		{
			fp<<"\tconsistent_1_dir: <"<<node->line_cur->id<<","<<node->line_ref->id<<"> "
									     <<node->line_ref->similarity_dir(node->line_cur)<<std::endl;
			fp<<"\tconsistent_1_normal: <"<<node->line_cur->id<<","<<node->line_ref->id<<"> "
									     <<node->line_ref->similarity_normal(node->line_cur)<<std::endl;
			fp<<"\tconsistent_1_dist : <"<<node->line_cur->id<<","<<node->line_ref->id<<"> "
								   	     <<node->line_ref->similarity_dist(node->line_cur)<<std::endl;
		}
		if(node->line_ref->similarity_dir(node->line_cur)<thres_dir && 
		   node->line_ref->similarity_normal(node->line_cur)<thres_normal && 
		   node->line_ref->similarity_dist(node->line_cur)<thres_dist)
			return true;
		else
			return false;
	}

	bool InterpTree_Line::consistent_2(Node_InterpTree_Line *node1, Node_InterpTree_Line *node2)
	{
		// angle between the two lines in the cur frame;
		double angle_cur=node1->line_cur->similarity_dir(node2->line_cur);
		// angle between the two lines in the ref frame;
		double angle_ref=node1->line_ref->similarity_dir(node2->line_ref);

		// angle between the normals of two lines in the cur frame;
		double normal_cur=node1->line_cur->similarity_normal(node2->line_cur);
		// angle between the normals of two lines in the ref frame;
		double normal_ref=node1->line_ref->similarity_normal(node2->line_ref);

		// distance between the two lines in the cur frame;
		double dist_cur=node1->line_cur->similarity_dist(node2->line_cur);
		// distance between the two lines in the ref frame;
		double dist_ref=node1->line_ref->similarity_dist(node2->line_ref);

		if(debug)
		{
			fp<<"\tconsistent_2_dir: <"<<node1->line_cur->id<<","<<node1->line_ref->id<<"> "
								   <<", <"<<node2->line_cur->id<<","<<node2->line_ref->id<<"> - "
								   <<angle_cur<<", "<<angle_ref<<"; "<<fabs(angle_cur-angle_ref)<<std::endl;
			fp<<"\tconsistent_2_dir: <"<<node1->line_cur->id<<","<<node1->line_ref->id<<"> "
								   <<", <"<<node2->line_cur->id<<","<<node2->line_ref->id<<"> - "
								   <<normal_cur<<", "<<normal_ref<<"; "<<fabs(normal_cur-normal_ref)<<std::endl;
			fp<<"\tconsistent_2_dir: <"<<node1->line_cur->id<<","<<node1->line_ref->id<<"> "
								   <<", <"<<node2->line_cur->id<<","<<node2->line_ref->id<<"> - "
								   <<dist_cur<<", "<<dist_ref<<"; "<<fabs(dist_cur-dist_ref)<<std::endl;
		}
		if(fabs(angle_cur-angle_ref)<thres_delta_dir && 
		   fabs(normal_cur-normal_ref)<thres_delta_normal && 
		   fabs(dist_cur-dist_ref)<thres_delta_dist)
			return true;
		else
			return false;
	}

	bool InterpTree_Line::consistent_3(Node_InterpTree_Line *node1, Node_InterpTree_Line *node2, Node_InterpTree_Line *node3)
	{
		return true;
	}


	bool LineMatching::match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		//fp<<"in the match:"<<interp_tree->getRoot()->children.size()<<std::endl;
		interp_tree->Construct(scan->scan_ref->lines_occluding, scan->lines_occluding, vis);
//		if(interp_tree->getMaxInterp()==0)
//			return false;
		LinePair tmp_line_pair;
		std::vector<Node_InterpTree_Line*> nodes_max=interp_tree->getMaxInterp();
		scan->line_matches.clear();
		for(size_t i=0;i<nodes_max.size();i++)
		{
			Node_InterpTree_Line *node_tmp=nodes_max[i];
			while(node_tmp!=interp_tree->getRoot())
			{
				tmp_line_pair.cur=node_tmp->line_cur;
				tmp_line_pair.ref=node_tmp->line_ref;
				node_tmp=node_tmp->parent;
				bool new_pair=true;
				for(size_t j=0;j<scan->line_matches.size();j++)
				{
					if(scan->line_matches[j].cur==tmp_line_pair.cur && scan->line_matches[j].ref==tmp_line_pair.ref)
					{
						new_pair=false;
						break;
					}
				}
				if(new_pair)
				{
					scan->line_matches.push_back(tmp_line_pair);
				}
			}
		}
		
		if(false)
		for(size_t i=0;i<scan->line_matches.size();i++)
		{
			Transform Trc=scan->scan_ref->Tcw*scan->Tcw.inv();
			Vector6d L;
			L.block<3,1>(0,0)=scan->line_matches[i].cur->u;
			L.block<3,1>(3,0)=scan->line_matches[i].cur->v;
			L=Trc.getLineTransform()*L;
			scan->line_matches[i].ref->u=L.block<3,1>(0,0);
			scan->line_matches[i].ref->v=L.block<3,1>(3,0);

//			std::cout<<"\t"<<scan->line_matches[i].ref->v.transpose()<<std::endl;
//			std::cout<<"\t"<<scan->line_matches[i].cur->v.transpose()<<std::endl;
//			Eigen::Matrix<double,1,1> tmp=scan->line_matches[i].cur->v.transpose()*scan->line_matches[i].ref->v;
//			std::cout<<"\t"<<tmp(0,0)<<std::endl;
//			if(tmp(0,0)<-1e-10)
//			{
//				scan->line_matches[i].cur->v=-scan->line_matches[i].cur->v;
//				scan->line_matches[i].cur->u=-scan->line_matches[i].cur->u;
//				std::cout<<"\t\t"<<scan->line_matches[i].ref->v.transpose()<<std::endl;
//				std::cout<<"\t\t"<<scan->line_matches[i].cur->v.transpose()<<std::endl;
//			}

//			// associate occluding lines with occluded lines in current frame;
//			Eigen::Vector3d u=scan->line_matches[i].cur->u;
//			Eigen::Vector3d v=scan->line_matches[i].cur->v;
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
//				if(min<thres_line2shadow)// && d>1.0)
//				{
//					scan->line_matches[i].cur->occluded=*idx_min;
//					if(debug) fp<<"\toccluded\t"<<scan->line_matches[i].cur->occluded->u.transpose()<<std::endl;
//				}
//			}

		}

		if(debug) visMatchedLines(scan,vis);

		return true;
	}

	void visMatchedLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);
		
		std::cout<<std::endl<<"visualize matched lines"<<std::endl;
		size_t i=0;
		for(std::vector<LinePair>::iterator it_line=scan->line_matches.begin();it_line!=scan->line_matches.end();it_line++)
		{
			std::cout<<"\n"<<(*it_line).cur->similarity_dir((*it_line).ref)<<"\t"
						   <<(*it_line).cur->similarity_normal((*it_line).ref)<<"\t"
						   <<(*it_line).cur->similarity_dist((*it_line).ref);
			std::cout<<"\ncur - line: red,";
			edge->resize((*it_line).cur->points.size()+(*it_line).ref->points.size());
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line).cur->points.begin();it!=(*it_line).cur->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=255;
				edge->at(j).g=0;
				edge->at(j).b=0;
				j++;
			}
			for(std::list<EdgePoint*>::iterator it=(*it_line).ref->points.begin();it!=(*it_line).ref->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=0;
				edge->at(j).g=0;
				edge->at(j).b=255;
				j++;
			}

//			plane1->resize(0);
//			plane2->resize(0);
//
//			if((*it_line).cur->occluded!=0)
//			{
//				Eigen::Vector3d u=(*it_line).cur->u;
//				Eigen::Vector3d u_pi=(*it_line).cur->occluded->u;
//				double ratio=fabs(u_pi.norm()/u.norm());
//				u.normalize();
//				u_pi.normalize();
//				double dist=u.transpose()*u_pi;
//				dist=fabs(dist);
//				dist=acos(dist)*180.0/M_PI;
//				std::cout<<"\tshadow: magenta - "<<dist<<","<<ratio;
//				edge->resize(edge->size()+(*it_line).cur->occluded->points.size());
//				for(std::list<EdgePoint*>::iterator it=(*it_line).cur->occluded->points.begin();
//													it!=(*it_line).cur->occluded->points.end();it++)
//				{
//					edge->at(j).x=(*it)->xyz(0);
//					edge->at(j).y=(*it)->xyz(1);
//					edge->at(j).z=(*it)->xyz(2);
//					edge->at(j).r=255;
//					edge->at(j).g=0;
//					edge->at(j).b=255;
//					j++;
//				}
//
//				if((*it_line).cur->occluded->plane!=0)
//				{
//					u=(*it_line).cur->occluded->u;
//					Eigen::Vector3d v=(*it_line).cur->occluded->v;
//					Eigen::Vector3d n=(*it_line).cur->occluded->plane->n;
//					double d=(*it_line).cur->occluded->plane->d;
//					Eigen::Vector3d tmp=u.cross(n)+d*v;
//					dist=tmp.norm();
//					std::cout<<"\tplane: yellow - "<<dist;
//					plane1->resize((*it_line).cur->occluded->plane->points.size());
//					for(int k=0;k<(*it_line).cur->occluded->plane->points.size();k++)
//					{
//						plane1->at(k).x=(*it_line).cur->occluded->plane->points[k].xyz(0);
//						plane1->at(k).y=(*it_line).cur->occluded->plane->points[k].xyz(1);
//						plane1->at(k).z=(*it_line).cur->occluded->plane->points[k].xyz(2);
//						plane1->at(k).r=255;
//						plane1->at(k).g=255;
//						plane1->at(k).b=0;
//					}
//				}
//			}

			std::cout<<"\nref - line: blue,";
//			if((*it_line).ref->occluded!=0)
//			{
//				Eigen::Vector3d u=(*it_line).ref->u;
//				Eigen::Vector3d u_pi=(*it_line).ref->occluded->u;
//				double ratio=fabs(u_pi.norm()/u.norm());
//				u.normalize();
//				u_pi.normalize();
//				double dist=u.transpose()*u_pi;
//				dist=fabs(dist);
//				dist=acos(dist)*180.0/M_PI;
//				std::cout<<"\tshadow: cyan - "<<dist<<","<<ratio;
//				edge->resize(edge->size()+(*it_line).ref->occluded->points.size());
//				for(std::list<EdgePoint*>::iterator it=(*it_line).ref->occluded->points.begin();
//													it!=(*it_line).ref->occluded->points.end();it++)
//				{
//					edge->at(j).x=(*it)->xyz(0);
//					edge->at(j).y=(*it)->xyz(1);
//					edge->at(j).z=(*it)->xyz(2);
//					edge->at(j).r=0;
//					edge->at(j).g=255;
//					edge->at(j).b=255;
//					j++;
//				}
//
//				if((*it_line).ref->occluded->plane!=0)
//				{
//					u=(*it_line).ref->occluded->u;
//					Eigen::Vector3d v=(*it_line).ref->occluded->v;
//					Eigen::Vector3d n=(*it_line).ref->occluded->plane->n;
//					double d=(*it_line).ref->occluded->plane->d;
//					Eigen::Vector3d tmp=u.cross(n)+d*v;
//					dist=tmp.norm();
//					std::cout<<"\tplane: orange - "<<dist;
//					plane2->resize((*it_line).ref->occluded->plane->points.size());
//					for(int k=0;k<(*it_line).ref->occluded->plane->points.size();k++)
//					{
//						plane2->at(k).x=(*it_line).ref->occluded->plane->points[k].xyz(0);
//						plane2->at(k).y=(*it_line).ref->occluded->plane->points[k].xyz(1);
//						plane2->at(k).z=(*it_line).ref->occluded->plane->points[k].xyz(2);
//						plane2->at(k).r=255; // orange
//						plane2->at(k).g=165;
//						plane2->at(k).b=0;
//					}
//				}
//
//			}
			std::cout<<std::endl;
			
			sprintf(id,"line");
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);

//			sprintf(id,"plane_cur");
//			if (!vis->updatePointCloud (plane1, id))
//				vis->addPointCloud (plane1, id);
//
//			sprintf(id,"plane_ref");
//			if (!vis->updatePointCloud (plane2, id))
//				vis->addPointCloud (plane2, id);

			i++;
			vis->spin();
		}
		
	}
}


