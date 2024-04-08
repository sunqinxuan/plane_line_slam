/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-06-16 13:28
#
# Filename:		plane_matching.cpp
#
# Description: 
#
===============================================*/
#include "features/plane_matching.h"

namespace ulysses
{
	// construct interpretation tree from two plane set
	// each path down to the leaf represent a hypothesis
	// *************************************************
	// consider: the circumstances returning false
	bool InterpTree_Plane::Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_)
	{
//		std::cout<<"Construct: debug="<<debug<<std::endl;
		fp.open("plane_matching.txt",std::ios::app);
		if(debug)
			fp<<"**************************************************************"<<std::endl;
		// destruct the existing interpretation tree
		Release();
		root=new Node_InterpTree_Plane;
		Node_InterpTree_Plane *tmp_node, *insert_node, *node_attach, *interp_node2, *interp_node3;
		std::vector<Node_InterpTree_Plane*> nodes_tmp;
		bool flag_consistent=true;
		tmp_node=new Node_InterpTree_Plane();
		int interp_length_max=0, interp_length;
//		leaf_max_interp=root;
		//fp<<"nodes:"<<nodes.size()<<std::endl;
		if(debug)
		{
			fp<<"plane_cur:"<<planes_cur_.size()<<std::endl;
			for(int i=0;i<planes_cur_.size();i++)
			{
				fp<<planes_cur_[i]->id<<", "<<planes_cur_[i]->n.transpose()<<", "<<planes_cur_[i]->d<<std::endl;
			}
			fp<<"plane_ref:"<<planes_ref_.size()<<std::endl;
			for(int i=0;i<planes_ref_.size();i++)
			{
				fp<<planes_ref_[i]->id<<", "<<planes_ref_[i]->n.transpose()<<", "<<planes_ref_[i]->d<<std::endl;
			}
		}

		// for any two planes out of planes_cur_ and planes_ref_
		for(std::vector<Plane*>::iterator it_cur=planes_cur_.begin();it_cur!=planes_cur_.end();it_cur++)
		{
			for(std::vector<Plane*>::iterator it_ref=planes_ref_.begin();it_ref!=planes_ref_.end();it_ref++)
			{
				tmp_node->plane_cur=*it_cur;
				tmp_node->plane_ref=*it_ref;
				if(debug)
				{
					fp<<std::endl<<"now in: <"<<tmp_node->plane_cur->id<<","<<tmp_node->plane_ref->id<<"> "<<std::endl;
				}
				if(!consistent_1(tmp_node))
					continue;
				// it has to be ensured that the plane id 0 in planes_cur_ must have a correspondence in planes_ref_
				// and there consistent_1 must be satisfied
				// otherwise there will be no node insert to the root
				//if(it_cur==planes_cur_.begin())
				{
					insert_node=new Node_InterpTree_Plane(*it_ref,*it_cur);
					root->insertChild(insert_node);
					nodes.push_back(insert_node);
					if(debug)
					{
						fp<<"insert <"<<insert_node->plane_cur->id<<","<<insert_node->plane_ref->id<<"> to root"<<std::endl;
					}
					//continue;
				}
				if(debug)
				{
					fp<<"nodes in the tree: ";
					for(std::vector<Node_InterpTree_Plane*>::iterator it=nodes.begin();it!=nodes.end();it++)
					{
						fp<<"<"<<(*it)->plane_cur->id<<","<<(*it)->plane_ref->id<<">,";
					}
					fp<<std::endl;
				}
				for(int i_node=nodes.size()-1;i_node>=0;i_node--)
				{
					flag_consistent=true;
					if(debug)
					{
						fp<<"node: "<<nodes[i_node]->plane_cur->id<<","<<nodes[i_node]->plane_ref->id<<std::endl;
					}

					node_attach=nodes[i_node];
					interp_node2=node_attach;
					
					interp_length=0;
					while(interp_node2!=root)
					{
						if(!flag_consistent)
							break;
						//interp_node2=*it_interp;
						if(interp_node2->plane_ref==tmp_node->plane_ref)
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
							//for(std::vector<Node_InterpTree_Plane*>::iterator ite_interp=Interp.begin();ite_interp!=Interp.end();ite_interp++)
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
							fp<<"insert <"<<tmp_node->plane_cur->id<<","<<tmp_node->plane_ref->id<<"> to <"<<node_attach->plane_cur->id<<","<<node_attach->plane_ref->id<<">"<<std::endl;
						}
						insert_node=new Node_InterpTree_Plane(*it_ref,*it_cur);
						node_attach->insertChild(insert_node);
						nodes_tmp.push_back(insert_node);
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
				while(node_attach!=root)
				{
					fp<<"<"<<node_attach->plane_cur->id<<","<<node_attach->plane_ref->id<<">,";
					node_attach=node_attach->parent;
				}
				fp<<std::endl;
			}
		}
		delete tmp_node;
		fp.close();
		return true;
	}

	bool InterpTree_Plane::isLeaf(Node_InterpTree_Plane *node)
	{
		if(node->parent!=NULL && node->children.size()==0)
			return true;
		else
			return false;
	}

	bool InterpTree_Plane::consistent_1(Node_InterpTree_Plane *node)
	{
		if(debug)
		{
			fp<<"\tconsistent_1_color: <"<<node->plane_cur->id<<","<<node->plane_ref->id<<"> "
									     <<node->plane_ref->similarity_color(node->plane_cur)<<std::endl;
			fp<<"\tconsistent_1_angle: <"<<node->plane_cur->id<<","<<node->plane_ref->id<<"> "
								   	     <<node->plane_ref->similarity_angle(node->plane_cur)<<std::endl;
			fp<<"\tconsistent_1_dist : <"<<node->plane_cur->id<<","<<node->plane_ref->id<<"> "
								   	     <<node->plane_ref->similarity_dist(node->plane_cur)<<std::endl;
			fp<<"\tconsistent_1_size_rate: <"<<node->plane_cur->id<<","<<node->plane_ref->id<<"> "<<double(node->plane_ref->inliers.indices.size())/double(node->plane_cur->inliers.indices.size())<<std::endl;
		}
		if(node->plane_ref->similarity_color(node->plane_cur)<thres_color && 
		   node->plane_ref->similarity_angle(node->plane_cur)<thres_delta_angle &&
		   node->plane_ref->similarity_dist(node->plane_cur)<thres_delta_d &&
		   double(node->plane_ref->inliers.indices.size())/double(node->plane_cur->inliers.indices.size())>0.5 &&
		   double(node->plane_ref->inliers.indices.size())/double(node->plane_cur->inliers.indices.size())<2.0 )
			return true;
		else
			return false;
	}

	bool InterpTree_Plane::consistent_2(Node_InterpTree_Plane *node1, Node_InterpTree_Plane *node2)
	{
		double cos_angle_cur, cos_angle_ref, angle_cur, angle_ref, dist_cur, dist_ref;
		cos_angle_cur=node1->plane_cur->n.transpose()*node2->plane_cur->n;
		if(cos_angle_cur>0.9999)
			cos_angle_cur=0.9999;
		cos_angle_ref=node1->plane_ref->n.transpose()*node2->plane_ref->n;
		if(cos_angle_ref>0.9999)
			cos_angle_ref=0.9999;
		angle_cur=acos(cos_angle_cur);
		angle_ref=acos(cos_angle_ref);
		if(angle_cur<thres_delta_angle)
		{
			dist_cur=fabs(node1->plane_cur->d-node2->plane_cur->d);
		}
		else
		{
			dist_cur=0;
		}
		if(angle_ref<thres_delta_angle)
		{
			dist_ref=fabs(node1->plane_ref->d-node2->plane_ref->d);
		}
		else
		{
			dist_ref=0;
		}
		if(debug)
		{
			fp<<"\tconsistent_2_added: <"<<node1->plane_cur->id<<","<<node1->plane_ref->id<<"> "
								   <<", <"<<node2->plane_cur->id<<","<<node2->plane_ref->id<<"> - "
								   <<cos_angle_cur<<", "<<cos_angle_ref<<"; "
								   <<angle_cur<<", "<<angle_ref<<"; "<<dist_cur<<", "<<dist_ref<<std::endl;
			fp<<"\tconsistent_2: <"<<node1->plane_cur->id<<","<<node1->plane_ref->id<<"> "
								   <<", <"<<node2->plane_cur->id<<","<<node2->plane_ref->id<<"> - "
								   <<fabs(angle_cur-angle_ref)<<", "<<fabs(dist_cur-dist_ref)<<std::endl;
		}
		if(fabs(angle_cur-angle_ref)<thres_delta_angle && fabs(dist_cur-dist_ref)<thres_delta_d)
			return true;
		else
			return false;
	}

	bool InterpTree_Plane::consistent_3(Node_InterpTree_Plane *node1, Node_InterpTree_Plane *node2, Node_InterpTree_Plane *node3)
	{
		return true;
	}


	bool PlaneMatching::match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		//fp<<"in the match:"<<interp_tree->getRoot()->children.size()<<std::endl;
		interp_tree->Construct(scan->scan_ref->observed_planes, scan->observed_planes);
//		if(interp_tree->getMaxInterp()==0)
//			return false;
		PlanePair tmp_plane_pair;
		std::vector<Node_InterpTree_Plane*> nodes_max=interp_tree->getMaxInterp();
		scan->plane_matches.clear();
		for(size_t i=0;i<nodes_max.size();i++)
		{
			Node_InterpTree_Plane *node_tmp=nodes_max[i];
			while(node_tmp!=interp_tree->getRoot())
			{
				tmp_plane_pair.cur=node_tmp->plane_cur;
				tmp_plane_pair.ref=node_tmp->plane_ref;
				node_tmp=node_tmp->parent;
				bool new_pair=true;
				for(size_t j=0;j<scan->plane_matches.size();j++)
				{
					if(scan->plane_matches[j].cur==tmp_plane_pair.cur && scan->plane_matches[j].ref==tmp_plane_pair.ref)
					{
						new_pair=false;
						break;
					}
				}
				if(new_pair)
				{
					scan->plane_matches.push_back(tmp_plane_pair);
				}
			}
		}

		if(false)
		for(int i=0;i<scan->plane_matches.size();i++)
		{
			Transform Trc=scan->scan_ref->Tcw*scan->Tcw.inv();
			Eigen::Vector4d pi;
			pi.block<3,1>(0,0)=scan->plane_matches[i].cur->n;
			pi(3)=scan->plane_matches[i].cur->d;
			pi=Trc.getPlaneTransform()*pi;
			scan->plane_matches[i].ref->n=pi.block<3,1>(0,0);
			scan->plane_matches[i].ref->d=pi(3);
		}

		if(debug) visMatchedPlanes(scan,vis);

		return true;
	}

	//void plane_feature_matching::depthFirstSearch()
	//{
	//	//fp<<"node num:"<<interp_tree->getNodeNum()<<std::endl;
	//	std::stack<Node_InterpTree_Plane*> nodeStack;
	//	nodeStack.push(interp_tree->getRoot());
	//	Node_InterpTree_Plane *node;
	//	fp<<"depthFirstSearch: ";
	//	while(!nodeStack.empty())
	//	{
	//		//fp<<"here"<<std::endl;
	//		node = nodeStack.top();
	//		nodeStack.pop();
	//		for(std::vector<Node_InterpTree_Plane*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeStack.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			fp<<"<"<<node->plane_cur->id<<","<<node->plane_ref->id<<">("<<node->layer<<"); ";
	//		}
	//	}
	//	fp<<std::endl;
	//}

	//void plane_feature_matching::breadthFirstTravel()
	//{
	//	std::queue<Node_InterpTree_Plane*> nodeQueue;
	//	nodeQueue.push(interp_tree->getRoot());
	//	Node_InterpTree_Plane *node;
	//	fp<<"breadthFirstTravel:";
	//	while(!nodeQueue.empty())
	//	{
	//		node = nodeQueue.front();
	//		nodeQueue.pop();
	//		for(std::vector<Node_InterpTree_Plane*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeQueue.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			fp<<"<"<<node->plane_cur->id<<","<<node->plane_ref->id<<">; ";
	//		}
	//	}
	//	fp<<std::endl;
	//}

	// segmented planar regions;
	void visMatchedPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

		std::cout<<"\nvisualize matched planes"<<std::endl;
		for(int i=0;i<scan->plane_matches.size();i++)
		{
			std::cout<<"\t"<<i<<"cur - red"<<std::endl;
			sprintf(id,"cur_plane");
			plane->resize(scan->plane_matches[i].cur->inliers.indices.size());
			for(int j=0;j<scan->plane_matches[i].cur->inliers.indices.size();j++)
			{
				int idx=scan->plane_matches[i].cur->inliers.indices[j];
				plane->at(j).x=scan->point_cloud->at(idx).x;
				plane->at(j).y=scan->point_cloud->at(idx).y;
				plane->at(j).z=scan->point_cloud->at(idx).z;
				plane->at(j).r=255;
                plane->at(j).g=0;
                plane->at(j).b=0;
			}
			if (!vis->updatePointCloud (plane, id))
				vis->addPointCloud (plane, id);

			std::cout<<"\t"<<i<<"ref - blue"<<std::endl;
			sprintf(id,"ref_plane");
			plane->resize(scan->plane_matches[i].ref->inliers.indices.size());
			for(int j=0;j<scan->plane_matches[i].ref->inliers.indices.size();j++)
			{
				int idx=scan->plane_matches[i].ref->inliers.indices[j];
				plane->at(j).x=scan->scan_ref->point_cloud->at(idx).x;
				plane->at(j).y=scan->scan_ref->point_cloud->at(idx).y;
				plane->at(j).z=scan->scan_ref->point_cloud->at(idx).z;
				plane->at(j).r=0;
                plane->at(j).g=0;
                plane->at(j).b=255;
			}
			if (!vis->updatePointCloud (plane, id))
				vis->addPointCloud (plane, id);

			vis->spin();
		}

	}


	/*
	void visMatchedPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

		std::cout<<"\nvisualize matched planes"<<std::endl;
		for(int i=0;i<scan->plane_matches.size();i++)
		{
			std::cout<<"\t"<<i<<"cur - red"<<std::endl;
			sprintf(id,"cur_plane");
			plane->resize(scan->plane_matches[i].cur->points.size());
			for(int j=0;j<scan->plane_matches[i].cur->points.size();j++)
			{
				plane->at(j).x=scan->plane_matches[i].cur->points[j].xyz[0];
				plane->at(j).y=scan->plane_matches[i].cur->points[j].xyz[1];
				plane->at(j).z=scan->plane_matches[i].cur->points[j].xyz[2];
				plane->at(j).r=255;
                plane->at(j).g=0;
                plane->at(j).b=0;
			}
			if (!vis->updatePointCloud (plane, id))
				vis->addPointCloud (plane, id);

			std::cout<<"\t"<<i<<"ref - blue"<<std::endl;
			sprintf(id,"ref_plane");
			plane->resize(scan->plane_matches[i].ref->points.size());
			for(int j=0;j<scan->plane_matches[i].ref->points.size();j++)
			{
				plane->at(j).x=scan->plane_matches[i].ref->points[j].xyz[0];
				plane->at(j).y=scan->plane_matches[i].ref->points[j].xyz[1];
				plane->at(j).z=scan->plane_matches[i].ref->points[j].xyz[2];
				plane->at(j).r=0;
                plane->at(j).g=0;
                plane->at(j).b=255;
			}
			if (!vis->updatePointCloud (plane, id))
				vis->addPointCloud (plane, id);

			vis->spin();
		}

	}
	*/

}


