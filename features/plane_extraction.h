/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 16:49
#
# Filename:		plane_extraction.h
#
# Description: 
#
===============================================*/

#include "types/types.h"
#include "features/plane_extraction_base.h"

namespace ulysses
{

	struct Cell
	{
		Cell()
		{
			isEmpty=true;
			isBottom=true;
			avg_pps.setZero(3);
			avg_rgb.setZero(3);
			cov_pps.setZero(3,3);
			cov_rgb.setZero(3,3);
			indices = boost::make_shared<pcl::PointIndices>();
		}

		~Cell()
		{
			std::vector<Point> tmp;
			tmp.swap(points);
		}

		Eigen::Vector3d avg_pps, avg_rgb;
		Eigen::Matrix3d cov_pps, cov_rgb;
		void computeAttribute();
		
		bool isEmpty;
		bool isBottom;

		std::vector<Point> points;

		// indices w.r.t. point_cloud and normal_cloud;
		// to index the cell points in the point_cloud;
		pcl::PointIndices::Ptr indices;

		void clear() 
		{
			points.clear();
			indices->indices.clear();
			isEmpty=true;
			isBottom=true;
			avg_pps.setZero(3);
			avg_rgb.setZero(3);
			cov_pps.setZero(3,3);
			cov_rgb.setZero(3,3);
		}

		// the followings are used in the sting;
		//int layer;
		//bool relevant;
		//Cell *father;
		//Cell *child[8];
	};

	// Sorted_Cell
	// - store the index of the cells;
	// - used to sort the cells in Cells_bottom;
	struct Sorted_Cell
	{
		Sorted_Cell(){}
		bool operator < (const Sorted_Cell &m)const
		{
			return num_point < m.num_point;
		}
		int index;
		int num_point;
	};

	class Cells_bottom
	{
	public:

		Cells_bottom(int theta, int phy, int d) 
		{
			bins_theta=theta;
			bins_phy=phy;
			bins_d=d;
			delta_theta=M_PI/bins_theta;
			delta_phy=M_PI*2/bins_phy;
			delta_d=6.0/bins_d;
			// cells are allocated here;
			cells.resize(bins_theta*bins_phy*bins_d);
			for(size_t i=0;i<cells.size();i++)
				cells[i]=new Cell;
		}

		~Cells_bottom()
		{
			for(size_t i=0;i<cells.size();i++)
			{
				delete cells[i];
			}
			std::vector<Cell*> tmp;
			tmp.swap(cells);
			std::vector<Sorted_Cell> tmp2;
			tmp2.swap(sorted_cells);
		}

		void clear()
		{
			sorted_cells.clear();
			for(size_t i=0;i<cells.size();i++)
			{
				cells[i]->clear();
			}
//			cells.clear();
		}

		// push_point
		// - push the point point_tmp into the corresponding cell;
		// - idx: the index w.r.t. the point_cloud;
		void push_point(Point point_tmp, int idx);

		void computeCellAttributes();

		// SortCells
		// - sort the cells according to the number of inside points;
		// - store the sorting result in the sorted_cells;
		void SortCells();

		Cell* getCell(int i) {return cells[i];}
		Cell* getCell(int d, int theta, int phy) {return cells[index(d,theta,phy)];}

		int size() {return cells.size();}

		std::vector<Sorted_Cell>::iterator getHighestCell() {return sorted_cells.end()-1;}

	private:

		int index(int d, int theta, int phy) 
		{ return d*bins_theta*bins_phy+theta*bins_phy+phy; }

		int bins_theta,bins_phy,bins_d;
		double delta_theta,delta_phy,delta_d;

		std::vector<Cell*> cells;
		std::vector<Sorted_Cell> sorted_cells;
	};

	class PlaneExtraction : public PlaneExtractionBase
	{
	public:
		PlaneExtraction() {}

		PlaneExtraction(const std::string &settingFile) : PlaneExtractionBase(settingFile)
		{
			cells_bottom=new Cells_bottom(10,20,1);
			max_plane=99;

			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			min_plane_size=(int)settings["PlaneExtraction.min_plane_size"];
			thres_angle=(double)settings["PlaneExtraction.thres_angle"]*M_PI/180.0;
			thres_dist=(double)settings["PlaneExtraction.thres_dist"];
			thres_color=(double)settings["PlaneExtraction.thres_color"];

			std::cout<<"debug="<<debug<<std::endl;
			std::cout<<"min_plane_size="<<min_plane_size<<std::endl;
		}

		~PlaneExtraction()
		{
			delete cells_bottom;
		}

		void setDebug(bool d) {debug=d;}

		void extractPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		
	private:

		double maxdist_point2plane;
		int max_plane;
		int min_plane_size;
		double thres_angle, thres_dist, thres_color;

		Cells_bottom *cells_bottom;

		bool loadPoints(Scan *scan);

		void computeRotationPCA(Scan *scan);

		void unifyPlaneDir(pcl::ModelCoefficients::Ptr plane);

		double dist_point2plane(Eigen::Vector3d point, pcl::ModelCoefficients::Ptr plane);

		void fusePlanes(Plane *cur, Plane *fuse);
	};

}
