/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-10-17 22:13
#
# Filename:		plane_param_estimation.h
#
# Description: 
#
===============================================*/

#include "types/types.h"

namespace ulysses
{
	class PlaneParamEstimation
	{
	public:

		PlaneParamEstimation() 
		{
			remove("plane_param_estimation.txt");
			pln_fitting_method=0;
		}
		~PlaneParamEstimation() {}

		void setDebug(bool d) {debug=d;}
		void setPlnFittingMethod(int i) {pln_fitting_method=i;}

		void estimatePlaneParams(Plane *plane, IntrinsicParam cam);

	private:
		
		bool debug;
		std::ofstream fp;
		int pln_fitting_method;

		void compute_point_weight(Point &point, Eigen::Vector3d n, double d, IntrinsicParam cam);

		void compute_plane_centroid(Plane *plane);

		void compute_plane_cov_inv(Plane *plane);

		void compute_scatter_matrix(Plane *plane);
	};
}

