/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-11-06 09:02
#
# Filename:		relative_pose_error.h
#
# Description: 
#
************************************************/

#include "types/types.h"

namespace ulysses
{
	class RelativePoseError
	{
	public:
		RelativePoseError(const std::string &folder)
		{
			error_translation_cache=0;
			error_rotation_cache=0;
			filename=folder+"/rpe.txt";
//			remove("relative_pose_error.txt");
		}
		~RelativePoseError()
		{
			std::vector<double>().swap(time_stamps);
			std::vector<ulysses::Transform>().swap(groundtruth);
			std::vector<ulysses::Transform>().swap(estimates);
			std::vector<double>().swap(time_stamps_error);
			std::vector<Vector6d>().swap(errors);
			std::vector<double>().swap(errors_translation);
			std::vector<double>().swap(errors_rotation);
		}

		bool evaluate(const double &time_stamp, const double &time_interval, const Transform &gt, const Transform &Tcg); 
//								boost::shared_ptr<pcl::visualization::PCLPainter2D> fig);
		bool evaluate(const double &time_stamp, const Transform &gt, const Transform &Tcg); 
//								boost::shared_ptr<pcl::visualization::PCLPainter2D> fig); // each frame;
		
		void saveErrors(const std::string &file);

		double getCurrentErrorTrans() 
		{
			if(errors_translation.size()>0)
				return errors_translation[errors_translation.size()-1]; 
			else 
				return 0;
		}

	private:
//		const double time_interval;
		std::ofstream fp;
		std::string filename;

		double error_translation_cache;
		double error_rotation_cache;

		std::vector<double> time_stamps;
		std::vector<ulysses::Transform> groundtruth; // Tcw
		std::vector<ulysses::Transform> estimates; // Tcg

		std::vector<double> time_stamps_error;
		std::vector<Vector6d> errors;
		std::vector<double> errors_translation;
		std::vector<double> errors_rotation;

		void computeError(const int i, const double &time_stamp, const Transform &Tcw, const Transform &Tcg);
		void push(const double &time_stamp, const Transform &Tcw, const Transform &Tcg);
	};
}
