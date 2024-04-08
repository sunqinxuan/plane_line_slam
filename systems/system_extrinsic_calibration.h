/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-08-05 14:28
#
# Filename:		system_extrinsic_calibration.h
#
# Description: 
#
************************************************/
#include "types/types.h"
//#include "system_plane_line_shadow.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace ulysses
{
	struct Pose
	{
		Pose() {valid=true;}
		Pose(Transform p) : pose(p) {valid=true;}
		Pose(double t, Transform p) : time_stamp(t), pose(p) {valid=true;}
		Pose(double t, std::string f) : time_stamp(t), filename(f) {valid=true;}
//		Pose(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4) : pt1(p1), pt2(p2), pt3(p3), pt4(p4) {}

		double time_stamp;
		std::string filename; // rgb/${time_stamp}.png
		Transform pose;
		bool valid;
	};

	struct CalibMeasure
	{
		CalibMeasure(const Transform &T) : T_tag_marker2(T) {}
		CalibMeasure(const Transform &T, const std::string &idx) : T_tag_marker2(T), cur_idx(idx) {}

		std::string cur_idx;

		Transform T_tag_sensor, T_mocap_marker, T_mocap_marker2;
		const Transform T_tag_marker2;

		std::vector<Pose> Ts_tag_sensor, Ts_mocap_marker, Ts_mocap_marker2;
	};


	class SystemExtrinsicCalibration// : public SystemPlaneLineShadow
	{
	public:
		SystemExtrinsicCalibration(const std::string &seq_path, const std::string &setting_path)
			: seq_path(seq_path), setting_path(setting_path) {}

		Transform calibrate(const std::string &mode);
		void outputPoses();

		void shutDown();

		void visTraj(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		void visChessCalib(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		void visStatic(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		
	private:
		std::string seq_path, setting_path;

		std::vector<CalibMeasure> calib_measures;
		
		std::vector<Pose> T_tag_sensor; // Tac
		std::vector<Pose> T_mocap_marker; // Twm
		std::vector<Pose> T_mocap_marker2; // Twm2

		std::vector<Transform> motion_sensor;
		std::vector<Transform> motion_marker;
		std::vector<double> motion_weight;

		std::vector<Pose> calib_matrices;
		Transform calib_sensor_marker; // Tcm

		void readFile(const std::string &association_file);
		void readFile_chess();
		void readFile_static(CalibMeasure &calib_measure);
		void storeCalibration();

		void calibrate_static();
		Transform averageTransform(const std::vector<Pose> &transforms);

		void generateMotions();
		void computeRotation();
		void computeTranslation();

		bool computeRotation_test();

		Transform T_align; //T_AW
		void ATE();
		void RPE();

		void simulateCameraPoses();

		void chessLocalization();
		void chessLocalization(CalibMeasure &calib_measure);
		Transform Mat2Trans(cv::Mat r, cv::Mat t);

		Transform T_color_depth; // Tcd
		Transform calib_depth_marker; // Tdm
	
	};
}
