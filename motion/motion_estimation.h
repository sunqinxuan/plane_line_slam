/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-21 10:41
#
# Filename:		motion_estimation.h
#
# Description: 
#
===============================================*/

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "types/types.h"
#include "types/map.h"

namespace ulysses
{
//	void visScans(Scan *scan, Scan *scan_ref, Transform Tcr, 
//				 boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	class PlaneLineBA
	{
	public:
		PlaneLineBA() {}
		PlaneLineBA(Map *map);//, int num_frames_, bool use_plane_, bool use_line_, bool use_shadow_);
//		PlaneLineBA(std::vector<Scan*> scans, bool use_plane_=true, bool use_line_=true);
		~PlaneLineBA();

		void computePsi(const int &idx_camera, Matrix6d &Psi);

//		std::vector<Eigen::Vector3d> points_plane;
//		std::vector<Eigen::Vector3d> points_line;
//		std::vector<Eigen::Vector3d> points_shadow;

		int camera_block_size()		const { return 7;					}
		int plane_block_size()		const { return 4;					}
		int line_block_size()		const { return 6;					}
		int num_cameras()			const { return num_cameras_;		}
		int num_planes()			const { return num_planes_;			}
		int num_lines()				const { return num_lines_;			}
		int num_observ_planes()		const { return num_observ_planes_; 	}
		int num_observ_lines()		const { return num_observ_lines_;	}
		int num_observations()		const { return num_observations_;	}
		int num_parameters()		const { return num_parameters_;		}
		int feature_index(int i)	const { return feature_index_[i];	}
		int camera_index(int i)		const { return camera_index_[i];	}
//		const double* parameters()  const { return parameters_;			}
		const double* cameras() 	const { return parameters_.data();	}
		std::string features_id(int i) const {return features_id_[i];}
		double timestamps(int i) const {return timestamps_[i];}
//		int* feature_index_map()    const { return feature_index_map_; 	}
		double* observations() {return observations_.data();}
		double* sqrt_information() {return sqrt_information_.data();}
		double* mutable_cameras() {return parameters_.data();}
		double* mutable_planes() { return parameters_.data()+camera_block_size()*num_cameras_; }
		double* mutable_lines() { return parameters_.data()+camera_block_size()*num_cameras_ +plane_block_size()*num_planes_; }
//		int num_observ_shadows()		const { return num_observ_shadows_;	}
//		const int* shadow_camera_index()	const { return shadow_camera_index_;		}
//		const int* shadow_plane_index()		const { return shadow_plane_index_;			}
//		const int* shadow_line_index()		const { return shadow_line_index_;			}
//		double* observ_shadows() 		      { return observ_shadows_;		}
//		double* sqrt_information_shadows()	      { return sqrt_information_shadows_;	}

//		void usePlane(bool p) { use_plane_=p; }
//		void useLine(bool p)  { use_line_=p;  }
		

	private:

		int num_observ_planes_;
		int num_observ_lines_;
		int num_observations_;
		// the observation vector is laid out as:
		// [observ_plane_1, ..., observ_plane_N,
		//  observ_line_1, ..., observ_line_M];
		std::vector<double> observations_;
		std::vector<double> sqrt_information_;
		std::vector<int> camera_index_;
		std::vector<int> feature_index_;

//		int num_observ_shadows_;
//		// the observation vector is laid out as:
//		// [observ_shadow_1, ..., observ_shadow_M];
//		double* observ_shadows_;
//		double* sqrt_information_shadows_;
//		int* shadow_camera_index_;
//		int* shadow_plane_index_;
//		int* shadow_line_index_;

		int num_cameras_;
		int num_planes_;
		int num_lines_;
		int num_parameters_;
		// the parameter vector is laid out as:
		// [camera_1, ..., camera_n,
		//  plane_1, ..., plane_m,
		//  line_1, ..., line_k];
		// camera - Tcg
		// camera_i = [q_w,q_x,q_y,q_z,t_x,t_y,t_z];
//		double* parameters_;
		std::vector<double> parameters_;
		// size of feature_index_map_ is num_planes_+num_lines_;
		// index to the map->planes and map->lines;
//		int* feature_index_map_;
		std::vector<double> timestamps_;
		std::vector<std::string> features_id_;

//		std::vector<PlaneLM*> plane_map_;
//		std::vector<LineLM*> line_map_;
	};

	extern void displayPlaneLineBA(Scan *scan, PlaneLineBA *plane_line_ba, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	class PlaneError
	{
	public:
		PlaneError(const Eigen::Vector4d& pi, const Eigen::Matrix4d& sqrt_information)
			 : observed_pi_(pi), sqrt_information_(sqrt_information) {}

		// camera = [q_w,q_x,q_y,q_z,t_x,t_y,t_z];
		// plane  = [n1,n2,n3,d];
		// residual_ptr = [delta_n1,delta_n2,delta_n3,delta_d];
		template <typename T>
		bool operator()(const T* const camera, const T* const plane, T* residual_ptr) const 
		{
			Eigen::Quaternion<T> q_cg(camera[0],camera[1],camera[2],camera[3]);
			q_cg.normalize();
			Eigen::Matrix<T,3,1> t_cg(camera[4],camera[5],camera[6]);
//			Eigen::Map<const Eigen::Quaternion<T> > q_cr(q_ptr);
			Eigen::Map<const Eigen::Matrix<T,4,1> > pi(plane);
			Eigen::Map<Eigen::Matrix<T,4,1> > residual(residual_ptr);

			Eigen::Matrix<T,3,3> R_cg=q_cg.toRotationMatrix();
			Eigen::Matrix<T,4,4> T_cg; // transform for planes;
			T_cg.setIdentity();
			T_cg.template block<3,3>(0,0)=R_cg;
			T_cg.template block<1,3>(3,0)=-t_cg.transpose()*R_cg;

			Eigen::Matrix<T,4,1> nd;
			T n_norm=pi.template block<3,1>(0,0).norm();
			nd=pi/n_norm;

//			residual = observed_pi_.template cast<T>()-T_cg.template cast<T>()*pi.template cast<T>();
			residual = observed_pi_.template cast<T>()-T_cg.template cast<T>()*nd.template cast<T>();
			residual.applyOnTheLeft(sqrt_information_.template cast<T>());

//			std::ofstream fp;
//			fp.open("PlaneError.txt",std::ios::app);
//			fp<<"\npi\t\t"<<pi(0)<<"\t"<<pi(1)<<"\t"<<pi(2)<<"\t"<<pi(3)<<std::endl;
//			fp<<"residual\t"<<residual(0)<<"\t"<<residual(1)<<"\t"<<residual(2)<<"\t"<<residual(3)<<std::endl;
//			fp.close();

			return true;
		}

		static ceres::CostFunction* Create(const Eigen::Vector4d& pi, const Eigen::Matrix4d& sqrt_information)
		{
			return new ceres::AutoDiffCostFunction<PlaneError,4,7,4>
					(new PlaneError(pi,sqrt_information));
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Vector4d observed_pi_;
		const Eigen::Matrix4d sqrt_information_;
	};

	class PlaneErrorCostFunction : public ceres::SizedCostFunction<4,7,4>
	{
	public:
		PlaneErrorCostFunction(const Eigen::Vector4d& pi, const Eigen::Matrix4d& sqrt_information)
			: observed_pi_(pi), sqrt_information_(sqrt_information) {}

		virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobian) const;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Vector4d observed_pi_;
		const Eigen::Matrix4d sqrt_information_;
	};

	class LineError
	{
	public:
		LineError(const Vector6d& L, const Matrix6d& sqrt_information)
			: observed_L_(L), sqrt_information_(sqrt_information) {}

		template <typename T>
		bool operator()(const T* const camera, const T* const line, T* residual_ptr) const
		{
			Eigen::Quaternion<T> q_cg(camera[0],camera[1],camera[2],camera[3]);
			Eigen::Matrix<T,3,1> t_cg(camera[4],camera[5],camera[6]);
			Eigen::Map<const Eigen::Matrix<T,6,1> > L(line);
			Eigen::Map<Eigen::Matrix<T,6,1> > residual(residual_ptr);

			Eigen::Matrix<T,3,3> R_cg=q_cg.toRotationMatrix();
			Eigen::Matrix<T,3,3> skew_t;
			skew_t.setZero();
			skew_t(0,1)=-t_cg(2);
			skew_t(0,2)= t_cg(1);
			skew_t(1,0)= t_cg(2);
			skew_t(1,2)=-t_cg(0);
			skew_t(2,0)=-t_cg(1);
			skew_t(2,1)= t_cg(0);
			Eigen::Matrix<T,6,6> T_cg; // transform for lines;
			T_cg.setZero();
			T_cg.template block<3,3>(0,0)=R_cg;
			T_cg.template block<3,3>(3,3)=R_cg;
			T_cg.template block<3,3>(0,3)=skew_t*R_cg;

			Eigen::Matrix<T,6,1> L_tilde;
			T n_norm=L.template block<3,1>(3,0).norm();
			L_tilde=L/n_norm;

			Eigen::Matrix<T,6,1> L_trans=T_cg.template cast<T>()*L_tilde.template cast<T>();

//			Eigen::Matrix<T,1,1> tmp;
//			tmp=L_trans.template cast<T>().transpose()*observed_L_.template cast<T>();
//			if(tmp(0,0)<T(0.0)) L_trans=-L_trans.template cast<T>();

//			tmp=L_trans.template cast<T>().transpose()*observed_L_.template cast<T>();
//			std::cout<<"&&&&&&&&&&&&&& line\t"<<tmp(0,0)<<std::endl;

			residual = observed_L_.template cast<T>()-L_trans.template cast<T>();
			residual.applyOnTheLeft(sqrt_information_.template cast<T>());

//			std::ofstream fp;
//			fp.open("LineError.txt",std::ios::app);
//			fp<<"\nL\t\t"<<L(0)<<"\t"<<L(1)<<"\t"<<L(2)<<"\t"<<L(3)<<"\t"<<L(4)<<"\t"<<L(5)<<std::endl;
//			fp<<"residual\t"<<residual(0)<<"\t"<<residual(1)<<"\t"<<residual(2)<<"\t"<<residual(3)<<"\t"<<residual(4)<<"\t"<<residual(5)<<std::endl;
//			fp.close();

			return true;
		}

		static ceres::CostFunction* Create(const Vector6d& L, const Matrix6d& sqrt_information)
		{
			return new ceres::AutoDiffCostFunction<LineError,6,7,6>
					(new LineError(L,sqrt_information));
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Vector6d observed_L_;
		const Matrix6d sqrt_information_;
	};

//	class LineShadowError
//	{
//	public:
//		LineShadowError(const Vector6d& L, const Matrix6d& sqrt_information)
//			: observed_L_(L), sqrt_information_(sqrt_information) {}
//
//		template <typename T>
//		bool operator()(const T* const camera, const T* const plane, const T* const line, T* residual_ptr) const
//		{
//			Eigen::Quaternion<T> q_cg(camera[0],camera[1],camera[2],camera[3]);
//			Eigen::Matrix<T,3,1> t_cg(camera[4],camera[5],camera[6]);
//			Eigen::Map<const Eigen::Matrix<T,4,1> > pi(plane);
//			Eigen::Map<const Eigen::Matrix<T,6,1> > L(line);
//			Eigen::Map<Eigen::Matrix<T,6,1> > residual(residual_ptr);
//
//			Eigen::Matrix<T,3,3> R_cg=q_cg.toRotationMatrix();
//			Eigen::Matrix<T,3,1> t_gc=-R_cg.transpose()*t_cg;
//			Eigen::Matrix<T,3,1> u=L.template block<3,1>(0,0);
//			Eigen::Matrix<T,3,1> v=L.template block<3,1>(3,0);
//			Eigen::Matrix<T,3,1> n=pi.template block<3,1>(0,0);
//			T d=pi(3)/n.norm();
//			n.normalize();
//			u=u/v.norm();
//			v.normalize();
//			
//			// compute the shadow line (described in world frame);
//			Eigen::Matrix<T,6,1> L_pi;
//			Eigen::Matrix<T,3,1> uu=-d*(skew_sym(v)*t_gc+u)-n*t_gc.transpose()*u;
//			Eigen::Matrix<T,3,1> vv=-skew_sym(n)*skew_sym(v)*t_gc+skew_sym(u)*n;
//			d=uu.norm()/vv.norm();
//			uu.normalize();
//			vv.normalize();
//			L_pi.template block<3,1>(0,0)=d*uu;
//			L_pi.template block<3,1>(3,0)=vv;
////			// homogeneous coordinates of t_cg;
////			Eigen::Matrix<T,4,1> t_gc_homo;
////			t_gc_homo.template block<3,1>(0,0)=t_gc;
////			t_gc_homo(3)=T(1.0);
//			Eigen::Matrix<T,1,1> tmp;
//			tmp=u.template cast<T>().transpose()*uu.template cast<T>();
//			if(tmp(0,0)<T(0.0)) L_pi=-L_pi.template cast<T>();
//
//			// compute 6x6 transform for lines;
//			Eigen::Matrix<T,6,6> T_cg; // transform for lines;
//			T_cg.setZero();
//			T_cg.template block<3,3>(0,0)=R_cg;
//			T_cg.template block<3,3>(3,3)=R_cg;
//			T_cg.template block<3,3>(0,3)=skew_sym(t_cg)*R_cg;
//
////			Eigen::Matrix<T,4,4> L_mat_dual;
////			L_mat_dual.setZero();
////			L_mat_dual.template block<3,3>(0,0)=skew_sym(v);
////			L_mat_dual.template block<3,1>(0,3)=u;
////			L_mat_dual.template block<1,3>(3,0)=-u.transpose();
////			Eigen::Matrix<T,4,1> pi_L=L_mat_dual*t_gc_homo;
//
//			L_pi=T_cg.template cast<T>()*L_pi.template cast<T>();
//
//
////			tmp=L_pi.template cast<T>().transpose()*observed_L_.template cast<T>();
////			std::cout<<"&&&&&&&&&&&&&& shadow\t"<<tmp(0,0)<<std::endl;
//
////			Eigen::Matrix<T,1,1> tmp;
////			tmp=L_pi.transpose()*observed_L_;
////			if(tmp(0,0)<T(0)) L_pi=-L_pi.template cast<T>();
//
////			std::cout<<"&&&&&&&&&&&&&&\t"<<L_pi.transpose()*observed_L_<<std::endl;
//
//			residual = observed_L_.template cast<T>()-L_pi.template cast<T>();
//			residual.applyOnTheLeft(sqrt_information_.template cast<T>());
//
//			return true;
//		}
//
//		static ceres::CostFunction* Create(const Vector6d& L, const Matrix6d& sqrt_information)
//		{
//			return new ceres::AutoDiffCostFunction<LineShadowError,6,7,4,6>
//					(new LineShadowError(L,sqrt_information));
//		}
//
//		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//	private:
//		const Vector6d observed_L_;
//		const Matrix6d sqrt_information_;
//
//		template <typename T>
//		const Eigen::Matrix<T,3,3> skew_sym(Eigen::Matrix<T,3,1> u) const
//		{
//			Eigen::Matrix<T,3,3> skew_u;
//			skew_u.setZero();
//			skew_u(0,1)=-u(2);
//			skew_u(0,2)= u(1);
//			skew_u(1,0)= u(2);
//			skew_u(1,2)=-u(0);
//			skew_u(2,0)=-u(1);
//			skew_u(2,1)= u(0);
//			return skew_u;
//		}
//	};

	class LineLocalParameterization : public ceres::LocalParameterization 
	{
	public:
		virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
		virtual bool ComputeJacobian(const double *x, double *jacobian) const;
		virtual int GlobalSize() const { return 6; }
		virtual int LocalSize() const { return 4; }

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
//		double w11,w21;
//		Eigen::Vector3d u1,u2,u3;
	};

	class MotionEstimation
	{
	public:

		MotionEstimation() {}

		MotionEstimation(const std::string &settingFile)
		{ 
			remove("motion_estimation.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			settings["debug"]>>debug; 
			settings["feature_constant"]>>feature_constant; 
			settings["max_iterations"]>>max_iterations; 
			settings.release();
//			use_plane_  = (double)settings["MotionEstimationLine.use_plane"];
//			use_line_   = (double)settings["MotionEstimationLine.use_line"];
////			use_shadow_ = (double)settings["MotionEstimationLine.use_shadow"];
//			use_line_residual_=(double)settings["MotionEstimationLine.use_line_res"];
		}

		~MotionEstimation() {}
		
//		void setDebug(bool d) {debug=d;}
//		void usePlane(bool p) { use_plane_=p; }
//		void useLine(bool p)  { use_line_=p;  }
//		void useShadow(bool p) { use_shadow_=p; }
//		void useLineResidual(bool p)  { use_line_residual_=p;  }

//		bool alignOccludingLines(Scan *scan, Transform &Tcr);
		bool estimate(Map *map);
		// for debugging
//		int observ_planes,observ_lines;//,observ_shadows;
		
	private:
		
		bool debug;
		std::ofstream fp;
		bool feature_constant;
		int max_iterations;

//		bool use_plane_,use_line_,use_shadow_;
//		bool use_line_residual_;

//		PlaneLineBA *plane_line_ba;

		void buildProblem(PlaneLineBA* plane_line_ba, ceres::Problem* problem);

//		Vector6d computeLineShadow(Vector6d l, Eigen::Vector4d pi, Transform Tcg);

	};
}

