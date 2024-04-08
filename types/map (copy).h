/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-11-18 09:01
#
# Filename:		map.h
#
# Description: 
#
************************************************/

#ifndef _MAP_H_
#define _MAP_H_

#include "types/types.h"
#include "LBD/LineDescriptor.hh"
#include "LBD/PairwiseLineMatching.hh"

namespace ulysses
{
	typedef std::map<std::string,Feature*>::iterator iterFeature;
	typedef std::map<std::string,Feature*>::const_iterator const_iterFeature;

	typedef std::map<std::string,Landmark*>::iterator iterLandmark;
	typedef std::map<std::string,Landmark*>::const_iterator const_iterLandmark;

	typedef std::map<double,Transform>::iterator iterCamera;
	typedef std::map<double,Transform>::const_iterator const_iterCamera;

	typedef std::map<double,Scan*>::iterator iterScan;
	typedef std::map<double,Scan*>::const_iterator const_iterScan;

	class Scan;
	class Map;
	class LandmarkAssociation;

	extern CameraIntrinsic camera_intrinsic;

	class FeatureAssociation
	{
	public:

		FeatureAssociation() {}
		FeatureAssociation(Map *map, Scan* scan_cur, Scan* scan_ref);
		~FeatureAssociation() {}
		
		void insert(Feature *cur, Feature *ref, Landmark *lm);
		void insert(Feature *cur, Feature *ref, Transform Trg);
		void insert(Feature *cur, Feature *ref);

		std::string findFeature(std::string id);
		std::string findLandmark(std::string id);

		void evalulatePR(FeatureAssociation *truth, double &precision, double &recall);
		void evalulatePR(double ref_time, LandmarkAssociation *truth, double &precision, double &recall);
		void evalulatePR(double cur_time, double ref_time, Map *map, double &precision, double &recall);

		void print(std::ostream &os) const;
		void load(const std::string &folder, Scan *scan, Map *map);
		void vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v);

		int size() {return features_cur.size();}
		Landmark* getLandmark(int i) {return landmarks[i];}
		Feature* getFeature(int i) {return features_cur[i];}
		Feature* getFeatureRef(int i) {return features_ref[i];}

	private:

		std::vector<Feature*> features_cur;
		std::vector<Feature*> features_ref;
		std::vector<Landmark*> landmarks;
		std::map<std::string,int> indices;

	};

	class LandmarkAssociation
	{
	public:

		LandmarkAssociation() {}
		~LandmarkAssociation() {}

		void insert(Feature *cur, Landmark *lm);
		void fillIndicesMap();
		
		std::string findLandmark(std::string id_feature);
		std::string findFeature(std::string id_landmark);
		
		void evalulatePR(LandmarkAssociation *truth, double &precision, double &recall);
		void evalulatePR(double time, Map *map, double &precision, double &recall);

		void print(std::ostream &os) const;
		void load(const std::string &folder, Scan *scan, Map *map);
		void vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v);

		int size() {return features_cur.size();}
		Landmark* getLandmark(int i) const {return landmarks[i];}
		Landmark* getLandmark(std::string id) {return landmarks[indices_map.find(id)->second];}
		Feature* getFeature(int i) {return features_cur[i];}

	private:

		std::vector<Feature*> features_cur;
		std::vector<Landmark*> landmarks;
		std::map<std::string,int> indices_feature;
		std::map<std::string,int> indices_map;

	};


	class Map
	{
	public:

		~Map();
		
		int sizeCamera() const {return cameras.size();}
		const_iterCamera beginCamera() const {return cameras.begin();}
		const_iterCamera endCamera() const {return cameras.end();}
		Transform camera(double time) const;
		void updateCamera(double time, const Transform &T);

		int sizeLandmark() const {return landmarks.size();}
		const_iterLandmark beginLandmark() const {return landmarks.begin();}
		const_iterLandmark endLandmark() const {return landmarks.end();}
		Landmark* findLandmark(std::string id);
		void clearLandmark() {landmarks.clear();}

		Scan* scan(double time) const;
		const_iterScan beginScan() const {return scans.begin();}
		const_iterScan endScan() const {return scans.end();}
		void clearScan() {scans.clear();}

		void addScan(Scan *scan, bool firstFr=false, bool truth=false);
		void addCamera(Transform Tcg, double timestamp);
		void addLandmark(Landmark *lm, const double &time, const std::string &observ);
		void addAssociation(FeatureAssociation *fa, double time);
		void addAssociation(LandmarkAssociation *la, double time);

		
//		// index of the cameras 
//		// to the planes and lines;
//		std::vector<Indices> indices_planes;
//		std::vector<Indices> indices_lines;
//
//		struct observation_plane
//		{
//			Eigen::Vector4d pi; // pi=[n,d];
//			Eigen::Matrix4d sqrt_info;
//			int idx_camera;
//			int idx_plane;
//			Plane *plane_ptr;
//			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//			Matrix6d Psi;
//		};
//		std::vector<observation_plane*> observ_planes;
//		void addObservationPlane(Plane *p, int idx_camera, int idx_plane);
//		void computePsiPlane(observation_plane* plane);
//
//		struct observation_line
//		{
//			Vector6d L;
//			Matrix6d sqrt_info;
//			int idx_camera;
//			int idx_line;
//			Line *line_ptr;
//			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//			Matrix6d Psi;
//			double weight;
//			double scale;
//		};
//		std::vector<observation_line*> observ_lines;
//		void addObservationLine(Line *l, int idx_camera, int idx_line);
//		void computePsiLine(observation_line* line);

//		struct observation_shadow
//		{
//			Vector6d L;
//			Matrix6d sqrt_info;
//			Matrix6d cov;
//			int idx_camera;
//			int idx_plane;
//			int idx_line;
//			//Line *line_ptr;
//			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//			Matrix6d Psi;
//			double weight;
//			double scale;
//		};
//		std::vector<observation_shadow*> observ_shadows;
//		void addObservationShadow(Line *l, Plane *p, int idx_camera);
//		void computePsiShadow(observation_shadow* line);

		void printCameras(std::ostream &os) const;
		void printLandmarks(std::ostream &os) const;
		void save(std::string folder) const;
		void load(std::string folder);
		void saveTimes(const std::string &folder) const;
		void vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v);

	private: 

		std::map<double,Scan*> scans;
		std::map<double,Transform> cameras;
		std::map<std::string,Landmark*> landmarks;

		void saveTrajSLAM(const std::string &filename) const;
		void saveTrajVO(const std::string &filename) const;
		void saveTraj(const std::string &filename) const;
		void saveMap(const std::string &filename) const;
		void loadTraj(const std::string &filename);
		void loadMap(const std::string &filename);
	};


	class Scan
	{
	public:
		Scan() {feature_association=0;landmark_association=0;}
		Scan(const double &time) : time_stamp(time) {feature_association=0;landmark_association=0;}
		~Scan();
		void release();

		std::string filename_rgb;

		Scan*& ref() {return scan_ref;}
		double& time() {return time_stamp;}

		Transform& Tcg() {return T_cg;}
		Transform& Tcr() {return T_cr;}
		Transform& Tcw() {return T_cw;}
		void localize() {T_cg=T_cr*scan_ref->Tcg();}

		FeatureAssociation*& association() {return feature_association;}
		LandmarkAssociation*& association_map() {return landmark_association;}
//		FeatureAssociation*& association_sim() {return feature_association_sim;}

		void addFeature(Feature *f);
		Feature* findFeature(std::string id);

		int sizeFeature() {return features.size();}
		iterFeature beginFeature() {return features.begin();}
		iterFeature endFeature() {return features.end();}
		iterFeature eraseFeature(iterFeature it) {return features.erase(it);}
		iterFeature eraseFeature(std::string id);

		void printFeatures(std::ostream &os);
		void saveFeatures(const std::string &folder);
		void saveAssociations(const std::string &folder);
		void saveAssociationsMap(const std::string &folder);
		void loadFeatures(const std::string &folder);
		void savePRC(const std::string &folder, FeatureAssociation *fa);
		void savePRCMap(const std::string &folder, LandmarkAssociation *fa);
		// frame=0 - current frame;
		// frame=1 - global frame;
		// frame=2 - world frame;
		void vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, int frame=0);
		void visScan(boost::shared_ptr<pcl::visualization::PCLVisualizer> v);
		void visScan(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> v);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		ptrPointCloud& points() {return point_cloud;}
		ptrNormalCloud& normals() {return normal_cloud;}
		ptrPixelCloud& pixels() {return pixel_cloud;}
		cv::Mat& imgRGB() {return img_rgb;}
		cv::Mat& imgDepth() {return img_depth;}

		// file_rgb, file_depth - full path;
		void loadScan(const double &time, const std::string &file_depth, const std::string &file_rgb);

		void addEdgePoint(int i);
		int sizeEdgePoint() const {return edge_points.size();}
		EdgePoint* edgePoint(int i) const {return edge_points[i];}

		ScaleLines key_lines;
		std::vector<std::string> key_lines_id;

		Eigen::Vector3d& Constraints() {return constraints;}
	
	private: 

		double time_stamp;
		Scan *scan_ref;

		Transform T_cg; // transform from global frame to current frame;
		Transform T_cr; // transform from reference frame to current frame;
		Transform T_cw; // [groundtruth] transform from world frame to current frame;

		FeatureAssociation *feature_association;
		LandmarkAssociation *landmark_association;
//		FeatureAssociation *feature_association_sim;
//		std::vector<Feature*> features;
		std::map<std::string,Feature*> features;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
		pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;
		cv::Mat img_rgb, img_depth;
		std::vector<EdgePoint*> edge_points;
		
		Eigen::Matrix3d Rotation_PCA;
		Eigen::Vector3d constraints;
	};

}

#endif
