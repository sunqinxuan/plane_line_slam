
SET(SRC_LIST_MAP types/map.cpp types/types.cpp)
SET(SRC_LIST_EXTR features/geometric_feature_matching.cpp features/it_geometric_feature_matching.cpp
    features/plane_extraction_base.cpp  features/plane_segmentation.cpp
	features/edge_point_extraction.cpp  features/line_extraction.cpp) #features/plane_extraction.cpp
SET(SRC_LIST_MOTION motion/motion_estimation.cpp)
SET(SRC_LIST_EVAL evaluate/relative_pose_error.cpp)
SET(SRC_LIST_SYSTEMS systems/system_slam_tum.cpp systems/system_simulation_tum.cpp)
#systems/system_simulation.cpp systems/system_slam_sim.cpp  systems/system_relocalization.cpp)
#		 systems/system_extrinsic_calibration.cpp systems/system_plane_line_shadow.cpp

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ulysses ${PROJECT_SOURCE_DIR}/3rdparty/include)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/lib)

find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR}) 
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})
find_package(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(OpenCV REQUIRED)


ADD_LIBRARY(map SHARED ${SRC_LIST_MAP})
target_link_libraries(map ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_LIBRARY(features SHARED ${SRC_LIST_EXTR})
target_link_libraries(features map LSD LineMatchingLib) # ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_LIBRARY(motion ${SRC_LIST_MOTION})
target_link_libraries(motion map ${CERES_LIBRARIES})

#ADD_LIBRARY(evaluate SHARED ${SRC_LIST_EVAL})

ADD_LIBRARY(systems ${SRC_LIST_SYSTEMS})
target_link_libraries(systems features evaluate motion)



