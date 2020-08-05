cmake_minimum_required(VERSION 3.0) 
PROJECT (examples)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

############ dependencies ###############
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL (Uncomment when PointCloud.hpp is included)
# find_package(PCL 1.11 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories    (${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

set(THIRD_PARTY_LIBS ${OpenCV_LIBS}  
# ${PCL_LIBRARIES}
)

############ source and examples ###############
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(examples)