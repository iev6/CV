cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( OpenCVReprojectImageToPointCloud )

find_package( PCL 1.2 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )
<<<<<<< HEAD
#add_executable( OpenCVReprojectImageToPointCloud opencv_reproject_image_pcl.cpp )
#add_executable( OpenCVSmooth opencvsmooth.cpp)
add_executable( Trial trial.cpp)
#Trial is the trial file with contours method implemented
target_link_libraries( Trial ${OpenCV_LIBS} ${PCL_LIBRARIES})
#target_link_libraries ( OpenCVSmooth ${PCL_LIBRARIES})
#target_link_libraries( OpenCVReprojectImageToPointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
=======
add_executable( OpenCVReprojectImageToPointCloud opencv_reproject_image_pcl.cpp )
add_executable( OpenCVSmooth opencvsmooth.cpp)
target_link_libraries ( OpenCVSmooth ${PCL_LIBRARIES})
target_link_libraries( OpenCVReprojectImageToPointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
>>>>>>> 331f6678eb855c9c55054cfd4198e4b709b40601

