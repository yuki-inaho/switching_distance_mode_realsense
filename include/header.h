#pragma once
#include <iostream>
#include <mutex>
#include <fstream>
#include <chrono>
#include <string>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/h/rs_option.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
