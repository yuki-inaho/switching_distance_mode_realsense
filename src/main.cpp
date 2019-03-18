
#include "header.h"
#include <unistd.h>
#include <chrono>
#include <cstdlib>

std::string data_save_path = ".";
double THETA_X = 0.0;

using namespace cv;

void 
change_option(const rs2::sensor& sensor, rs2_option option_type, float requested_value)
{
	try
	{
		sensor.set_option(option_type, requested_value);
	}
	catch(const rs2::error& e)
	{
		std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}


void 
drawPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::mutex &flagMutex){
    std::lock_guard<std::mutex> lock (flagMutex);
    auto f_update = viewer->updatePointCloud(cloud, "sample cloud");
    if (!f_update){
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    }
    viewer->spinOnce(10);
}

void 
savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_set, std::mutex &flagMutex, std::string FileName){
    std::lock_guard<std::mutex> lock (flagMutex);
    pcl::io::savePCDFileASCII (data_save_path + "/../data/pcd/" + FileName, *pc_set);
}

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; 
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char * argv[]) try
{
    
    std::mutex flagMutex;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::colorizer color_map;
    //rs2::pipeline pipe;
    rs2::hole_filling_filter hf_filter;
    
    rs2::config config_pipe;
    const int stream_width = 1280;
    const int stream_height = 720;
    const int stream_fps = 30;    
    config_pipe.enable_stream(rs2_stream::RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
    config_pipe.enable_stream(rs2_stream::RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, stream_fps);

    rs2::config config_pipe2;
    config_pipe2.enable_stream(rs2_stream::RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
    config_pipe2.enable_stream(rs2_stream::RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, stream_fps);

    //auto profile = pipe.start();
    rs2::pipeline pipe;
    rs2::pipeline pipe2;
    pipe.start(config_pipe);
    rs2::pipeline_profile profile = pipe.get_active_profile();        
    auto dev = profile.get_device().as<rs400::advanced_mode>();//    
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    pipe.stop();

    pipe2.start(config_pipe2);
    rs2::pipeline_profile profile2 = pipe2.get_active_profile();            
    auto dev2 = profile2.get_device().as<rs400::advanced_mode>();//    
    auto sensor2 = profile.get_device().first<rs2::depth_sensor>();        
    pipe2.stop();
    pipe.start(config_pipe);

//    rs2::rs2_error* e = 0;
//    int is_advanced_mode_enabled;
//    rs2::rs2_is_enabled(dev, &is_advanced_mode_enabled, &e);    

    //if(!dev.is_enabled()){
    //    dev.toggle_advanced_mode(1);
    //}


    sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
    change_option(sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    change_option(sensor, RS2_OPTION_EMITTER_ENABLED, 1);
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = depth_stream.get_intrinsics();

    sensor2.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
    change_option(sensor2, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    change_option(sensor2, RS2_OPTION_EMITTER_ENABLED, 1);
    auto depth_stream2 = profile2.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics2 = depth_stream2.get_intrinsics();

    const auto window_name = "Depth";
    namedWindow (window_name, WINDOW_AUTOSIZE);
    moveWindow(window_name, 1500,10);
    const auto window_name2 = "Depth2";
    namedWindow (window_name2, WINDOW_AUTOSIZE);
    moveWindow(window_name2, 1200,10);

    const auto window_name_color = "Color";
    namedWindow (window_name_color, WINDOW_AUTOSIZE);
    moveWindow(window_name_color, 1500,500);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);
//    viewer = simpleVis(pcl_points);

    int currentKey = 0;
    int count = 0;
    cv::Mat gimg_raw, zimg_raw ,gimg, zimg, gimg_view, zimg_view;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dat(new pcl::PointCloud<pcl::PointXYZ>);


    std::chrono::system_clock::time_point tcount,last_count; // 型は auto で可
    last_count = std::chrono::system_clock::now(); // 計測開始時間    

    bool toggled = false;
    while (currentKey != 27)
    {
        cout << "test" << endl;
//        STDepthTableControl table_tmp = dev.get_depth_table();
//        cout << table_tmp.disparityShift << endl;
        rs2::frameset data;
        if(!toggled){
            data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
            pipe.stop();
            STDepthTableControl table2 = dev2.get_depth_table();
            table2.disparityShift = 100;
            dev2.set_depth_table(table2);
            pipe2.start(config_pipe2);
            toggled = true;
        }else{
            data = pipe2.wait_for_frames(); // Wait for next set of frames from the camera
            pipe2.stop();
            STDepthTableControl table = dev.get_depth_table();
            table.disparityShift = 0;
            dev.set_depth_table(table);
            pipe.start(config_pipe);
            toggled = false;
        }

		rs2::align align( rs2_stream::RS2_STREAM_COLOR );
        rs2::frameset aligned_frameset = align.process( data );
        rs2::frame depth = aligned_frameset.get_depth_frame();
        rs2::frame color = data.get_color_frame();
        rs2::frame depth_heat = depth.apply_filter(color_map);
        
        tcount = std::chrono::system_clock::now(); // 計測開始時間    
        cout << std::chrono::duration_cast<std::chrono::milliseconds>(tcount - last_count).count() << endl;
        last_count = tcount;
//        last_time = depth.get_timestamp();        
        ///////////////////////////////////////////////////////////////
        /*   
             近距離測定部分 
        */
/*
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        auto data2 = pipe.wait_for_frames();
//        table_tmp = dev.get_depth_table();
//        cout << table_tmp.disparityShift << endl;
        rs2::align align2( rs2_stream::RS2_STREAM_COLOR );
        rs2::frameset aligned_frameset2 = align2.process( data2 );
        //rs2::frame depth2 = aligned_frameset2.get_depth_frame();
        rs2::frame depth2 = aligned_frameset2.get_depth_frame();
        cout << depth2.get_timestamp() - last_time << endl; 
        rs2::frame color2 = data2.get_color_frame();
        rs2::frame depth_heat2 = depth2.apply_filter(color_map);
        */
        /*
        else{
            dev.toggle_advanced_mode(true);    
        }
*/


//        table.disparityShift = 0;
//        dev.set_depth_table(table);

//        end = std::chrono::system_clock::now();  // 計測終了時間
//        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
//        std::cout << "elapsed" << elapsed << std::endl;

       ///////////////////////////////////////////////////////////////
/*
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
        points = pc.calculate(depth);
        clouds = points_to_pcl(points);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 1.0))); //1.2m
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (clouds);
        condrem.setKeepOrganized(true);
        condrem.filter (*cloud_flt);

        pcl::VoxelGrid<pcl::PointXYZ> sorv;
        sorv.setInputCloud (cloud_flt);
        sorv.setLeafSize (0.003f, 0.003f, 0.003f);
        sorv.filter (*cloud_flt);

        Eigen::Affine3f transform_flip = Eigen::Affine3f::Identity();
        transform_flip.rotate (Eigen::AngleAxisf (180/180*M_PI, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*cloud_flt, *cloud_flt, transform_flip);        
        transform_flip = Eigen::Affine3f::Identity();
        transform_flip.rotate (Eigen::AngleAxisf (float(THETA_X/180.0)*M_PI, Eigen::Vector3f::UnitX()));
        pcl::transformPointCloud (*cloud_flt, *cloud_flt, transform_flip);        
*/
        const int w = depth_heat.as<rs2::video_frame>().get_width();
        const int h = depth_heat.as<rs2::video_frame>().get_height();
        const int w_col = color.as<rs2::video_frame>().get_width();
        const int h_col = color.as<rs2::video_frame>().get_height();

        cv::Mat image(Size(w, h), CV_8UC3, (void*)depth_heat.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_col(Size(w_col, h_col), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cvtColor(image_col, image_col, cv::COLOR_BGR2RGB);

        cv::Mat depthImage(Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depthImage_color = cv::Mat::zeros( depthImage.rows,
                                    depthImage.cols,
                                    CV_8UC3);
        cv::Mat depthImage8, IRImage8;
        depthImage.convertTo( depthImage8, CV_8U, 255.0 / 4096.0 );
        cv::applyColorMap(depthImage8, depthImage_color, COLORMAP_JET);    
/*
        cv::Mat depthImage2(Size(w, h), CV_16UC1, (void*)depth2.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depthImage_color2 = cv::Mat::zeros( depthImage2.rows,
                                    depthImage2.cols,
                                    CV_8UC3);
        cv::Mat depthImage28, IRImage28;
        depthImage2.convertTo( depthImage28, CV_8U, 255.0 / 4096.0 );
        cv::applyColorMap(depthImage28, depthImage_color2, COLORMAP_JET);    
*/

        if (currentKey == 's'){
            cout << "PointCloud Saved" << endl;
            std::string  depth_name= "Depth" + std::to_string(count) + ".png" ;
            imwrite(data_save_path + "/../data/depth/" + depth_name, depthImage_color);
            std::string  color_name = "Color" + std::to_string(count) + ".png" ;
            imwrite(data_save_path + "/../data/color/" + color_name, image_col);
            std::string  pcd_name = "pc_" + std::to_string(count) + ".pcd" ;
//            savePointCloud(pcl_points, flagMutex, pcd_name);            
            //savePointCloud_main(test_pc);
            count += 1;
        }

        cv::resize(image_col, image_col, Size(std::floor(double(w)/2.0), std::floor(double(h)/2.0)));        
        cv::resize(depthImage_color, depthImage_color, Size(std::floor(double(w)/2.0), std::floor(double(h)/2.0)));        
//        cv::resize(depthImage_color2, depthImage_color2, Size(std::floor(double(w)/2.0), std::floor(double(h)/2.0)));        

        // Update the window with new data
        imshow(window_name, depthImage_color);
//        imshow(window_name2, depthImage_color2);
        imshow(window_name_color, image_col);
        currentKey = waitKey (100);            
        //viewer->spinOnce(10);
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


