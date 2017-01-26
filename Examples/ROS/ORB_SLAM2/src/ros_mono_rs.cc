/**
* Based on examples from ORB-SLAM2 (Copyright (C) 2014-2016 Raúl Mur-Artal).
*
* For more information see <https://github.com/raulmur/ORB_SLAM2>*
*/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <librealsense/rs.hpp>
#include <cstdio>

//#define GLFW_INCLUDE_GLU
//#include <GLFW/glfw3.h>

int main( int argc, char *argv[] ) try
{
    std::vector<std::string> arguments(argv + 1, argv + argc);

    //Vocabulary for detecting objects
    string Vocabulary = "/home/nuc/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.bin";
    //ConfigFile
    string Yaml = "/home/nuc/catkin_ws/src/ORB_SLAM2/config/realsense.yaml";
    //Video source
    //cv::VideoCapture cap(1); // open camera

    if(arguments.size() != 2)
    {
        std::cout  << "" << std::endl;
        std::cout  << "Usage: ORB_SLAM2-webcam VOCABULARY CONFIG.yaml VIDEO_STREAM" << std::endl;
        std::cout  << "Using default values" << std::endl;
        std::cout  << "" << std::endl;
    }
    else
    {
        Vocabulary = arguments[0];
        Yaml = arguments[1];
    }    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(Vocabulary,Yaml,ORB_SLAM2::System::MONOCULAR,true);
    // ORB_SLAM2::System SLAM(Vocabulary,Yaml,ORB_SLAM2::System::RGBD,true);


    //INTEL REALSENSE
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx; // Create a context object. This object owns the handles to all connected realsense devices.
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;
    rs::device * dev = ctx.get_device(0); //Use first device
    std::cout << ctx.get_device_count() << " connected to computer." << std::endl;
    std::cout << "Using first - " << dev->get_name() << " - " << dev->get_serial() << ", FW: " << dev->get_firmware_version() << std::endl;

    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->start();

    std::cout << endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;

    cv::Mat imageDataDepth = cv::Mat::zeros(480,640, CV_16UC1);
    //cv::Mat imageDataDepth = cv::Mat::zeros(480,640, CV_8UC1);
    cv::Mat imageDataRGB = cv::Mat::zeros(480,640, CV_8UC3);
    uint16_t* pixelPtrDepth = (uint16_t*)imageDataDepth.data;
    //uint8_t* pixelPtrDepth = (uint8_t*)imageDataDepth.data;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while(true)
    {
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        imageDataRGB.data = (uint8_t *)dev->get_frame_data(rs::stream::color);


        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();


        for (unsigned int i = 0; i < 640 * 480; i++) {
            pixelPtrDepth[i] = 0x00;
        }
        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;


                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx >= 0 && cy >= 0 && cx < color_intrin.width && cy < color_intrin.height)
                {
                    pixelPtrDepth[cy*640+cx] = (uint16_t)(depth_in_meters*5000);
                }
            }
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        double t = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - begin).count();
        SLAM.TrackMonocular(imageDataRGB,t);
        // SLAM.TrackRGBD(imageDataRGB,imageDataDepth,t);
    }

    // Stop all threads
    SLAM.Shutdown();

//    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}