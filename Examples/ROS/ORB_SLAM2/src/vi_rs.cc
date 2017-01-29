/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>        //thnabadee edited
#include <crossover_nav/odom_data.h>  //thnabadee edited
#include <std_msgs/Bool.h> 
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <sensor_msgs/PointCloud2.h> 


#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"


#include "MsgSync/MsgSynchronizer.h"

#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"


#include <librealsense/rs.hpp>
#include <cstdio>
#include <atomic>
#include <map>
#include "concurrency.hpp"

using namespace std;

sensor_msgs::ImuConstPtr simu;
ros::Publisher publisher_pose;
ros::Publisher pcl_pub;

bool RESET_REQUEST = false;

image_transport::Publisher mImagePub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    
    ORB_SLAM2::System* mpSLAM;

     
    // Transfor broadcaster (for visualization in rviz)
private:
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    sensor_msgs::PointCloud2 output;  
    //-----------------------------------------
};
void reset_callback(const std_msgs::BoolConstPtr& msg);




/**
 * converts from encoding string to OpenCV type id
 */
int encodingToId(std::string enc)
{
    // cannot unfortunately be in a switch
    if (enc == sensor_msgs::image_encodings::MONO8)
        return CV_8UC1;
    else if (enc == sensor_msgs::image_encodings::MONO16)
        return CV_16UC1;
    else if (enc == sensor_msgs::image_encodings::BGR8 ||
              enc == sensor_msgs::image_encodings::RGB8)
        return CV_8UC3;
    else if (enc == sensor_msgs::image_encodings::BGRA8 ||
              enc == sensor_msgs::image_encodings::RGBA8)
        return CV_8UC4;
    else if (enc == sensor_msgs::image_encodings::BGR16 ||
              enc == sensor_msgs::image_encodings::RGB16)
        return CV_16UC3;
    else if (enc == sensor_msgs::image_encodings::BGRA16 ||
              enc == sensor_msgs::image_encodings::RGBA16)
        return CV_16UC4;
    else
        return -1;
}

/**
 * converts from OpenCV type id to encoding string to
 */
std::string idToEncoding(int type)
{
    // cannot unfortunately be in a switch
    switch (type)
    {
    case CV_8UC1:
        return sensor_msgs::image_encodings::MONO8;
    case CV_16UC1:
        return sensor_msgs::image_encodings::MONO16;
    case CV_8UC3:
        return sensor_msgs::image_encodings::BGR8;
    case CV_8UC4:
        return sensor_msgs::image_encodings::BGRA8;
    case CV_16UC3:
        return sensor_msgs::image_encodings::BGR16;
    case CV_16UC4:
        return sensor_msgs::image_encodings::BGRA16;
    default:
        return "";
    }
}

/**
 * Converts sensor_msgs::Image to cv::Mat
 */
cv::Mat imageToMat(sensor_msgs::Image image) {
    int type = encodingToId(image.encoding);
    if (type == -1)
    {
        ROS_ERROR("[Invalid encoding specified: %s", image.encoding.c_str());
        return cv::Mat();
    }

    cv::Mat matTemp(image.height, image.width, type);
    memcpy(matTemp.data, &image.data[0], image.step * image.height);
    return matTemp;
}

/**
 * Converts sensor_msgs::ImageConstPtr to cv::Mat
 */
cv::Mat imageToMat(const sensor_msgs::ImageConstPtr &image)
{
    return imageToMat(*image);
}

/**
 * Converts a cv::Mat to sensor_msgs::ImagePtr
 */
sensor_msgs::ImagePtr matToImage(cv::Mat mat)
{
    sensor_msgs::ImagePtr output(new sensor_msgs::Image());

    // copy header
    output->header.stamp = ros::Time::now();
    output->width = mat.cols;
    output->height = mat.rows;
    output->step = mat.cols * mat.elemSize();
    output->is_bigendian = false;
    output->encoding = idToEncoding(mat.type());

    // copy actual data
    output->data.assign(mat.data, mat.data + size_t(mat.rows * output->step));
    return output;
}







int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
 
    ORB_SLAM2::ConfigParam config(argv[2]);
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    sensor_msgs::ImageConstPtr imageMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);

    int LOST_COUNT=0;
    crossover_nav::odom_data odomOUT;
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    sensor_msgs::PointCloud2 output;  
    //-----------------------------------------------------

    // ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub_reset = nodeHandler.subscribe("/slam_reset", 1, &reset_callback);
    // ros::Subscriber imagesub = nodeHandler.subscribe("/image_repub", 1, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
    ros::Subscriber imusub = nodeHandler.subscribe("/imu_repub", 5, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);


    pcl_pub = nodeHandler.advertise<sensor_msgs::PointCloud2> ("/ORB_SLAM2/Cloud", 10,true);
    publisher_pose = nodeHandler.advertise<crossover_nav::odom_data>("slam", 10);
    image_transport::ImageTransport it(nodeHandler);
    mImagePub = it.advertise("/Frame",10);
    

    //INTEL REALSENSE
       rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if (ctx.get_device_count() == 0) return EXIT_FAILURE;

    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    const auto streams = 2;
    std::vector<uint16_t> supported_streams = { (uint16_t)rs::stream::depth, (uint16_t)rs::stream::color};
    const size_t max_queue_size = 1; // To minimize latency prefer frame drops
    single_consumer_queue<rs::frame> frames_queue[streams];
    // texture_buffer buffers[streams];
    std::atomic<bool> running(true);

    struct resolution
    {
        int width;
        int height;
        rs::format format;
    };
    std::map<rs::stream, resolution> resolutions;


    for (auto i : supported_streams)
    {
        dev->set_frame_callback((rs::stream)i, [dev, &running, &frames_queue, &resolutions, i, max_queue_size](rs::frame frame)
        {
            if (running && frames_queue[i].size() <= max_queue_size) frames_queue[i].enqueue(std::move(frame));
        });
    }

    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30, rs::output_buffer_format::native);


// Configure depth to run at VGA resolution at 30 frames per second
    // rs::option::r200_lr_auto_exposure_enabled;
    int aee = dev->get_option(rs::option::r200_lr_exposure);
    printf("Startup: autoexposure: %d\n",aee) ;
    int ee =  dev->get_option(rs::option::r200_lr_gain);
    printf("Startup: Emitter: %d\n", ee) ;
    dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 0);
    dev->set_option(rs::option::r200_emitter_enabled, 1);
    dev->set_option(rs::option::r200_lr_exposure, 200);//7 for outdoor 400 for indoor
    dev->set_option(rs::option::r200_lr_gain, 200);
    aee = dev->get_option(rs::option::r200_lr_exposure);
    printf("Settings: autoexposure to: %d\n",aee) ;
    ee =  dev->get_option(rs::option::r200_lr_gain);
    printf("Settings: Emitter to: %d\n", ee) ;


    dev->start();



    ros::Rate r(200);
    while(ros::ok()) {
        ros::Time ros_time=ros::Time::now();
        // static ros::Time rs_ts=ros_time;
        // if(ros_time - rs_ts > ros::Duration(0.033))
        // {
        //     rs_ts = ros_time;
            

            rs::frame frame;

            static std::clock_t start= std::clock();
            double duration;
    
            if(frames_queue[1].try_dequeue(&frame)) 
            {
                duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

                // std::cout<<"printf: "<< duration <<'\n';
                start = std::clock();
                // Retrieve our images
                // const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
                // imageDataRGB.data = (uint8_t *)dev->get_frame_data(rs::stream::color);
                cv::Mat imageDataRGB(480, 640, CV_8UC3, (uchar*)reinterpret_cast<const uint8_t *>(frame.get_data()));



                // cv::resize(imageDataRGB, imageDataRGB,
                //     cv::Size(
                //         960,
                //         540
                //     ));

                

                // Because the Realsense stream provides the color as BGR and opencv uses RGB to represent the image, the last step is to transform the matrix to the correct format:
                // cv::cvtColor(imageDataRGB, imageDataRGB, CV_BGR2RGB);

                // sensor_msgs::ImageConstPtr img=matToImage(imageDataRGB);
                msgsync.addImageMsg(matToImage(imageDataRGB));
            }
        // depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());
        const uint16_t * depth_frame;
        const uint16_t one_meter = static_cast<uint16_t>(1.0f / dev->get_depth_scale());  
        float x_av=0.0;
        float y_av=0.0;
        int num_grid=0;
        int row_grid=-1;

         if (frames_queue[0].try_dequeue(&frame))
          {
            // depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());
            // buffers[i].upload(frame);
            // cv::Mat d(480, 640, CV_32FC1, (uchar*)depth_frame);
             // mImagePub.publish(matToImage(d));


            depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());
 

            char buffer[(640/10+1)*(480/20)+1];
            char * out = buffer;
            int coverage[64] = {};
            for(int y=0; y<480; ++y)
            {
              for(int x=0; x<640; ++x)
              {
                  int depth = *depth_frame++;
                  if(depth > 0 && depth < one_meter) ++coverage[x/10];
              }

              if(y%20 == 19)
              {
                  for(int & c : coverage)
                  {
                          *out++ = " .:nhWWWW"[c/25]; //scale of depth (char)
                          c = 0;
                      }
                      *out++ = '\n';
                  }
              }
              *out++ = 0;

              x_av=32;
              y_av=11;
              num_grid=0;
              for(int i=0;i<(640/10+1)*(480/20)+1;i++)
              {
                  if(i%(640/10+1)==0) row_grid++;

                  if(buffer[i]=='W')
                  {
                      x_av+=(i%(640/10+1));
                      y_av+=row_grid;
                      num_grid++;
                  }
              }
              if(num_grid!=0)
              {
                  x_av/=num_grid;
                  y_av/=num_grid;
                  row_grid=-1;
              }
              x_av-=32;
              y_av-=11;
              // if(TERMINAL_VISUALIZATION)
                // printf("\n%s", buffer);
        }


      
            
        // }
        bool bdata = msgsync.getRecentMsgs(imageMsg,vimuMsg);
         if(bdata)
        {
            std::vector<ORB_SLAM2::IMUData> vimuData;
            //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
            for(unsigned int i=0;i<vimuMsg.size();i++)
            {
                sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                double ax = imuMsg->linear_acceleration.x;
                double ay = imuMsg->linear_acceleration.y;
                double az = imuMsg->linear_acceleration.z;
                // if(bAccMultiply98)
                // {
                //     ax *= g3dm;
                //     ay *= g3dm;
                //     az *= g3dm;
                // }
                ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                ax,ay,az,imuMsg->header.stamp.toSec());
                vimuData.push_back(imudata);
                //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
            }

            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(imageMsg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }

            // Consider delay of image message
            //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            cv::Mat im = cv_ptr->image.clone();
            {
                // To test relocalization
                static double startT=-1;
                if(startT<0)
                    startT = imageMsg->header.stamp.toSec();
                // Below to test relocalizaiton
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
                    im = cv::Mat::zeros(im.rows,im.cols,im.type());
            }
            cv::Mat mTcw = SLAM.TrackMonocular(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);

            ros::Time currenttime = imageMsg->header.stamp - ros::Duration(imageMsgDelaySec);
            odomOUT.odom_state = SLAM.GetState();
            odomOUT.Scale_Inited = SLAM.GetVINSInited();
            if(odomOUT.odom_state==3) {
                LOST_COUNT ++ ;
                if(LOST_COUNT>30) {
                    SLAM.Reset();
                    LOST_COUNT=0;
                }
            }else{
                LOST_COUNT=0;
            }
            
            if(!mTcw.empty())
            {    
                
                cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twc = -Rwc*mTcw.rowRange(0,3).col(3);
                tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                                Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                                Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
                // tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

                tf::Quaternion Q;
                M.getRotation(Q);
                Q.normalize();
                tf::Quaternion q_vc_rebuild(-Q.z(),
                   Q.x(),
                   Q.y(),
                   -Q.w());
                tf::Matrix3x3 M_rebuild;
                M_rebuild.setRotation(q_vc_rebuild);

                tf::Vector3 V_rebuild(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1));


                tf::Transform tfTcw(M_rebuild,V_rebuild);

                
                odomOUT.pose.pose.position.x = V_rebuild.x();
                odomOUT.pose.pose.position.y = V_rebuild.y();
                odomOUT.pose.pose.position.z = V_rebuild.z();
                odomOUT.pose.pose.orientation.x = q_vc_rebuild.getX();
                odomOUT.pose.pose.orientation.y = q_vc_rebuild.getY();
                odomOUT.pose.pose.orientation.z = q_vc_rebuild.getZ();
                odomOUT.pose.pose.orientation.w = q_vc_rebuild.getW();
                odomOUT.track_count = SLAM.GetNumTrack();
                

                // tf::Transform vision_to_odom;
                // tf::Quaternion q_ov;
                // q_ov.setEuler(1.5707,0,-1.5707);
                // q_ov.setRPY(-1.5707,0,-1.5707);
                // vision_to_odom.setRotation(q_ov);

                static tf::TransformBroadcaster mTfBr;
                
                mTfBr.sendTransform(tf::StampedTransform(tfTcw,currenttime, "vision", "Camera"));

                cloud.width = SLAM.GetMap()->GetReferenceMapPoints().size();  
                cloud.height = 1;  
                cloud.points.resize(cloud.width * cloud.height); 
                int temp=0;  
                for(auto mp: SLAM.GetMap()->GetReferenceMapPoints())  
                {
                  cv::Mat wp = mp->GetWorldPos();
                  cloud.points[temp].x = wp.at<float>(2);           // pos x: float
                  cloud.points[temp].y = -wp.at<float>(0);           // pos y: float
                  cloud.points[temp].z = -wp.at<float>(1);           // pos z: float
                  temp++;
                }

                pcl::toROSMsg(cloud, output);  
                output.header.stamp = currenttime;
                output.header.frame_id = "vision";
                pcl_pub.publish(output); 

            }
            
            odomOUT.header.stamp = currenttime;
            odomOUT.lost_count = LOST_COUNT;
            publisher_pose.publish(odomOUT);


            if(RESET_REQUEST) {
                SLAM.Reset();
                printf("SLAM reset by topic\n");
                RESET_REQUEST = false;
            }


            if(mImagePub.getNumSubscribers()) {
                sensor_msgs::ImagePtr imMsg = matToImage(SLAM.ImageToPub());
                imMsg->header.stamp = currenttime;
                imMsg->header.frame_id ="Camera";
                imMsg->header.frame_id ="Camera";
                imMsg->encoding = "bgr8";
            
                mImagePub.publish(imMsg);
            }

        }
        r.sleep();
        ros::spinOnce();



    }



    //REALSENSE
    running = false;
    for (auto i : supported_streams) frames_queue[i].clear();
    dev->stop();
    for (auto i : supported_streams)
    {
        if (dev->is_stream_enabled((rs::stream)i))
            dev->disable_stream((rs::stream)i);
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
void reset_callback(const std_msgs::BoolConstPtr& msg)
{
    RESET_REQUEST = msg->data;
    
}
// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     //thanabadee edited-------------------------------------------------------------------------------
//     ros::Time currenttime=msg->header.stamp;
//     cv::Mat mTcw = SLAM.TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    


// }


