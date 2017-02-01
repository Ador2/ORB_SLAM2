//
// TODO: generate pointcloud
// TODO: provide depth with correct scaling, currently it is Z16 that is the scaling of
// rs::format::z16
// alternatively directly use: points
//
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <pcl/ros/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <librealsense/rs.hpp>
#include <cstdio>
#include <atomic>
#include <map>
#include "concurrency.hpp"

ros::Publisher realsense_reg_points_pub;
ros::Publisher v_avoid_pub;

image_transport::CameraPublisher realsense_rgb_image_pub;
image_transport::CameraPublisher realsense_ir_image_pub;
image_transport::CameraPublisher realsense_depth_image_pub;

int getNumRGBSubscribers()
{
  return realsense_reg_points_pub.getNumSubscribers() + realsense_rgb_image_pub.getNumSubscribers() + realsense_ir_image_pub.getNumSubscribers();
}

int getNumDepthSubscribers()
{
  int n = realsense_reg_points_pub.getNumSubscribers() + realsense_depth_image_pub.getNumSubscribers() + realsense_ir_image_pub.getNumSubscribers();
#ifdef V4L2_PIX_FMT_INZI
  n += realsense_infrared_image_pub.getNumSubscribers();
#endif
  return n;
}
static float constrain_float(float amt, float low, float high) 
{
  // the check for NaN as a float prevents propogation of
  // floating point errors through any function that uses
  // constrain_float(). The normal float semantics already handle -Inf
  // and +Inf
  if (isnan(amt)) {
    return (low+high)*0.5f;
  }
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*
    ci.header.stamp = head_time_stamp;
    ci.header.seq = head_sequence_id;
*/
void rs_intrinsics2camerainfo(sensor_msgs::CameraInfo & ci, rs::intrinsics x, std::string frame_id)
{
  //memset(&ci,0,sizeof(ci));
  ci.header.frame_id = frame_id;
  // distortion_model D
  ci.width = x.width;
  ci.height = x.height;
  ci.K[0] = x.fx;
  ci.K[2] = x.ppx;
  ci.K[4] = x.fy;
  ci.K[5] = x.ppy;


}

void mySigintHandler(int sig)
{
  ros::shutdown();
}
















/**
   converts from encoding string to OpenCV type id
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
   converts from OpenCV type id to encoding string to
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
   Converts sensor_msgs::Image to cv::Mat
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
   Converts sensor_msgs::ImageConstPtr to cv::Mat
*/
cv::Mat imageToMat(const sensor_msgs::ImageConstPtr &image)
{
  return imageToMat(*image);
}

/**
   Converts a cv::Mat to sensor_msgs::ImagePtr
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






bool REALSENSE_ENABLE=false;
bool LR_EMITTER_ENABLED=true;
bool LR_AUTO_EXPOSURE_ENABLED=false;
int LR_GAIN;
int LR_EXPOSURE; 
bool TERMINAL_VISUALIZATION=false;
double RANGE_TO_AVOID = 1.0;




int main(int argc, char * argv[])
{
  ros::init(argc, argv, "OBSTACLE_AVOIDANCE");
  std::string topic_points_id = "/points";
  std::string topic_depth_id = "/depth";
  std::string topic_ir_id = "/ir";
  std::string topic_image_rgb_id = "/rgb";
  std::string depth_frame_id = "/sr300_depth_optical_frame";
  std::string rgb_frame_id = "/sr300_rgb_optical_frame";

  ros::NodeHandle n;
  ros::NodeHandle private_node_handle_("~");
  image_transport::ImageTransport image_transport(n);

  //    private_node_handle_.param("realsense_camera_type", realsense_camera_type, std::string("Intel(R) RealSense(TM) 3D Camer"));
  //    private_node_handle_.param("rgb_frame_id", rgb_frame_id, std::string("_rgb_optical_frame"));
  //    private_node_handle_.param("rgb_frame_w", rgb_frame_w, 1280);
  //    private_node_handle_.param("rgb_frame_h", rgb_frame_h, 720);



  rs::intrinsics depth_intrin;
  rs::extrinsics depth_to_color;
  rs::intrinsics color_intrin;
  rs::intrinsics ir_intrin;
  memset(&depth_intrin, 0, sizeof(depth_intrin));
  memset(&depth_to_color, 0, sizeof(depth_to_color));
  memset(&color_intrin, 0, sizeof(color_intrin));
  memset(&ir_intrin, 0, sizeof(ir_intrin));



  n.param("REALSENSE_ENABLE", REALSENSE_ENABLE, false);
  n.param("LR_AUTO_EXPOSURE_ENABLED", LR_AUTO_EXPOSURE_ENABLED, false);
  n.param("LR_EXPOSURE", LR_EXPOSURE, 7);
  n.param("LR_GAIN", LR_GAIN, 200);
  n.param("LR_EMITTER_ENABLED", LR_EMITTER_ENABLED, true);
  n.param("TERMINAL_VISUALIZATION", TERMINAL_VISUALIZATION, false);
  n.param("RANGE_TO_AVOID", RANGE_TO_AVOID, 1.0);



  //INTEL REALSENSE
  rs::context ctx;
  printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
  if (ctx.get_device_count() == 0 || !REALSENSE_ENABLE) return EXIT_FAILURE;

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

  dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
  dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60, rs::output_buffer_format::native);


  if (!dev->is_stream_enabled(rs::stream::color))
  {
    std::cerr << "cannot open\n";
    dev->stop();
    return 0;
  }
  else
  {
    auto &intrin  = color_intrin;
    // auto stream = rs::stream::rectified_color;
    auto stream = rs::stream::color;
    intrin = dev->get_stream_intrinsics(stream);
    auto fmt = dev->get_stream_format(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height << " format " << fmt;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
  }
  // TODO: unsupported INVERSE_BROWN_CONRADY
  std::cout << "allocating color_ci\n";
  sensor_msgs::CameraInfoPtr color_ci_ptr(boost::make_shared<sensor_msgs::CameraInfo>());
  rs_intrinsics2camerainfo(*color_ci_ptr, color_intrin, rgb_frame_id);



  // rs::option::r200_lr_auto_exposure_enabled;
  bool aee = dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
  printf("Startup: autoexposure: %d\n",aee) ;
  bool ee =  dev->get_option(rs::option::r200_emitter_enabled);
  printf("Startup: Emitter: %d\n", ee) ;
  dev->set_option(rs::option::r200_lr_auto_exposure_enabled, LR_AUTO_EXPOSURE_ENABLED);
  dev->set_option(rs::option::r200_emitter_enabled, LR_EMITTER_ENABLED);
  dev->set_option(rs::option::r200_lr_exposure, LR_EXPOSURE); //7 for outdoor 400 for indoor
  dev->set_option(rs::option::r200_lr_gain, LR_GAIN);
  aee = dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
  printf("Settings: autoexposure to: %d\n",aee) ;
  ee =  dev->get_option(rs::option::r200_emitter_enabled);
  printf("Settings: Emitter to: %d\n", ee) ;



  dev->set_option(rs::option::frames_queue_size, 2.00);
  printf("FRMAE Q SIZE %f\n\n", dev->get_option(rs::option::frames_queue_size));

  dev->start();


  // fill in ...
  realsense_rgb_image_pub = image_transport.advertiseCamera(topic_image_rgb_id, 1);
  v_avoid_pub = 

  v_avoid_pub = n.advertise<geometry_msgs::TwistStamped>("v_avoid", 1 );

  std::cout << "advertised\n";


  ros::Rate loop_rate(120);
  ros::Rate idle_rate(1);


  std::cout << "started " << std::endl;

  unsigned int head_sequence_id = 0;
  ros::Time head_time_stamp;
  while (true)
  {
    // any subscription
    while ((getNumRGBSubscribers() + getNumDepthSubscribers()) == 0 && ros::ok())
    {
      ros::spinOnce();
      idle_rate.sleep();
    }
    if (!ros::ok())
      break;

    rs::frame frame;

    static std::clock_t start = std::clock();
    double duration;

    if (frames_queue[1].try_dequeue(&frame))
    {
      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

      // std::cout<<"printf: "<< duration <<'\n';
      start = std::clock();
      // Retrieve our images
      // const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
      // imageDataRGB.data = (uint8_t *)dev->get_frame_data(rs::stream::color);
      cv::Mat imageDataRGB(480, 640, CV_8UC3, (uchar*)reinterpret_cast<const uint8_t *>(frame.get_data()));

      cv::Mat greyMat;

      cv::cvtColor(imageDataRGB, greyMat, CV_RGB2GRAY);

      sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

      rgb_img = matToImage(greyMat);

      realsense_rgb_image_pub.publish(rgb_img, color_ci_ptr);

    }
    // depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());
    const uint16_t * depth_frame;
    const uint16_t max_range = static_cast<uint16_t>((float)RANGE_TO_AVOID / dev->get_depth_scale());
    float x_av = 0.0;
    float y_av = 0.0;
    int num_grid = 0;
    int row_grid = -1;

    if (frames_queue[0].try_dequeue(&frame))
    {
      // depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());
      // buffers[i].upload(frame);
      // cv::Mat d(480, 640, CV_32FC1, (uchar*)depth_frame);
      // mImagePub.publish(matToImage(d));


      depth_frame = reinterpret_cast<const uint16_t *>(frame.get_data());


      char buffer[(640 / 10 + 1) * (480 / 20) + 1];
      char * out = buffer;
      int coverage[64] = {};
      for (int y = 0; y < 480; ++y)
      {
        for (int x = 0; x < 640; ++x)
        {
          int depth = *depth_frame++;
          if (depth > 0 && depth < max_range) ++coverage[x / 10];
        }

        if (y % 20 == 19)
        {
          for (int & c : coverage)
          {
            *out++ = " .:nhWWWW"[c / 25]; //scale of depth (char)
            c = 0;
          }
          *out++ = '\n';
        }
      }
      *out++ = 0;

      x_av = 32;
      y_av = 11;
      num_grid = 0;
      for (int i = 0; i < (640 / 10 + 1) * (480 / 20) + 1; i++)
      {
        if (i % (640 / 10 + 1) == 0) row_grid++;

        if (buffer[i] == 'W')
        {
          x_av += (i % (640 / 10 + 1));
          y_av += row_grid;
          num_grid++;
        }
      }
      if (num_grid != 0)
      {
        x_av /= num_grid;
        y_av /= num_grid;
        row_grid = -1;
      }
      x_av -= 32;
      y_av -= 11;
      if(TERMINAL_VISUALIZATION)
        printf("\n%s", buffer);

      geometry_msgs::TwistStamped v_avoid_msgs;

      tf::Vector3 v_avoid;

      //if x_av positive mean quad must left (+y body vel)
      //if y_av positive mean object below quad must go up (+z body vel)
      //center is zero,zero
      //if num is positive quad must 
      //y_av not used yet (height avoidance)
      const float VEL_MAX = 1.5;
      float x_conv = -((float)num_grid)/450;
      float y_conv = VEL_MAX*(x_av/32)/**(((float)num_grid)/250)*/; //if num_grid is low (<20) mean obstacle maybe noise so be relax

      if(y_conv>0)
      { 
        y_conv = constrain_float(y_conv,0,VEL_MAX);
        y_conv = -y_conv + VEL_MAX+0.2;
      }
      else //(y_conv<0)
      {
        y_conv = constrain_float(y_conv,-VEL_MAX,0);
        y_conv = -y_conv - VEL_MAX-0.2;
      }

      if(fabs(x_conv) < 0.3) x_conv=0;

      if(num_grid < 10) y_conv =0;
      tf::Vector3 vel_b_avoid(x_conv,
                              y_conv,
                              0);

      v_avoid_msgs.header.stamp = ros::Time::now();
      v_avoid_msgs.twist.linear.x = vel_b_avoid.x();
      v_avoid_msgs.twist.linear.y = vel_b_avoid.y();
      v_avoid_msgs.twist.linear.z = vel_b_avoid.z();


      // printf("avoid = %.2f\t%.2f\n", vel_b_avoid.x(),vel_b_avoid.y()/**(((float)num_grid)/400)*/);

      v_avoid_pub.publish(v_avoid_msgs);
    }



    loop_rate.sleep();

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

  return  0;
}