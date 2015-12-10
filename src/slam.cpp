#include <iostream>
#include <rgbd_pair.h>
#include <intrinsic_matrix.h>
#include <Eigen/Geometry>

mySLAM::RgbdImagePyramidPtr load( mySLAM::RgbdCameraPyramid& camera,
                                  std::string rgb_file, std::string depth_file )
{
  cv::Mat rgb, grey, grey_s16, depth, depth_float;
  rgb   = cv::imread( rgb_file, 1 );
  depth = cv::imread( depth_file, -1 );
  if( rgb.total() == 0 || depth.total() == 0)
    return mySLAM::RgbdImagePyramidPtr();

  if( rgb.type() != CV_32FC1 )
  {
    if( rgb.type() == CV_8UC3 )
      cv::cvtColor( rgb, grey, CV_BGR2GRAY );
    else
      grey = rgb;

    grey.convertTo( grey_s16, CV_32F );
  }

  if( depth.type() != CV_32FC1)
  {
    mySLAM::ConvertRawDepthImage::convert(depth, depth_float, 1.0f /5000.0f);
  }

  mySLAM::RgbdImagePyramidPtr result = camera.create(grey_s16, depth_float);

  rgb.convertTo(result->level(0).rgb, CV_32FC3);

  return result;
}

void run()
{
  std::string str = "/media/DataSet/data_tgz/Handheld_SLAM/rgbd_dataset_freiburg1_room/assoc.txt";
  std::string folder = str.substr(0, str.find_last_of("/")+1);
  std::vector<mySLAM::RgbdPair> pairs;
  mySLAM::FileReader file_reader(str);
  file_reader.readAllEntries(pairs);
}
int main()
{
  run();
  return 0;
}
