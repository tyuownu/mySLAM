#include <iostream>
#include <rgbd_pair.h>
#include <rgbd_image.h>
#include <intrinsic_matrix.h>
#include <Eigen/Geometry>
#include <update_cfg.h>

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
  cv::namedWindow( "grey", CV_WINDOW_AUTOSIZE);
  cv::namedWindow( "depth", CV_WINDOW_AUTOSIZE);

  cv::imshow("grey", grey);
  cv::imshow("depth", depth_float);

  cv::waitKey(0);

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

  // default
  //mySLAM::IntrinsicMatrix intrinsics = mySLAM::IntrinsicMatrix::create(525.0f, 525.0f, 320.0f, 240.0f);
  // fr1
  mySLAM::IntrinsicMatrix intrinsics = mySLAM::IntrinsicMatrix::create(517.3f, 516.5f, 318.6f, 255.3f);
  // fr2
  //mySLAM::IntrinsicMatrix intrinsics = mySLAM::IntrinsicMatrix::create(520.9f, 521.0f, 325.1f, 249.7f);
  // fr3
  //mySLAM::IntrinsicMatrix intrinsics = mySLAM::IntrinsicMatrix::create(535.4f, 539.2f, 320.1f, 247.6f);

  mySLAM::RgbdCameraPyramid camera(640, 480, intrinsics); // width and height of image

  // setup tracker configuration
  mySLAM::DenseTracker::Config cfg = mySLAM::DenseTracker::getDefaultConfig();
  updateDenseTrackerConfig(cfg);

  mySLAM::KeyframeTrackerConfig frontend_cfg;
  mySLAM::KeyframeGraphConfig backend_cfg;
  updateKeyframeConfig(frontend_cfg, backend_cfg);

  camera.build(cfg.getNumLevels());

  mySLAM::KeyframeTracker keyframe_tracker();
  keyframe_tracker.configureTracking(cfg);
  keyframe_tracker.configureKeyframeSelection(frontend_cfg);
  keyframe_tracker.configureMapping(backend_cfg);
  // initialize first pose
  Eigen::Affine3d trajectory, relative;
  trajectory.setIdentity();
  keyframe_tracker.init(trajectory);

  mySLAM::RgbdImagePyramidPtr current;

  for(std::vector<mySLAM::RgbdPair>::iterator it = pairs.begin(); it != pairs.end(); it++)
  {
    current = load(camera, folder + it->RgbFile, folder + it->DepFile);
    if (!current) {continue;}

    if (pairs.end() - it == 1)
    {
      // TODO, maybe opti
    }

  }
}
int main()
{
  run();
  return 0;
}
