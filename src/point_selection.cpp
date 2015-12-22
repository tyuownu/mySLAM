#include "point_selection.h"

namespace mySLAM
{
  // begin PointSelection function
  PointSelection::PointSelection(const PointSelectionPredicate& predicate) :
    pyramid_(0),
    predicate_(predicate),
    debug_(false)
  {}
  PointSelection::PointSelection(mySLAM::RgbdImagePyramid& pyramid,
                                 const PointSelectionPredicate& predicate) :
    pyramid_(&pyramid),
    predicate_(predicate),
    debug_(false)
  {}

  PointSelection::~PointSelection()
  {}

  void PointSelection::setRgbdImagePyramid(mySLAM::RgbdImagePyramid& pyramid)
  {
    pyramid_ = & pyramid;
    for (size_t id = 0; id < storage_.size(); ++id)
    {
      storage_[id].is_cached = false;
    }
  }

  size_t PointSelection::getMaximumNumberOfPoint(const size_t& level)
  {
    return size_t(pyramid_->level(0).intensity.total() * std::pow(0.25, double(level)));
  }

  void PointSelection::select(const size_t& level, PointSelection::PointIterator& first, PointSelection::PointIterator& last)
  {
    assert(pyramid_ != 0);
    pyramid_->compute(level + 1);
    if(storage_.size() < level + 1)
      storage_.resize(level + 1);
    Storage& storage = storage_[level];

    if(!storage.is_cached || debug_)  // TODO: what is this debug_?
    {
      mySLAM::RgbdImage& img = pyramid_->level(level);
      img.buildPointCloud();
      img.buildAccelerationStructure();

      storage.allocate(img.intensity.total());
      storage.points_end = selectPointsFromImage(img, storage.points.begin(), storage.points.end(), storage.debug_idx);
      storage.is_cached = true;
    }
    first = storage.points.begin();
    last = storage.points_end;
  }

  PointSelection::PointIterator PointSelection::selectPointsFromImage(const mySLAM::RgbdImage& img,
                                                                      const PointSelection::PointIterator& first,
                                                                      const PointSelection::PointIterator& last,
                                                                      cv::Mat& debug_idx)
  {
    const PointWithIntensityAndDepth::Point *points = (const PointWithIntensityAndDepth::Point *) img.pointcloud.data();
    const PointWithIntensityAndDepth::IntensityAndDepth *intensity_and_depth = img.acceleration.ptr<PointWithIntensityAndDepth::IntensityAndDepth>();
    PointWithIntensityAndDepth::VectorType::iterator selected_points_it = first;

    for(int y = 0; y < img.height; y++)
    {
      for(int x = 0; x < img.width; x++, points++, intensity_and_depth++)
      {
        if(predicate_.isPointOK(x, y, points->z, intensity_and_depth->idx, intensity_and_depth->idy, intensity_and_depth->zdx, intensity_and_depth->zdy))
        {
          selected_points_it->point = *points;
          selected_points_it->intensity_and_depth = *intensity_and_depth;

          selected_points_it++;

          if(selected_points_it == last)
            return selected_points_it;
        }
      }
    }

    return selected_points_it;
  }
  // end PointSelection function

  // begin Storage function
  PointSelection::Storage::Storage() :
    points(),
    points_end(points.end()),
    is_cached(false)
  {}

  void PointSelection::Storage::allocate(size_t max_points)
  {
    if(points.size() < max_points)
      points.resize(max_points);
  }
  // end Storage function
}
