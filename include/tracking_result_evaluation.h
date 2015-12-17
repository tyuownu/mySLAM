#ifndef _TRACKING_RESULT_EVALUATION_H
#define _TRACKING_RESULT_EVALUATION_H

#include <boost/shared_ptr.hpp>
#include <dense_tracking.h>

namespace mySLAM
{
  class TrackingResultEvaluation
  {
  public:
    typedef boost::shared_ptr<TrackingResultEvaluation> Ptr;
    typedef boost::shared_ptr<const TrackingResultEvaluation> ConstPtr;

    virtual ~TrackingResultEvaluation() {}
    virtual void add(const DenseTracker::Result& r);
    virtual double ratioWithFirst(const DenseTracker::Result& r) const;
    virtual double ratioWithAverage(const DenseTracker::Result& r) const;
  protected:
    TrackingResultEvaluation(double first);
    virtual double value(const DenseTracker::Result& r) const = 0;
  private:
    double first_, average_, n_;
  };

}  // end namespace mySLAM


#endif
