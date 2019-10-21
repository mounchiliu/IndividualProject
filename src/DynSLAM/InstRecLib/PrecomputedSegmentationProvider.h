

#ifndef INSTRECLIB_PRECOMPUTEDSEGMENTATIONPROVIDER_H
#define INSTRECLIB_PRECOMPUTEDSEGMENTATIONPROVIDER_H

#include <memory>

#include "InstanceSegmentationResult.h"
#include "SegmentationProvider.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

namespace instreclib {
namespace segmentation {

/// \brief Reads pre-existing frame segmentations from the disk, instead of computing them
/// on-the-fly.
class PrecomputedSegmentationProvider : public SegmentationProvider {
 public:
  PrecomputedSegmentationProvider(const std::string &seg_folder, int frame_offset, float scale)
      : seg_folder_(seg_folder),
        frame_idx_(frame_offset),
        dataset_used(&kPascalVoc2012),
        last_seg_preview_(nullptr),
        input_scale_(scale) { }

  ~PrecomputedSegmentationProvider() override { delete last_seg_preview_; }

  std::shared_ptr<InstanceSegmentationResult> SegmentFrame(const cv::Mat3b &view) override;

  const cv::Mat3b *GetSegResult() const override;

  cv::Mat3b *GetSegResult() override;

  std::shared_ptr<InstanceSegmentationResult> ReadSegmentation(int frame_idx);

 protected:
  /// \brief For the segmentation from the given file path, loads all available file containing
  /// detection information (class, bounding box, etc.) and instance masks.
  std::vector<InstanceDetection> ReadInstanceInfo(const std::string &base_img_fpath);

  //Added
  ///Combine boxes (There are some boxes detected that belong to the same object, we need to combine them
  std::vector<InstanceDetection> CombineBoxes(std::vector<InstanceDetection> instance_detections);


 private:
  const std::string seg_folder_;
  int frame_idx_;
  const SegmentationDataset *dataset_used;
  cv::Mat3b *last_seg_preview_;
  // Used when evaluating low-res input.
  const float input_scale_;

};

}  // namespace segmentation
}  // namespace instreclib

#endif  // INSTRECLIB_PRECOMPUTEDSEGMENTATIONPROVIDER_H
