

#include "VisoSparseSFProvider.h"

namespace instreclib {

Eigen::Matrix4d VisoToEigen(const Matrix &viso_matrix) {
  Matrix viso_matrix_copy = viso_matrix;
  //  The '~' transposes the matrix...
  //  TODO(andrei): Instead of making a copy, create the Eigen matrix and transpose that.
  return Eigen::Matrix4d((~viso_matrix_copy).val[0]);
}

void VisoSparseSFProvider::ComputeSparseSF(const ViewPair &, const ViewPair &current_view,
                                           std::vector <instreclib::segmentation::InstanceDetection> &detections) {

  using namespace std;
  using namespace dynslam::utils;

  // TODO(andrei): Is this safe? What if OpenCV represents the images differently?
  uint8_t *left_bytes = current_view.first->data;//first for left image
  uint8_t *right_bytes = current_view.second->data;//second fot right image

  // TODO(andrei): Consider caching this char version in an umbrella object, in case other parts
  // of the engine need it...
  /// This is the "paranoid" way of doing things, slow, but robust to any weird memory layout
  /// incompatibilities between libviso2 and OpenCV.
//    const auto *l_img = current_view.first;
//    const auto *r_img = current_view.second;
//    uint8_t *left_bytes = new uint8_t[l_img->rows * l_img->cols];
//    uint8_t *right_bytes = new uint8_t[r_img->rows * r_img->cols];
//    for(int i = 0; i < l_img->rows; ++i) {
//      for(int j = 0; j < l_img->cols; ++j) {
//        // We can do this because the two image must have the same dimensions anyway.
//        left_bytes[i * l_img->cols + j] = l_img->at<uint8_t>(i, j);
//        right_bytes[i * l_img->cols + j] = r_img->at<uint8_t>(i, j);
//      }
//    }

//    cv::Mat_<uint8_t> mat(l_img->rows, l_img->cols, right_bytes);
//    cv::imshow("Reconstructed preview...", mat);
//    cv::waitKey(0);



  int dims[] = {
      current_view.first->cols,
      current_view.first->rows,
      current_view.first->cols
  };


  bool viso2_success = stereo_vo_->process(left_bytes, right_bytes, dims);//matchFeatures

  if (! viso2_success) {
    matches_available_ = false;
  }
  else {
//      Tic("get matches");
    // Just marshal the data from the viso-specific format to DynSLAM format.

    std::vector<RawFlow, Eigen::aligned_allocator<RawFlow>> flow;
    for (const Matcher::p_match &match : stereo_vo_->getRawMatches()) {
    //GetRawMatches() -> not do the ransac //Use raw matches for reconstruction and identify of dynamic objects
                                           //Track state of objects
    //for (const Matcher::p_match &match : stereo_vo_->Getp_matches()) {//Modified //This for me to show the matching results
                                                                      //Results after the RANSAC
      flow.emplace_back(match.u1c, match.v1c, match.i1c, match.u2c, match.v2c, match.i2c,
                        match.u1p, match.v1p, match.i1p, match.u2p, match.v2p, match.i2p);


    }
    SparseSceneFlow new_flow = {flow};
    latest_flow_ = new_flow;


    matches_available_ = true;


//      cout << "viso2 success! " << latest_flow_.matches.size() << " matches found." << endl;
//      cout << "               " << stereo_vo_->getNumberOfInliers() << " inliers" << endl;
//      Toc();
  }
}


std::vector<double> VisoSparseSFProvider::ExtractMotion(const std::vector<RawFlow, Eigen::aligned_allocator<RawFlow>> &flow,
                                                        const std::vector<double> &initial_estimate,
                                                        std::vector<Eigen::Vector3d> normals
) const {
  // TODO-LOW(andrei): Extract helper method for this.
  std::vector<Matcher::p_match> flow_viso;
  for(const RawFlow &f : flow) {
    flow_viso.push_back(Matcher::p_match(f.prev_left(0), f.prev_left(1), f.prev_left_idx,//u,v, index of feature point
                                         f.prev_right(0), f.prev_right(1), f.prev_right_idx,
                                         f.curr_left(0), f.curr_left(1), f.curr_left_idx,
                                         f.curr_right(0), f.curr_right(1), f.curr_right_idx));
    //        left/right current/past frames.

  }

  //Added
  //Add normals
 if(!normals.empty())
    stereo_vo_->AddNormals(normals);
  //End


  //Compute estimation
  return stereo_vo_->estimateMotion(flow_viso, initial_estimate);


/*  //debug //for drawing inliers
  flow.clear();
  for (const Matcher::p_match &match : stereo_vo_->test) {
    flow.emplace_back(match.u1c, match.v1c, match.i1c, match.u2c, match.v2c, match.i2c,
                      match.u1p, match.v1p, match.i1p, match.u2p, match.v2p, match.i2p);
  }

  return result;
  //end debug
*/

}

} // namespace instreclib
