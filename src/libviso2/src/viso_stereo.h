/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/



#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"
#include <Eigen/StdVector>

class VisualOdometryStereo : public VisualOdometry {

public:

  // stereo-specific parameters (mandatory: base)
  struct parameters : public VisualOdometry::parameters {
    double  base;             // baseline (meters)
    int32_t ransac_iters;     // number of RANSAC iterations
    double  inlier_threshold; // fundamental matrix inlier threshold
    bool    reweighting;      // lower border weights (more robust to calibration errors)
    parameters () {
      base             = 1.0;
      ransac_iters     = 200;
      inlier_threshold = 2.0; //modified //2.0
      reweighting      = true;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryStereo (parameters param);

  // deconstructor
  virtual ~VisualOdometryStereo ();

  // process a new images, push the images back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
  //         I2 ........ pointer to rectified right image (uint8, row-aligned)
  //         dims[0] ... width of I1 and I2 (both must be of same size)
  //         dims[1] ... height of I1 and I2 (both must be of same size)
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current images with I1 and I2, without copying last current
  //                     images to previous images internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if an error occured
  bool process (uint8_t *I1_c,uint8_t *I2_c,int32_t* dims,bool replace=false,
                uint8_t *I1_p = 0 ,uint8_t *I2_p = 0
                );

  const std::vector<Matcher::p_match> getRawMatches() const {
    return matcher->getRawMatches();
  }


  //Added
  const std::vector<Matcher::p_match> GetRawDeleted_matches()
  {return matcher->GetDeletedRawMatches();}


  using VisualOdometry::process;

  // Note: made public for experimental DynSLAM purposes.
  // The method uses RANSAC to robustly fit a 6-DoF transform over 3 4-way matches. At every RANSAC
  // iteration, 3 (curr-left, curr-right, prev-left, prev-right) tuples are sampled and the 6-DoF
  // transform is fit using Gauss-Newton, minimising the sum of squared reprojection errors in the
  // left and right current frames, of the 3D points triangulated from the previous frame.
  std::vector<double>  estimateMotion(std::vector<Matcher::p_match> p_matched,
                                      const std::vector<double> &initial_guess);



private:
  enum                 result { UPDATED, FAILED, CONVERGED };
  result               updateParameters(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
  void                 computeObservations(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active);
  void                 computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);
  std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched,std::vector<double> &tr);

  double *X,*Y,*Z;    // 3d points
  double *p_residual; // residuals (p_residual=p_observe-p_predict)
  //Added
  double *p_residual2;

  // parameters
  parameters param;
};

#endif // VISO_STEREO_H
