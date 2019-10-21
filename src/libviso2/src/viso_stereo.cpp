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

#include <cassert>
#include <iomanip>
#include "viso_stereo.h"

using namespace std;

VisualOdometryStereo::VisualOdometryStereo (parameters param) : param(param), VisualOdometry(param) {
  matcher->setIntrinsics(param.calib.f,param.calib.cu,param.calib.cv,param.base);
}

VisualOdometryStereo::~VisualOdometryStereo() {
}



bool VisualOdometryStereo::process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace,
                                    uint8_t *I1_p,uint8_t *I2_p
                                    ) {



  matcher->pushBack(I1,I2,dims,replace); //Get images, e.g. current, previous, left, right ....

//Find matches

//  dynslam::utils::Tic("VOS::process");

  // bootstrap motion estimate if invalid
  Tr_valid = true;
  if (!Tr_valid) {
    printf("viso2 (%s): Tr was not valid; doing full feature match\n", __FILE__);
    matcher->matchFeatures(2);//Find matches and also do the RANSAC


    matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
    p_matched = matcher->getMatches();

    updateMotion();
//    dynslam::utils::Toc();
  }
  
  // match features and update motion

  if (Tr_valid) matcher->matchFeatures(2,&Tr_delta);// // transformation (previous -> current frame)
  else          matcher->matchFeatures(2);
  matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
  p_matched = matcher->getMatches();


  bool updateOk = updateMotion();
//  dynslam::utils::Toc();

  return updateOk;
}



vector<double> VisualOdometryStereo::estimateMotion(
    vector<Matcher::p_match> p_matched,
    const vector<double> &initial_estimate
) {
  
  // return value
  bool success = true;
  
  // compute minimum distance for RANSAC samples
  double width=0,height=0;
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1c>width)  width  = it->u1c;
    if (it->v1c>height) height = it->v1c;
  }
  double min_dist = min(width,height)/3.0;//unused
  
  // get number of matches
  int32_t N  = p_matched.size();
  if (N < 6) {
//    cerr << "Insufficient matches between images: " << N << " instead of 6, the minimum required."
//         << endl;
    return vector<double>();
  }

  // TODO-PERF(andrei): We can just pre-allocate these on init for some large enough N. If full-size
  // matching is used, N can easily be ~350 => ~120kb. Ok, not too much.
  // allocate dynamic memory
  X          = new double[N];
  Y          = new double[N];
  Z          = new double[N];
  J          = new double[4*N*6];
  //Added
  J2          = new double[4*N*6];
  p_predict  = new double[4*N];
  p_observe  = new double[4*N];
  p_residual = new double[4*N];
  //Added
  p_residual2 = new double[4*N];

  // project matches of previous image into 3d
  for (int32_t i=0; i<N; i++) {
    //world coordinate
    double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
    X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
    Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
    Z[i] = param.calib.f*param.base/d;
  }//3D points

  // loop variables
  vector<double> tr_delta;
  vector<double> tr_delta_curr;
  tr_delta_curr.resize(6);
  
  // clear parameter vector
  inliers.clear();

  // initial RANSAC estimate
  for (int32_t k=0;k<param.ransac_iters;k++) {

    // draw random sample set
      //3 randomly drawn correspondence
    vector<int32_t> active = getRandomSample(N,3); //Get random 3 matches from all the matches

    // clear parameter vector
    for (int32_t i=0; i<6; i++) {
      tr_delta_curr[i] = initial_estimate[i];
    }

    // minimize reprojection errors of the previous frame's keypoints onto the current frame
    VisualOdometryStereo::result result = UPDATED;
    int32_t iter=0;
//    int MAX_ITERS = 50;   // original was 20, and usually took all 20 to converge with 0-init.
    int MAX_ITERS = 10;     // seems to work quite well with warm starts
    while (result==UPDATED) {
      result = updateParameters(p_matched, active, tr_delta_curr, 1.0, 1e-6);

      if (iter++ > MAX_ITERS || result == CONVERGED) {
//        cout << "Break @ " << iter << " iterations in libviso motion estimation UPDATE." << endl;
        break;
      }
    } //Converge using this three correspondences

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      vector<int32_t> inliers_curr = getInlier(p_matched, tr_delta_curr);
      if (inliers_curr.size()>inliers.size()) { //If find more inliers, using this estimation
        inliers = inliers_curr;
        tr_delta = tr_delta_curr;
      }
    }
  }
  
  // final optimization (refinement)
  //Re-estimate using all inliers
  if (inliers.size()>=6) {
    int32_t iter=0;
    VisualOdometryStereo::result result = UPDATED;
    while (result==UPDATED) {     
      result = updateParameters(p_matched,inliers,tr_delta, 1, 1e-8);
//      result = updateParameters(p_matched,inliers,tr_delta, 0.25, 1e-8);
      if (iter++ > 250 || result == CONVERGED) {
//        cout << "Break @ " << iter << " iterations in libviso motion estimation REFINEMENT." << endl;
        break;
      }
    }

    // not converged
    if (result!=CONVERGED) {
      success = false;
    }
    else {
      double res_sum = 0.0;
      for(int i = 0; i < 4 * N; ++i) {
        res_sum += p_residual[i] * p_residual[i];

      }


      double residual_frobenius = sqrt(res_sum / N / 4);
      cout << "Final residual RMSE: " << setprecision(6) << residual_frobenius << endl;

/*
      //Added
      //Debug
      std::vector<Matcher::p_match> test;
      for (int i=0; i<inliers.size();i++) {
          test.push_back(p_matched[inliers[i]]);

      }
      //this->test = test;
      //end debug
*/

    }

  // not enough inliers
  } else {
    cerr << "Insufficient final inliers in viso! (" << inliers.size() << " < 6)" << endl;
    success = false;
  }

  myNormals_.clear();
  // release dynamic memory
  delete[] X;
  delete[] Y;
  delete[] Z;
  delete[] J;
  delete[] J2;
  delete[] p_predict;
  delete[] p_observe;
  delete[] p_residual;
  delete[] p_residual2;
  
  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return vector<double>();
}




vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched,vector<double> &tr) {

  // mark all observations active
  vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(p_matched, active);
  computeResidualsAndJacobian(tr, active);

  // compute inliers
  vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
      if(myNormals_.empty()){
          if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
              pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2)
                  < param.inlier_threshold*param.inlier_threshold){
              inliers.push_back(i);
          }
      }
      else {//for dynamic objects we get some normal vectors
          if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
              pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2)
              < param.inlier_threshold*param.inlier_threshold -2//2
                  ){//param.inlier_threshold*param.inlier_threshold
              //modified add normals
              if(abs(p_residual2[4*i+0])<=1){//0.1 //1 //0.5
                inliers.push_back(i);


              }
          }
      }
  return inliers;
}

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(vector<Matcher::p_match> &p_matched,vector<int32_t> &active,vector<double> &tr,double step_size,double eps) {
  
  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;
  

  //active -> random index of feature points

  // extract observations and compute predictions
  computeObservations(p_matched,active);//get observations, put it in p_matched
  computeResidualsAndJacobian(tr,active);
  //Here only get 3 random matches for doing PnP,
  //because for one match we get 2D points of current left/right
  //                                          previous left/right
  //and the  3D points
  //That is we have 12 matches for PnP.
  //Six variables, 3 vaiables for rotation, 3 variables for translation
  //THen solve the problem with GN



  // init
  Matrix A(6,6); // = (J^T * J) //J -> 6 * 4
  Matrix B(6,1); // = (J^T * r) //r -> 4x1(left_c,right_c,left_p,right_p)

  // fill matrices A and B
  for (int32_t m=0; m<6; m++) {
    for (int32_t n=0; n<6; n++) {
      double a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
        // i-th row and m-th column, i-th row and n-th column
          a += J[i * 6 + m] * J[i * 6 + n] + J2[i * 6 + m] * J2[i * 6 + m]; //Modified
        }
        A.val[m][n] = a;
      }

      double b = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
        b += J[i * 6 + m] * (p_residual[i]) - J2[i * 6 + m]*p_residual2[i]; //Modifed
    }

    B.val[m][0] = b;
  }

  // Instead of solving (inv(A) * B) it's much more efficient to solve Ax = B, since it doesn't
  // involve inverting a matrix. Solving Ax = B solves
  // In essence, this part computes the Gauss-Newton parameter update.
  // perform elimination Ax = B (stores result in B)
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<6; m++) {
      // Update each parameter of the transform
      tr[m] += step_size * B.val[m][0];
      //tr size 6
      if (fabs(B.val[m][0])>eps) {
        converged = false;
      }
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}

void VisualOdometryStereo::computeObservations(vector<Matcher::p_match> &p_matched,vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
    p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
  }
}

void VisualOdometryStereo::computeResidualsAndJacobian(vector<double> &tr,vector<int32_t> &active) {

  // extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];//Rotation
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];//Translation

  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  // compute rotation matrix and derivatives
  //Rotation matrix
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;

  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;

  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;

  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
    
    // weighting
    // (Lower weights towards the edges of the image; meant to increase robustness to
    //  calibration errors.)
    double weight = 1.0;
    if (param.reweighting)
      weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);//lower weight for features in the edge
    
    // compute 3d point in current right coordinate system
    X2c = X1c-param.base;

    //Objective function: observation in 2d - prediction from 3d in 2d
    // for all parameters do
    for (int32_t j = 0; j < 6; j++) {

        //Added
        int J2_ = 0;

      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      // We're optimizing the pose (6 elements: 3 rot, 3 trans), so we want the derivative of the
      // 3D point wrt each of these 6 parameters.
        switch (j) {
          // derivative of the ith observation w.r.t., r_1 //rx
          case 0:
            X1cd = 0;//non-related to rx -> 0
            Y1cd = rdrx10 * X1p + rdrx11 * Y1p + rdrx12 * Z1p;
            Z1cd = rdrx20 * X1p + rdrx21 * Y1p + rdrx22 * Z1p;
            J2_ = 0;
            break;
            // derivative w.r.t., r_2 //ry
          case 1:
            X1cd = rdry00 * X1p + rdry01 * Y1p + rdry02 * Z1p;
            Y1cd = rdry10 * X1p + rdry11 * Y1p + rdry12 * Z1p;
            Z1cd = rdry20 * X1p + rdry21 * Y1p + rdry22 * Z1p;
            J2_ = 0;
            break;
            // derivative w.r.t., r_3 //rz
          case 2:
            X1cd = rdrz00 * X1p + rdrz01 * Y1p;
            Y1cd = rdrz10 * X1p + rdrz11 * Y1p;
            Z1cd = rdrz20 * X1p + rdrz21 * Y1p;
            J2_ = 0;
            break;
          // derivative w.r.t., t_1
          case 3:
              X1cd = 1 ; Y1cd = 0; Z1cd = 0; break;
              for (Eigen::Vector3d n: myNormals_) {
                  J2_ += n.x();
              }


          // derivative w.r.t., t_2
          case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
            for (Eigen::Vector3d n: myNormals_) {
                J2_ += n.y();
            }
          // derivative w.r.t., t_3
          case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
            for (Eigen::Vector3d n: myNormals_) {
                J2_ += n.z();
            }
          default:
            assert("Invalid parameter ID. Only 0..5 supported.");
            return;   // Makes the data flow analysis not complain.
        }

        // set jacobian entries (project via K)
        // (Each observation has four components: the left-u, left-v, right-u, right-v, and each
        //  gradient has 6 elements, since our pose has 6 parameters; therefore, each observation
        //  leads to 6 * 4 = 24 elements in the Jacobian.)
        // left u'
        //col 6 row
        J[(4 * i + 0) * 6 + j] = weight * param.calib.f * (X1cd * Z1c - X1c * Z1cd) / (Z1c * Z1c);
        J2[(4 * i + 0) * 6 + j] = /*weight **/(J2_);
        // left v'
        J[(4 * i + 1) * 6 + j] = weight * param.calib.f * (Y1cd * Z1c - Y1c * Z1cd) / (Z1c * Z1c);
        J2[(4 * i + 1) * 6 + j] = /*weight **/(J2_);
        // right u'
        J[(4 * i + 2) * 6 + j] = weight * param.calib.f * (X1cd * Z1c - X2c * Z1cd) / (Z1c * Z1c);
        J2[(4 * i + 2) * 6 + j] = /*weight **/(J2_);
        // right v'
        J[(4 * i + 3) * 6 + j] = weight * param.calib.f * (Y1cd * Z1c - Y1c * Z1cd) / (Z1c * Z1c);
        J2[(4 * i + 3) * 6 + j] =/*weight **/(J2_);//0.2 weight for this term of cost
      }

      // set prediction (project via K)
      // No derivatives involved here: this is just the standard K * dehomog(X)
      p_predict[4 * i + 0] = param.calib.f * X1c / Z1c + param.calib.cu; // left u
      p_predict[4 * i + 1] = param.calib.f * Y1c / Z1c + param.calib.cv; // left v
      p_predict[4 * i + 2] = param.calib.f * X2c / Z1c + param.calib.cu; // right u
      p_predict[4 * i + 3] = param.calib.f * Y1c / Z1c + param.calib.cv; // right v

      // set residuals
      p_residual[4 * i + 0] = weight * (p_observe[4 * i + 0] - p_predict[4 * i + 0]);
      p_residual[4 * i + 1] = weight * (p_observe[4 * i + 1] - p_predict[4 * i + 1]);
      p_residual[4 * i + 2] = weight * (p_observe[4 * i + 2] - p_predict[4 * i + 2]);
      p_residual[4 * i + 3] = weight * (p_observe[4 * i + 3] - p_predict[4 * i + 3]);



      p_residual2[4 * i + 0] = 0;
      p_residual2[4 * i + 1] = 0;
      p_residual2[4 * i + 2] = 0;
      p_residual2[4 * i + 3] = 0;
      if(!myNormals_.empty()){
          //set residuals for the second function and Jacobian

          for (Eigen::Vector3d n: myNormals_) {
            p_residual2[4 * i + 0] += /*weight **/(n.x()*tx +  n.y()*ty +  n.z()*tz);
          }

          p_residual2[4 * i + 1] = p_residual2[4 * i + 0];
          p_residual2[4 * i + 2] = p_residual2[4 * i + 0];
          p_residual2[4 * i + 3] = p_residual2[4 * i + 0];
      }


    }
  }
