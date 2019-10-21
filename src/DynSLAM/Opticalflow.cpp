#include "Opticalflow.h"
#include <iostream>

#include <opencv2/opencv.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

void Opticalflow::OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<Point2f> &kp1,
        vector<Point2f> &kp2,
        vector<bool> &success,
        bool inverse,//inverse method of optical flow
        float dx,
        float dy
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    //success.clear();
    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        //float dx = 0, dy = 0; // dx,dy need to be estimated

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded


        if (have_initial) {
            dx = kp2[i].x - kp.x;
            dy = kp2[i].y - kp.y;
         }


        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.x + dx <= half_patch_size || kp.x + dx >= img1.cols - half_patch_size ||
                kp.y + dy <= half_patch_size || kp.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)

                    double I1,I2;
                    Eigen::Vector2d GradientI;

                    I1 = GetPixelValue(img1,kp.x+x,kp.y+y);
                    I2 = GetPixelValue(img2,kp.x+x+dx,kp.y+y+dy);
                    double error = I2-I1;
                    Eigen::Vector2d J;  // Jacobian

                    if (inverse == false) {
                        // Forward Jacobian
                      GradientI(0) = GetPixelValue(img2,kp.x+x+dx,kp.y+y+dy)
                                     - GetPixelValue(img2,kp.x+x+dx-1,kp.y+y+dy);//x
                      GradientI(1) = GetPixelValue(img2,kp.x+x+dx,kp.y+y+dy)
                                     - GetPixelValue(img2,kp.x+x+dx,kp.y+y+dy-1);//y

                      J = GradientI;


                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                      GradientI(0) = 0.5*(GetPixelValue(img1,kp.x+x+1,kp.y+y)
                          - GetPixelValue(img1,kp.x+x-1,kp.y+y));//x
                      GradientI(1) = 0.5*(GetPixelValue(img1,kp.x+x,kp.y+y+1)
                          - GetPixelValue(img1,kp.x+x,kp.y+y-1));//y

                      J = GradientI;



                    }

                    // compute H, b and set cost;
                    H += J*J.transpose();
                    b += -J*error;
                    cost += error*error;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            Eigen::Vector2d update;

            update = H.ldlt().solve(b);

            //update = H.inverse() * b;
            if (update.squaredNorm() < 1 ) { //small update -> converge -> break //1
                //cout<<"break"<<update.squaredNorm()<<endl;
                // update dx, dy
                dx += update[0];
                dy += update[1];
                lastCost = cost;
                succ = true;
                break;
            }


/*            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }*/
            if (iter > 0 && cost > lastCost) {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);
        //cout<<"a"<<endl;

         //set kp2
        if (have_initial) {
            kp2[i] = kp + Point2f(dx, dy);
        } else {
            Point2f tracked = kp;
            tracked += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}


void Opticalflow::OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<Point2f> &kp1,
        vector<Point2f> &kp2,
        vector<bool> &success,
        bool inverse,
        float dx,
        float dy) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {

      Mat temp1, temp2;
      if(i==0){
        pyr1.push_back(img1);
        pyr2.push_back(img2);
      }
      else{

        pyrDown(pyr1[pyr1.size()-1],temp1);
        pyrDown(pyr2[pyr2.size()-1],temp2);
        pyr1.push_back(temp1);
        pyr2.push_back(temp2);

      }
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    for (int i = pyramids-1; i >= 0; i--) {
      Point2f kp_temp;
      vector<Point2f> kp1_temp;

      for(int j=0; j<kp1.size();j++)
      {
        kp_temp.x = kp1[j].x*scales[i];
        kp_temp.y = kp1[j].y*scales[i];
        kp1_temp.push_back(kp_temp);
        dx = dx * scales[i];
        dy = dy * scales[i];

        if(i!=pyramids-1)
        {
          kp2[j].x = kp2[j].x*(1/pyramid_scale);
          kp2[j].y = kp2[j].y*(1/pyramid_scale);

        }

      }

      if(i == pyramids-1)
        OpticalFlowSingleLevel(pyr1[i],pyr2[i],kp1_temp,kp2,success,inverse,dx,dy);
      else{
        OpticalFlowSingleLevel(pyr1[i],pyr2[i],kp1_temp,kp2,success,inverse);
      }

    }

    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2

}






