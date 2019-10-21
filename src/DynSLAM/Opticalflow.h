#ifndef OpticalFlow_H
#define OpticalFlow_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "Utils.h"

using namespace std;
using namespace cv;
class Opticalflow
{
public:
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 * @param [dx] initial shift dx
 * @param [dy] initial shift dy
 */
static void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<Point2f> &kp1,
        vector<Point2f> &kp2,
        vector<bool> &success,
        bool inverse = false,
        float dx = 0,
        float dy = 0
);


// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 * @param [dx] initial shift dx
 * @param [dy] initial shift dy
 *
 */
static void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<Point2f> &kp1,
        vector<Point2f> &kp2,
        vector<bool> &success,
        bool inverse = false,
        float dx = 0,
        float dy = 0
);



/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
static inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}






};

#endif
