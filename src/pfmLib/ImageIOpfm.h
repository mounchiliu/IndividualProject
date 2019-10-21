/*  Name:
 *      ImageIOpfm.h
 *
 *  Description:
 *      Used to read/write pfm images to and from 
 *      opencv Mat image objects
 *      
 *      Works with PF color pfm files and Pf grayscale
 *      pfm files
 */


#ifndef __ImageIOpfm_H_INCLUDED__ 
#define __ImageIOpfm_H_INCLUDED__ 

#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <cmath>

using namespace std;

int ReadFilePFM(cv::Mat &im, string path, bool verbose = false);
int WriteFilePFM(const cv::Mat &im, string path, float scalef = 1.0f/255.0f, bool verbose = false);

#endif