/*
The MIT License
Copyright (c) 2015 Avi Singh
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#ifndef VO_FEATURES_H
#define VO_FEATURES_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
namespace vo_features{
/**
 * @brief featureTracking this function automatically gets rid of points for which tracking fails
 * @param img_1
 * @param img_2
 * @param points1
 * @param points2
 * @param status
 */
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{
  vector<float> err;
  Size winSize=Size(21,21);
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for(std::size_t i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                  if((pt.x<0)||(pt.y<0))	{
                        status.at(i) = 0;
                  }
                  points1.erase (points1.begin() + (i - indexCorrection));
                  points2.erase (points2.begin() + (i - indexCorrection));
                  indexCorrection++;
        }
     }
}

/**
 * @brief featureDetection uses FAST to detect feature points
 * @param img_1
 * @param points1
 * @param fast_threshold
 */
void featureDetection(Mat img_1, vector<Point2f>& points1,int fast_threshold){
  vector<KeyPoint> keypoints_1;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}
}


#endif
