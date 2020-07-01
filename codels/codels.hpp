/*
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                                  Martin Jacquet - June 2020
 */
#include "acarucotag.h"

#include "arucotag_c_types.h"

#include <opencv2/opencv.hpp>
#include "opencv2/aruco.hpp"

#include <iostream>

using namespace std;
using namespace cv;

struct  arucotag_calib {
    Mat K = Mat::zeros(Size(3,3), CV_32F);
    Mat D = Mat::zeros(Size(1,5), CV_32F);
};

struct arucotag_detector {
    Mat frame;
    vector<int> ids;
    vector<vector<Point2f>> corners;
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
};
