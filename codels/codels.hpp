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
#ifndef H_ARUCOTAG_CODELS
#define H_ARUCOTAG_CODELS

#include "acarucotag.h"

#include "arucotag_c_types.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <sys/time.h>
#include <aio.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>

#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

struct arucotag_calib {
    Matrix3d K = Matrix3d::Zero();              // intrinsic calibration matrix
    Mat K_cv = Mat::zeros(Size(3,3), CV_32F);   // opencv representation of K
    Mat D = Mat::zeros(Size(1,5), CV_32F);      // camera distortion coefs
    Vector3d B_p_C = Vector3d::Zero();          // translation from camera to body
    Matrix3d B_R_C = Matrix3d::Identity();      // rotation from body to camera
};


struct arucotag_detector {
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); // TODO give choice of dictionary
    vector<int> ids;                    // ids of detected tags
    Matrix<double,3,4> corners_marker;  // coordinates of corners in marker frame
    Mat corners_marker_cv;              // opencv reprensation of corners_marker

    void set_length(double l) {
        corners_marker <<
            -1,  1,  1, -1,
             1,  1, -1, -1,
             0,  0,  0,  0;
        corners_marker *= l;

        Matrix<double,4,3> tmp = corners_marker.transpose();
        eigen2cv(tmp, corners_marker_cv);
    }
};

static inline
Matrix3d skew(Vector3d v)
{
    Matrix3d skew;
    skew << 0,-v(2),v(1),  v(2),0,-v(0),  -v(1),v(0),0;
    return skew;
}

struct arucotag_log_s {
    aiocb req;
    char buffer[4096];
    bool pending, skipped;
    uint32_t decimation;
    size_t missed, total;
    arucotag_log_s() {
        this->req.aio_fildes = -1;
        this->req.aio_buf = this->buffer;
    }
    # define arucotag_logfmt	"%g "
    # define arucotag_log_header                                            \
        "ts frame i px py x y z roll pitch yaw sxx sxy syy sxz syz szz sqww sqwx sqxx sqwy sqxy sqyy sqwz sqxz sqyz sqzz "
    # define arucotag_log_fmt                                               \
        "%d.%09d %i %s "                                                    \
        "%d %d "                                                            \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt
};


static inline genom_event
arucotag_e_sys_error(const char *s, genom_context self)
{
    arucotag_e_sys_detail d;
    char buf[64], *p;
    d.code = errno;

    p = strerror_r(d.code, buf, sizeof(buf));
    snprintf(d.what, sizeof(d.what), "%s%s%s", s ? s : "", s ? ": " : "", p);
    return arucotag_e_sys(&d, self);
}

#endif /* H_ARUCOTAG_CODELS */
