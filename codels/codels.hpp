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
#include <opencv2/aruco.hpp>

#include <iostream>
#include <sys/time.h>
#include <iostream>
#include <aio.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>

using namespace std;
using namespace cv;

struct  arucotag_calib {
    Mat K = Mat::zeros(Size(3,3), CV_32F);
    Mat D = Mat::zeros(Size(1,5), CV_32F);
    Mat B_R_C;
    Mat B_t_C;
};

struct arucotag_detector {
    Mat frame;
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    vector<int> valid_ids;
    vector<Mat> raw_meas, transformed_meas;
};

struct arucotag_log_s {
    struct aiocb req;
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
        "ts i C_x C_y C_z W_x W_y W_z "
    # define arucotag_log_fmt                                               \
        "%ld.%09ld %i "                                                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt                     \
        arucotag_logfmt arucotag_logfmt arucotag_logfmt
};

static inline genom_event
arucotag_e_sys_error(const char *s, genom_context self)
{
    arucotag_e_sys_detail d;
    size_t l = 0;

    d.code = errno;
    if (s) {
        strncpy(d.what, s, sizeof(d.what) - 3);
        l = strlen(s);
        strcpy(d.what + l, ": ");
        l += 2;
    }
    if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) { /* ignore error*/; }
    return arucotag_e_sys(&d, self);
}
