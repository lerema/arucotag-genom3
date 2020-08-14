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

#include "codels.hpp"

/* --- Task detect ------------------------------------------------------ */


/** Codel detect_start of task detect.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_wait.
 */
genom_event
detect_start(arucotag_ids *ids, const genom_context self)
{
    ids->length = 0;
    ids->tags = new arucotag_detector();
    ids->calib = new arucotag_calib();
    ids->log = new arucotag_log_s();

    return arucotag_wait;
}


/** Codel detect_wait of task detect.
 *
 * Triggered by arucotag_wait.
 * Yields to arucotag_pause_wait, arucotag_detect.
 */
genom_event
detect_wait(float length, const arucotag_intrinsics *intrinsics,
            const arucotag_frame *frame, arucotag_calib **calib,
            const genom_context self)
{
    if (intrinsics->read(self) == genom_ok && intrinsics->data(self) &&
        frame->read(self) == genom_ok && frame->data(self) &&
        frame->data(self)->pixels._length > 0 &&
        length > 0)
    {
        (*calib)->K.at<float>(0,0) = intrinsics->data(self)->calib._buffer[0];
        (*calib)->K.at<float>(1,1) = intrinsics->data(self)->calib._buffer[1];
        (*calib)->K.at<float>(0,1) = intrinsics->data(self)->calib._buffer[2];
        (*calib)->K.at<float>(0,2) = intrinsics->data(self)->calib._buffer[3];
        (*calib)->K.at<float>(1,2) = intrinsics->data(self)->calib._buffer[4];
        (*calib)->K.at<float>(2,2) = 1;
        (*calib)->D.at<float>(0,0) = intrinsics->data(self)->disto._buffer[0];
        (*calib)->D.at<float>(1,1) = intrinsics->data(self)->disto._buffer[1];
        (*calib)->D.at<float>(0,2) = intrinsics->data(self)->disto._buffer[2];
        (*calib)->D.at<float>(1,2) = intrinsics->data(self)->disto._buffer[3];
        (*calib)->D.at<float>(0,1) = intrinsics->data(self)->disto._buffer[4];

        return arucotag_detect;
    }
    else
        return arucotag_pause_wait;
}


/** Codel detect_detect of task detect.
 *
 * Triggered by arucotag_detect.
 * Yields to arucotag_pause_detect.
 */
genom_event
detect_detect(const arucotag_frame *frame, float length,
              const arucotag_calib *calib, arucotag_detector **tags,
              const genom_context self)
{
    frame->read(self);

    // Convert frame to cv::Mat
    const uint16_t h = frame->data(self)->height;
    const uint16_t w = frame->data(self)->width;
    (*tags)->frame = Mat(Size(w, h), CV_8UC3, (void*)frame->data(self)->pixels._buffer, Mat::AUTO_STEP);

    // Detect tags in frame
    vector<vector<Point2f>> corners;
    aruco::detectMarkers((*tags)->frame, (*tags)->dict, corners, (*tags)->ids);

    // Estimate pose from corners
    if ((*tags)->ids.size() > 0) {
        vector<Vec3d> translations, rotations;
        aruco::estimatePoseSingleMarkers(corners, length, calib->K, calib->D, rotations, translations);
        if (!(*tags)->measured_state.data)
            (*tags)->measured_state = (Mat_<float>(6,1) <<
                translations[0][0],
                translations[0][1],
                translations[0][2],
                0,
                0,
                0
            );
        else
            (*tags)->measured_state = (Mat_<float>(6,1) <<
                translations[0][0],
                translations[0][1],
                translations[0][2],
                (translations[0][0] - (*tags)->measured_state.at<float>(0))/(arucotag_detect_period/1000.),
                (translations[0][1] - (*tags)->measured_state.at<float>(1))/(arucotag_detect_period/1000.),
                (translations[0][2] - (*tags)->measured_state.at<float>(2))/(arucotag_detect_period/1000.)
            );
        (*tags)->new_detection = true;
    }

    imshow("", (*tags)->frame);
    waitKey(1);

    return arucotag_pause_detect;
}
