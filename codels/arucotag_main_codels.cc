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

/* --- Task main -------------------------------------------------------- */


/** Codel main_start of task main.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_wait.
 */
genom_event
main_start(arucotag_ids *ids, const genom_context self)
{
    ids->length = 0;
    ids->tags = new arucotag_detector();
    ids->calib = new arucotag_calib();

    return arucotag_wait;
}


/** Codel main_wait of task main.
 *
 * Triggered by arucotag_wait.
 * Yields to arucotag_pause_wait, arucotag_detect.
 */
genom_event
main_wait(float length, const arucotag_intrinsics *intrinsics,
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


/** Codel main_detect of task main.
 *
 * Triggered by arucotag_detect.
 * Yields to arucotag_pause_detect.
 */
genom_event
main_detect(const arucotag_frame *frame, float length,
            const arucotag_calib *calib, arucotag_detector **tags,
            const arucotag_pose *pose, const genom_context self)
{
    frame->read(self);

    const uint16_t h = frame->data(self)->height;
    const uint16_t w = frame->data(self)->width;
    (*tags)->frame = Mat(Size(w, h), CV_8UC3, (void*)frame->data(self)->pixels._buffer, Mat::AUTO_STEP);

    aruco::detectMarkers((*tags)->frame, (*tags)->dict, (*tags)->corners, (*tags)->ids);

    if ((*tags)->ids.size() > 0) {
        vector<Vec3d> rotations, translations;
        aruco::estimatePoseSingleMarkers((*tags)->corners, length, calib->K, calib->D, rotations, translations);

        pose->data(self)->pos._value.x = translations[0][0];
        pose->data(self)->pos._value.y = translations[0][1];
        pose->data(self)->pos._value.z = translations[0][2];
        pose->data(self)->pos._present = true;

        pose->write(self);
    }

    return arucotag_pause_detect;
}
