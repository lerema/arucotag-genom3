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
detect_start(arucotag_ids *ids, const arucotag_pose *pose,
             const genom_context self)
{
    // Init IDS fields
    ids->length = 0;
    ids->tags = new arucotag_detector();
    ids->calib = new arucotag_calib();
    ids->log = new arucotag_log_s();

    // Init outport
    pose->data(self)->pos._present = true;
    pose->data(self)->att._present = false;
    pose->data(self)->vel._present = true;
    pose->data(self)->avel._present = false;
    pose->data(self)->acc._present = false;
    pose->data(self)->aacc._present = false;

    double sigma_x = 0.05;
    double sigma_y = 0.05;
    double sigma_z = 0.05;
    pose->data(self)->pos_cov._value.cov[0] = sigma_x*sigma_x;
    pose->data(self)->pos_cov._value.cov[1] = sigma_x*sigma_y;
    pose->data(self)->pos_cov._value.cov[2] = sigma_x*sigma_z;
    pose->data(self)->pos_cov._value.cov[3] = sigma_y*sigma_y;
    pose->data(self)->pos_cov._value.cov[4] = sigma_y*sigma_z;
    // pose->data(self)->pos_cov._value.cov[5] = sigma_z*sigma_z;
    pose->data(self)->pos_cov._value.cov[5] = 0;

    // Init extrinsic calibration (hardcoded)
    ids->calib->B_R_C = (Mat_<float>(3,3) <<
        1, 0, 0,
        0,-1, 0,
        0, 0,-1
    );
    ids->calib->B_t_C = (Mat_<float>(3,1) <<
        0,
        0,
        -0.02
    );

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
 * Yields to arucotag_log, arucotag_pause_detect.
 */
genom_event
detect_detect(const arucotag_frame *frame, float length,
              const arucotag_calib *calib, arucotag_detector **tags,
              const arucotag_drone *drone, const arucotag_pose *pose,
              const genom_context self)
{
    frame->read(self);
    drone->read(self);
    struct timeval tv;
    gettimeofday(&tv, NULL);

    // Convert frame to cv::Mat
    const uint16_t h = frame->data(self)->height;
    const uint16_t w = frame->data(self)->width;
    (*tags)->frame = Mat(Size(w, h), CV_8UC3, (void*)frame->data(self)->pixels._buffer, Mat::AUTO_STEP);

    // Detect tags in frame
    vector<int> ids;
    vector<vector<Point2f>> corners;
    aruco::detectMarkers((*tags)->frame, (*tags)->dict, corners, ids);

    // Estimate pose from corners
    if (ids.size() > 0)
    {
        vector<Vec3d> translations, rotations;
        aruco::estimatePoseSingleMarkers(corners, length, calib->K, calib->D, rotations, translations);
        (*tags)->raw_meas = (Mat_<float>(3,1) <<
            translations[0][0],
            translations[0][1],
            translations[0][2]
        );

        // Convert to world frame
        double qw = drone->data(self)->att._value.qw;
        double qx = drone->data(self)->att._value.qx;
        double qy = drone->data(self)->att._value.qy;
        double qz = drone->data(self)->att._value.qz;
        Mat W_R_B = (Mat_<float>(3,3) <<
            1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
                2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw,
                2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy
        );
        Mat W_t_B = (Mat_<float>(3,1) <<
            drone->data(self)->pos._value.x,
            drone->data(self)->pos._value.y,
            drone->data(self)->pos._value.z
        );
        (*tags)->transformed_meas = W_R_B * (calib->B_R_C * (*tags)->raw_meas + calib->B_t_C ) + W_t_B;

        // Publish
        pose->data(self)->pos._value.x = (*tags)->transformed_meas.at<float>(0);
        pose->data(self)->pos._value.y = (*tags)->transformed_meas.at<float>(1);
        pose->data(self)->pos._value.z = 0;

        pose->data(self)->vel._value.vx = 0;
        pose->data(self)->vel._value.vy = 0;
        pose->data(self)->vel._value.vz = 0;

        pose->data(self)->ts.sec = tv.tv_sec;
        pose->data(self)->ts.nsec = tv.tv_usec * 1000;
        pose->write(self);

        return arucotag_log;
    }

    return arucotag_pause_detect;
}


/** Codel detect_log of task detect.
 *
 * Triggered by arucotag_log.
 * Yields to arucotag_pause_detect.
 */
genom_event
detect_log(const arucotag_detector *tags, arucotag_log_s **log,
           const genom_context self)
{
    if (*log)
    {
        if ((*log)->req.aio_fildes >= 0)
        {
            (*log)->total++;
            if ((*log)->total % (*log)->decimation == 0)
                if ((*log)->pending)
                {
                    if (aio_error(&(*log)->req) != EINPROGRESS)
                    {
                        (*log)->pending = false;
                        if (aio_return(&(*log)->req) <= 0)
                        {
                            warn("log");
                            close((*log)->req.aio_fildes);
                            (*log)->req.aio_fildes = -1;
                        }
                    }
                    else
                    {
                        (*log)->skipped = true;
                        (*log)->missed++;
                    }
                }
        }
        if ((*log)->req.aio_fildes >= 0 && !(*log)->pending)
        {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            (*log)->req.aio_nbytes = snprintf(
                (*log)->buffer, sizeof((*log)->buffer),
                "%s" arucotag_log_fmt "\n",
                (*log)->skipped ? "\n" : "",
                tv.tv_sec, tv.tv_usec*1000,
                tags->raw_meas.at<float>(0),
                tags->raw_meas.at<float>(1),
                tags->raw_meas.at<float>(2),
                tags->transformed_meas.at<float>(0),
                tags->transformed_meas.at<float>(1),
                tags->transformed_meas.at<float>(2)
            );
            if (aio_write(&(*log)->req))
            {
                warn("log");
                close((*log)->req.aio_fildes);
                (*log)->req.aio_fildes = -1;
            }
            else
                (*log)->pending = true;
            (*log)->skipped = false;
        }
    }
    return arucotag_pause_detect;
}
