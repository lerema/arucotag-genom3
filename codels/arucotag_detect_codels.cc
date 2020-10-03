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

#include <string.h>
#include <cmath>

/* --- Task detect ------------------------------------------------------ */


/** Codel detect_start of task detect.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_wait.
 */
genom_event
detect_start(arucotag_ids *ids, const genom_context self)
{
    // Init IDS fields
    ids->length = 0;
    ids->reset = false;
    ids->calib = new arucotag_calib();
    ids->tags = new arucotag_detector();
    ids->pred = new arucotag_predictor();
    ids->log = new arucotag_log_s();

    return arucotag_wait;
}


/** Codel detect_wait of task detect.
 *
 * Triggered by arucotag_wait.
 * Yields to arucotag_pause_wait, arucotag_main.
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
        or_sensor_calibration* c = &(intrinsics->data(self)->calib);
        (*calib)->K = (Mat_<float>(3,3) <<
            c->fx, c->gamma, c->cx,
                0,    c->fy, c->cy,
                0,        0,     1
        );
        (*calib)->D = (Mat_<float>(4,1) <<
            intrinsics->data(self)->disto.k1,
            intrinsics->data(self)->disto.k2,
            intrinsics->data(self)->disto.k3,
            intrinsics->data(self)->disto.p1,
            intrinsics->data(self)->disto.p2
        );
        return arucotag_main;
    }
    else
    {
        return arucotag_pause_wait;
    }
}


/** Codel detect_main of task detect.
 *
 * Triggered by arucotag_main.
 * Yields to arucotag_pause_main, arucotag_valid.
 */
genom_event
detect_main(const arucotag_frame *frame, float length,
            const arucotag_calib *calib, arucotag_detector **tags,
            const sequence_arucotag_portinfo *ports,
            const genom_context self)
{
    // Sleep if no marker is tracked
    if (!ports->_length) return arucotag_pause_main;

    frame->read(self);
    or_sensor_frame* fdata = frame->data(self);

    // Convert frame to cv::Mat
    Mat cvframe = Mat(
        Size(fdata->width, fdata->height),
        (fdata->bpp == 1) ? CV_8UC1 : CV_8UC3,
        (void*)fdata->pixels._buffer,
        Mat::AUTO_STEP
    );

    // Detect tags in frame
    vector<int> ids;
    vector<vector<Point2f>> corners;
    aruco::detectMarkers(cvframe, (*tags)->dict, corners, ids);

    // Estimate pose from corners
    if (ids.size() > 0)
    {
        vector<Vec3d> translations, rotations;
        aruco::estimatePoseSingleMarkers(corners, length, calib->K, calib->D, rotations, translations);

        (*tags)->valid_ids.clear();
        (*tags)->meas.clear();
        // Check that detected tags are among tracked markers
        for (uint16_t i=0; i<ids.size(); i++)
        {
            const char* id = to_string(ids[i]).c_str();
            uint16_t j = 0;
            for (j=0; j<ports->_length; j++)
                if (!strcmp(ports->_buffer[j], id))
                    break;
            if (j >= ports->_length) continue;

            (*tags)->valid_ids.push_back(ids[i]);
            (*tags)->meas.push_back((Mat_<float>(6,1) <<
                translations[i][0],
                translations[i][1],
                translations[i][2],
                rotations[i][0],
                rotations[i][1],
                rotations[i][2]
            ));
        }

        if ((*tags)->valid_ids.size())
            return arucotag_valid;
    }
    return arucotag_pause_main;
}


/** Codel detect_valid of task detect.
 *
 * Triggered by arucotag_valid.
 * Yields to arucotag_log.
 */
genom_event
detect_valid(const arucotag_detector *tags, arucotag_predictor **pred,
             const genom_context self)
{
    (*pred)->meas = tags->meas;
    (*pred)->new_detections = tags->valid_ids;

    return arucotag_log;
}


/** Codel detect_log of task detect.
 *
 * Triggered by arucotag_log.
 * Yields to arucotag_pause_main.
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
            timeval tv;
            gettimeofday(&tv, NULL);
            char buffer[512];
            (*log)->req.aio_nbytes = 0;
            for (uint16_t i=0; i<tags->valid_ids.size(); i++)
            {
                (*log)->req.aio_nbytes += snprintf(
                    buffer, sizeof(buffer),
                    "%s" arucotag_log_fmt "\n",
                    (*log)->skipped ? "\n" : "",
                    tv.tv_sec, tv.tv_usec*1000,
                    tags->valid_ids[i],
                    0,
                    tags->meas[i].at<float>(0),
                    tags->meas[i].at<float>(1),
                    tags->meas[i].at<float>(2)
                );
                if (i==0) strcpy((*log)->buffer, buffer);
                else      strcat((*log)->buffer, buffer);
            }
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
    return arucotag_pause_main;
}


/* --- Activity add_marker ---------------------------------------------- */

/** Codel add_marker of activity add_marker.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_ether.
 */
genom_event
add_marker(const char marker[16], arucotag_predictor **pred,
           sequence_arucotag_portinfo *ports,
           const arucotag_pose *pose, const genom_context self)
{
    // Add new marker in port list
    uint16_t i;
    for(i=0; i<ports->_length; i++)
        if (!strcmp(ports->_buffer[i], marker)) return arucotag_ether;

    if (i >= ports->_maximum)
        if (genom_sequence_reserve(ports, i + 1))
            return arucotag_e_sys_error("add", self);
    (ports->_length)++;
    strncpy(ports->_buffer[i], marker, 16);
    (*pred)->add(std::stoi(marker));

    // Init new out port
    pose->open(marker, self);

    pose->data(marker, self)->pos._present = true;
    pose->data(marker, self)->pos_cov._present = true;
    pose->data(marker, self)->att._present = false;
    pose->data(marker, self)->vel._present = false;
    pose->data(marker, self)->avel._present = false;
    pose->data(marker, self)->acc._present = false;
    pose->data(marker, self)->aacc._present = false;

    warnx("tracking new marker: %s", marker);
    return arucotag_ether;
}
