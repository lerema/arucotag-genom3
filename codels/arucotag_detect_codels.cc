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
    ids->out_frame = 0;
    ids->calib = new arucotag_calib();
    ids->tags = new arucotag_detector();
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
            const arucotag_extrinsics *extrinsics,
            const arucotag_frame *frame, arucotag_calib **calib,
            const genom_context self)
{
    if (intrinsics->read(self) == genom_ok && intrinsics->data(self) &&
        extrinsics->read(self) == genom_ok && extrinsics->data(self) &&
        frame->read(self) == genom_ok && frame->data(self) &&
        frame->data(self)->pixels._length > 0 &&
        length > 0)
    {
        // Init intr
        or_sensor_calibration* c = &(intrinsics->data(self)->calib);
        (*calib)->K = (Mat_<float>(3,3) <<
            c->fx, c->gamma, c->cx,
                0,    c->fy, c->cy,
                0,        0,     1
        );
        (*calib)->D = (Mat_<float>(5,1) <<
            intrinsics->data(self)->disto.k1,
            intrinsics->data(self)->disto.k2,
            intrinsics->data(self)->disto.k3,
            intrinsics->data(self)->disto.p1,
            intrinsics->data(self)->disto.p2
        );
        // Init extr
        (*calib)->B_t_C = (Mat_<float>(3,1) <<
            extrinsics->data(self)->trans.tx,
            extrinsics->data(self)->trans.ty,
            extrinsics->data(self)->trans.tz
        );
        float r = extrinsics->data(self)->rot.roll;
        float p = extrinsics->data(self)->rot.pitch;
        float y = extrinsics->data(self)->rot.yaw;
        (*calib)->B_R_C = (Mat_<float>(3,3) <<
            cos(p)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y), cos(r)*sin(p)*cos(y) + sin(r)*sin(y),
            cos(p)*sin(y), sin(r)*sin(p)*sin(y) + cos(r)*cos(y), cos(r)*sin(p)*sin(y) - sin(r)*cos(y),
                  -sin(p),                        sin(r)*cos(p),                        cos(r)*cos(p)
        );

        return arucotag_main;
    }
    return arucotag_pause_wait;
}


/** Codel detect_main of task detect.
 *
 * Triggered by arucotag_main.
 * Yields to arucotag_pause_main, arucotag_log.
 */
genom_event
detect_main(const arucotag_frame *frame, float length,
            const arucotag_calib *calib, const arucotag_drone *drone,
            arucotag_detector **tags,
            const sequence_arucotag_portinfo *ports,
            const arucotag_pose *pose,
            const arucotag_pixel_pose *pixel_pose, int16_t out_frame,
            const genom_context self)
{
timeval start_codel, stop_codel;
gettimeofday(&start_codel, NULL);
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

    if (ids.size() == 0)
    {
        for (uint16_t i=0; i<ports->_length; i++)
        {
            timeval tv;
            gettimeofday(&tv, NULL);
            pose->data(ports->_buffer[i], self)->ts.sec = tv.tv_sec;
            pose->data(ports->_buffer[i], self)->ts.nsec = tv.tv_usec*1000;
            pose->data(ports->_buffer[i], self)->pos._present = false;
            pose->data(ports->_buffer[i], self)->pos_cov._present = false;
            pose->data(ports->_buffer[i], self)->att._present = false;
            pose->data(ports->_buffer[i], self)->att_cov._present = false;
            pose->write(ports->_buffer[i], self);
        }
        return arucotag_pause_main;
    }
    else
        for (uint16_t i=0; i<ports->_length; i++)
            if (find(ids.begin(), ids.end(), std::stoi(ports->_buffer[i])) == ids.end())
            {
                timeval tv;
                gettimeofday(&tv, NULL);
                pose->data(ports->_buffer[i], self)->ts.sec = tv.tv_sec;
                pose->data(ports->_buffer[i], self)->ts.nsec = tv.tv_usec*1000;
                pose->data(ports->_buffer[i], self)->pos._present = false;
                pose->data(ports->_buffer[i], self)->pos_cov._present = false;
                pose->data(ports->_buffer[i], self)->att._present = false;
                pose->data(ports->_buffer[i], self)->att_cov._present = false;
                pose->write(ports->_buffer[i], self);
            }

    // Estimate pose from corners
    vector<Vec3d> translations, rotations;
    aruco::estimatePoseSingleMarkers(corners, length, calib->K, calib->D, rotations, translations);

    // Get state feedback
    Mat W_t_B, W_R_B;
    if (!(drone->read(self) == genom_ok && drone->data(self)))
    {
        W_R_B = Mat::eye(3, 3, CV_32F);
        W_t_B = Mat::zeros(3, 1, CV_32F);
    }
    else
    {
        W_t_B = (Mat_<float>(3,1) <<
            drone->data(self)->pos._value.x,
            drone->data(self)->pos._value.y,
            drone->data(self)->pos._value.z
        );
        double qw = drone->data(self)->att._value.qw;
        double qx = drone->data(self)->att._value.qx;
        double qy = drone->data(self)->att._value.qy;
        double qz = drone->data(self)->att._value.qz;
        W_R_B = (Mat_<float>(3,3) <<
            1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
                2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw,
                2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy
        );
    }

    (*tags)->valid_ids.clear();
    (*tags)->meas.clear();

    for (uint16_t i=0; i<ids.size(); i++)
    {
        // Check that detected tags are among tracked markers
        const char* id = to_string(ids[i]).c_str();
        uint16_t j = 0;
        for (j=0; j<ports->_length; j++)
            if (!strcmp(ports->_buffer[j], id))
                break;
        if (j >= ports->_length) continue;

        // Transform to world frame
        Mat C_t_M = (Mat_<float>(3,1) << translations[i][0], translations[i][1], translations[i][2]);
        Mat C_R_M = Mat::zeros(3,3, CV_32F);
        Rodrigues(rotations[i], C_R_M);
        C_R_M.convertTo(C_R_M, CV_32F);
        Mat position, orientation;
        switch (out_frame)
        {
            case 0:  // Camera frame
                position = C_t_M;
                orientation = C_R_M;
                break;
            case 1:  // Body frame
                position = (calib->B_R_C * C_t_M + calib->B_t_C);
                orientation = calib->B_R_C * C_R_M;
                break;
            case 2:  // World frame
                position = W_R_B * (calib->B_R_C * C_t_M + calib->B_t_C) + W_t_B;
                orientation = W_R_B * calib->B_R_C * C_R_M;
                break;
        }

        // Compute covariance
        // arbitrary isotropic pixel error
        float sigma_p = 3;

        Mat J_pos = Mat::zeros(8,3, CV_32F);
        Mat c = (Mat_<float>(3,4) <<
            -1,  1,  1, -1,
            -1, -1,  1,  1,
             0,  0,  0,  0
        );
        c = c * length/2;
        for (uint16_t i=0; i<4; i++)
        {
            Mat ci = c.col(i);
            Mat hi = calib->K*(C_R_M*ci + C_t_M);
            // jacobian of pixellization (homogeneous->pixel) operation wrt homogeneous coordinates
            Mat J_pix = (Mat_<float>(2,3) <<
                1/hi.at<float>(2), 0, -hi.at<float>(0)/hi.at<float>(2)/hi.at<float>(2),
                0, 1/hi.at<float>(2), -hi.at<float>(1)/hi.at<float>(2)/hi.at<float>(2)
            );
            // jacobian of projection
            Mat J_proj = Mat::zeros(3,6, CV_32F);
            // jacobian of projection wrt translation
            calib->K.copyTo(J_proj(Rect(0,0,3,3)));
            // jacobian of projection wrt rotation
            Mat skew = (Mat_<float>(3,3) <<
                              0, -ci.at<float>(2),  ci.at<float>(1),
                ci.at<float>(2),                0, -ci.at<float>(0),
               -ci.at<float>(1),  ci.at<float>(0),                0
            );
            J_proj(Rect(3,0,3,3)) = -calib->K * C_R_M * skew;
            // jacobian of projection wrt euclidean coordinates (chain rule)
            Mat J_full = J_pix*J_proj;
            J_full(Rect(0,0,3,2)).copyTo(J_pos(Rect(0,i*2,3,2)));
        }

        Mat cov_pos = sigma_p*sigma_p * (J_pos.t() * J_pos).inv();

        // Convert rotation
        // Equations taken from [Robotics (Siciliano), p. 55]
        float& r11 = orientation.at<float>(0,0);
        float& r12 = orientation.at<float>(0,1);
        float& r13 = orientation.at<float>(0,2);
        float& r21 = orientation.at<float>(1,0);
        float& r22 = orientation.at<float>(1,1);
        float& r23 = orientation.at<float>(1,2);
        float& r31 = orientation.at<float>(2,0);
        float& r32 = orientation.at<float>(2,1);
        float& r33 = orientation.at<float>(2,2);

        float qw = 0.5 * sqrt(max(1 + r11 + r22 + r33, 0.f));
        float qx = (r32 - r23 < 0) ? -0.5 * sqrt(max(r11 - r22 - r33 + 1, 0.f)) : 0.5 * sqrt(max(r11 - r22 - r33 + 1, 0.f));
        float qy = (r13 - r31 < 0) ? -0.5 * sqrt(max(r22 - r33 - r11 + 1, 0.f)) : 0.5 * sqrt(max(r22 - r33 - r11 + 1, 0.f));
        float qz = (r21 - r12 < 0) ? -0.5 * sqrt(max(r33 - r11 - r22 + 1, 0.f)) : 0.5 * sqrt(max(r33 - r11 - r22 + 1, 0.f));

        Mat q = (Mat_<float>(4,1) << qw, qx, qy, qz);

        Mat rpy = (Mat_<float>(3,1) <<
            atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy)),
            asin(2 * (qw*qy - qz*qx)),
            atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
        );

        // Publish
        pose->data(to_string(ids[i]).c_str(), self)->pos._present = true;
        pose->data(to_string(ids[i]).c_str(), self)->pos_cov._present = true;
        pose->data(to_string(ids[i]).c_str(), self)->att._present = true;
        pose->data(to_string(ids[i]).c_str(), self)->att_cov._present = false;

        pose->data(to_string(ids[i]).c_str(), self)->pos._value =
        {
            position.at<float>(0),
            position.at<float>(1),
            position.at<float>(2)
        };
        pose->data(to_string(ids[i]).c_str(), self)->pos_cov._value =
        {
            cov_pos.at<float>(0,0),
            cov_pos.at<float>(1,0),
            cov_pos.at<float>(1,1),
            cov_pos.at<float>(2,0),
            cov_pos.at<float>(2,1),
            cov_pos.at<float>(2,2)
        };
        pose->data(to_string(ids[i]).c_str(), self)->att._value =
        {
            q.at<float>(0),
            q.at<float>(1),
            q.at<float>(2),
            q.at<float>(3)
        };

        timeval tv;
        gettimeofday(&tv, NULL);
        pose->data(to_string(ids[i]).c_str(), self)->ts.sec = tv.tv_sec;
        pose->data(to_string(ids[i]).c_str(), self)->ts.nsec = tv.tv_usec*1000;
        pose->write(to_string(ids[i]).c_str(), self);

        // Compute centroid of tag
        Point2f center(0, 0);
        for(int p = 0; p < 4; p++)
            center += corners[i][p];
        center = center / 4.;

        pixel_pose->data(to_string(ids[i]).c_str(), self)->ts = pose->data(to_string(ids[i]).c_str(), self)->ts;
        pixel_pose->data(to_string(ids[i]).c_str(), self)->x = round(center.x);
        pixel_pose->data(to_string(ids[i]).c_str(), self)->y = round(center.y);
        pixel_pose->write(to_string(ids[i]).c_str(), self);

        // Save for logs
        (*tags)->valid_ids.push_back(ids[i]);
        (*tags)->meas.push_back((Mat_<float>(8,1) <<
            round(center.x),
            round(center.y),
            position.at<float>(0),
            position.at<float>(1),
            position.at<float>(2),
            rpy.at<float>(0),
            rpy.at<float>(1),
            rpy.at<float>(2)
        ));
    }

    if ((*tags)->valid_ids.size())
        return arucotag_log;
    return arucotag_pause_main;
}


/** Codel detect_log of task detect.
 *
 * Triggered by arucotag_log.
 * Yields to arucotag_pause_main.
 */
genom_event
detect_log(int16_t out_frame, const arucotag_detector *tags,
           arucotag_log_s **log, const genom_context self)
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
                    out_frame,
                    tags->valid_ids[i],
                    tags->meas[i].at<float>(0),
                    tags->meas[i].at<float>(1),
                    tags->meas[i].at<float>(2),
                    tags->meas[i].at<float>(3),
                    tags->meas[i].at<float>(4),
                    tags->meas[i].at<float>(5),
                    tags->meas[i].at<float>(6),
                    tags->meas[i].at<float>(7)
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
add_marker(const char marker[16], sequence_arucotag_portinfo *ports,
           const arucotag_pose *pose,
           const arucotag_pixel_pose *pixel_pose,
           const genom_context self)
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

    // Init new out ports
    pixel_pose->open(marker, self);
    pose->open(marker, self);

    warnx("tracking new marker: %s", marker);
    return arucotag_ether;
}
