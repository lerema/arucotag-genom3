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
#include <algorithm>

/* --- Task predict ----------------------------------------------------- */


/** Codel predict_start of task predict.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_wait.
 */
genom_event
predict_start(const genom_context self)
{
    return arucotag_wait;
}


/** Codel predict_wait of task predict.
 *
 * Triggered by arucotag_wait.
 * Yields to arucotag_pause_wait, arucotag_main.
 */
genom_event
predict_wait(const arucotag_extrinsics *extrinsics,
             arucotag_calib **calib, arucotag_predictor **pred,
             const genom_context self)
{
    if (extrinsics->read(self) == genom_ok && extrinsics->data(self)
        && (*pred)->new_detections.size() != 0)
    {
        // Init static extr and intr and yield to detect codel
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

        Mat rot = (*calib)->B_R_C.t();
        double tx = -extrinsics->data(self)->trans.tx;
        double ty = -extrinsics->data(self)->trans.ty;
        double tz = -extrinsics->data(self)->trans.tz;
        Mat t = (Mat_<float>(3,3) <<
            0,  tz, -ty,
          -tz,   0,  tx,
           ty, -tx,   0
        );

        // Fixed transformation matrix from drone to camera
        // (t is translation from drone to camera)
        // (r is rotation from drone to camera)
        // [ r11 r12 r13   0   tz -ty ]  [ vx ]
        // [ r21 r22 r23  -tz   0  tx ]  [ vy ]
        // [ r31 r32 r33   ty -tx   0 ]  [ vz ]
        // [   0   0   0  r11 r12 r13 ]  [ wx ]
        // [   0   0   0  r21 r22 r23 ]  [ wy ]
        // [   0   0   0  r31 r32 r33 ]  [ wz ]
        rot.copyTo((*pred)->C_T_B(Range(0,3),Range(0,3)));
        rot.copyTo((*pred)->C_T_B(Range(3,6),Range(3,6)));
        t.copyTo((*pred)->C_T_B(Range(0,3),Range(3,6)));

        return arucotag_main;
    }
    else
        return arucotag_pause_wait;
}


/** Codel predict_main of task predict.
 *
 * Triggered by arucotag_main.
 * Yields to arucotag_pause_main, arucotag_log.
 */
genom_event
predict_main(const arucotag_calib *calib, arucotag_predictor **pred,
             const arucotag_drone *drone, const arucotag_pose *pose,
             const genom_context self)
{
    // 1- Get control
    Mat control, W_t_B, W_R_B;
    bool nostate = !(drone->read(self) == genom_ok && drone->data(self));
    if (nostate)
    {
        control = Mat::zeros(6, 1, CV_32F);
        W_R_B = Mat::zeros(3, 3, CV_32F);
        W_t_B = Mat::zeros(3, 1, CV_32F);
    }
    else
    {
        control = (Mat_<float>(6,1) <<
            drone->data(self)->vel._value.vx,
            drone->data(self)->vel._value.vy,
            drone->data(self)->vel._value.vz,
            drone->data(self)->avel._value.wx,
            drone->data(self)->avel._value.wy,
            drone->data(self)->avel._value.wz
        );

        // 1.1- Transformation from world to drone to camera
        // (t is translation from world to drone)
        // (r is rotation from world to drone)
        // [ r11 r12 r13  0   0   0 ]  [ vx ]
        // [ r21 r22 r23  0   0   0 ]  [ vy ]
        // [ r31 r32 r33  0   0   0 ]  [ vz ]
        // [ 0   0   0  r11 r12 r13 ]  [ wx ]
        // [ 0   0   0  r21 r22 r23 ]  [ wy ]
        // [ 0   0   0  r31 r32 r33 ]  [ wz ]
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
        Mat r = W_R_B.t();
        Mat B_T_W = Mat::eye(6, 6, CV_32F);
        r.copyTo(B_T_W(Range(0,3),Range(0,3)));
        r.copyTo(B_T_W(Range(3,6),Range(3,6)));

        control = (*pred)->C_T_B * B_T_W * control;
    }

    for (uint16_t i=0; i<(*pred)->filters.size(); i++)
    {
        // Check if there is a new measurement for this tag
        vector<int>* v = &((*pred)->new_detections);
        vector<int>::iterator j = find(v->begin(), v->end(), (*pred)->filters[i].id);

        if ((*pred)->filters[i].state.empty())
        {
            // Init the kf state if its a new one
            if (j != v->end())
                (*pred)->meas[j - v->begin()].copyTo((*pred)->filters[i].state);
        }
        else
        {
            if (!nostate)
            {
                // 1.2- Compute control matrix as function of (dt,X)
                // [ -dt   0   0     0  dt*z -dt*y ]  [ vx ]
                // [   0 -dt   0 -dt*z     0  dt*x ]  [ vy ]
                // [   0   0 -dt  dt*y -dt*x     0 ]  [ vz ]
                //                                    [ wx ]
                //                                    [ wy ]
                //                                    [ wz ]
                float dt = arucotag_predict_period/1000.;
                float x = (*pred)->filters[i].state.at<float>(0);
                float y = (*pred)->filters[i].state.at<float>(1);
                float z = (*pred)->filters[i].state.at<float>(2);
                (*pred)->filters[i].kf.controlMatrix = (Mat_<float>(3,6) <<
                    -dt,  0,  0,    0,-dt*z, dt*y,
                      0,-dt,  0, dt*z,    0,-dt*x,
                      0,  0,-dt,-dt*y, dt*x,    0
                );
            }

            // 2- Predict with controls
            (*pred)->filters[i].state = (*pred)->filters[i].kf.predict(control);

            // 3- Correct
            if (j != v->end())
                (*pred)->filters[i].state = (*pred)->filters[i].kf.correct((*pred)->meas[j - v->begin()]);

            // 4- Update
            // (*pred)->filters[i].state is updated to statePre if no measurement, statePost if there is
            // idem for covariances but they are not published ATM

            // 5- Publish
            Mat W_r = W_R_B * (calib->B_R_C * (*pred)->filters[i].state + calib->B_t_C) + W_t_B;
            pose->data(to_string((*pred)->filters[i].id).c_str(), self)->pos._value =
            {
                W_r.at<float>(0),
                W_r.at<float>(1),
                W_r.at<float>(2)
            };
            timeval tv;
            gettimeofday(&tv, NULL);
            pose->data(to_string((*pred)->filters[i].id).c_str(), self)->ts.sec = tv.tv_sec;
            pose->data(to_string((*pred)->filters[i].id).c_str(), self)->ts.nsec = tv.tv_usec*1000;
            pose->write(to_string((*pred)->filters[i].id).c_str(), self);
        }
    }

    for (uint16_t i=0; i<(*pred)->filters.size(); i++)
        if (!((*pred)->filters[i].state.empty()))
            return arucotag_log;
    return arucotag_pause_main;
}


/** Codel predict_log of task predict.
 *
 * Triggered by arucotag_log.
 * Yields to arucotag_pause_main.
 */
genom_event
predict_log(const arucotag_predictor *pred, arucotag_log_s **log,
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
            for (uint16_t i=0; i<pred->filters.size(); i++)
            {
                if (pred->filters[i].state.empty()) continue;
                (*log)->req.aio_nbytes += snprintf(
                    buffer, sizeof(buffer),
                    "%s" arucotag_log_fmt "\n",
                    (*log)->skipped ? "\n" : "",
                    tv.tv_sec, tv.tv_usec*1000,
                    pred->filters[i].id,
                    1,
                    pred->filters[i].state.at<float>(0),
                    pred->filters[i].state.at<float>(1),
                    pred->filters[i].state.at<float>(2)
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
