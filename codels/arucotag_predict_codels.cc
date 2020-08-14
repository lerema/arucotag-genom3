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
#include <sys/time.h>

/* --- Task predict ----------------------------------------------------- */


/** Codel predict_start of task predict.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_wait.
 */
genom_event
predict_start(arucotag_ids *ids, const arucotag_pose *pose,
              const genom_context self)
{
    pose->data(self)->pos._present = true;
    pose->data(self)->att._present = false;
    pose->data(self)->vel._present = true;
    pose->data(self)->avel._present = false;
    pose->data(self)->acc._present = false;
    pose->data(self)->aacc._present = false;

    ids->kalman = new arucotag_predictor();
    ids->kalman->kf.init(6, 6, 0, CV_32F);

    setIdentity(ids->kalman->kf.processNoiseCov, Scalar::all(1e-2));     // Trust in predict step: the less, the more trust
    setIdentity(ids->kalman->kf.measurementNoiseCov, Scalar::all(5e-3)); // Trust in correct step: the less, the more trust
    setIdentity(ids->kalman->kf.errorCovPost, cv::Scalar::all(1e-1));
    setIdentity(ids->kalman->kf.errorCovPre, cv::Scalar::all(1e-1));

    setIdentity(ids->kalman->kf.measurementMatrix); // 3,6
    setIdentity(ids->kalman->kf.transitionMatrix); // 6,6
    setIdentity(ids->kalman->kf.controlMatrix); // 6,6
    // Transition matrix from x_k to x_(k+1)
    // (constant velocity model)
    // [ 1 0 0 dt  0  0 ]  [ x  ]
    // [ 0 1 0  0 dt  0 ]  [ y  ]
    // [ 0 0 1  0  0 dt ]  [ z  ]
    // [ 0 0 0  1  0  0 ]  [ vx ]
    // [ 0 0 0  0  1  0 ]  [ vy ]
    // [ 0 0 0  0  0  1 ]  [ vz ]
    ids->kalman->kf.transitionMatrix.at<float>(0,3) = arucotag_predict_period/1000.;
    ids->kalman->kf.transitionMatrix.at<float>(1,4) = arucotag_predict_period/1000.;
    ids->kalman->kf.transitionMatrix.at<float>(2,5) = arucotag_predict_period/1000.;

    // Fixed transformation matrix from drone to camera
    // (t is translation from drone to camera)
    // (r is rotation from drone to camera)
    // [ r11 r12 r13   0   tz -ty ] vx
    // [ r21 r22 r23  -tz   0  tx ] vy
    // [ r31 r32 r33   ty -tx   0 ] vz
    // [   0   0   0  r11 r12 r13 ] wx
    // [   0   0   0  r21 r22 r23 ] wy
    // [   0   0   0  r31 r32 r33 ] wz
    ids->kalman->C_T_B = (Mat_<float>(6,6) <<
        1, 0, 0, 0, 0, 0,
        0,-1, 0, 0, 0, 0,
        0, 0,-1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0,-1, 0,
        0, 0, 0, 0, 0,-1
    );

    return arucotag_wait;
}

/** Codel predict_wait of task predict.
 *
 * Triggered by arucotag_wait.
 * Yields to arucotag_pause_wait, arucotag_log.
 */
genom_event
predict_wait(arucotag_detector **tags, arucotag_predictor **kalman,
             const genom_context self)
{
    if ((*tags)->new_detection) {
        (*tags)->measured_state.copyTo((*kalman)->state);
        (*kalman)->kf.statePost = (*kalman)->state;
        (*tags)->new_detection = false;
        return arucotag_log;
    }
    else
        return arucotag_pause_wait;
}


/** Codel predict_main of task predict.
 *
 * Triggered by arucotag_predict.
 * Yields to arucotag_log.
 */
genom_event
predict_main(arucotag_detector **tags, arucotag_predictor **kalman,
             const arucotag_drone *drone, const arucotag_pose *pose,
             const genom_context self)
{
    // PREDICT
    // 1- Read state from POM and move it to camera frame
    drone->read(self);
    Mat control = (Mat_<float>(6,1) <<
        drone->data(self)->vel._value.vx,
        drone->data(self)->vel._value.vy,
        drone->data(self)->vel._value.vz,
        drone->data(self)->avel._value.wx,
        drone->data(self)->avel._value.wy,
        drone->data(self)->avel._value.wz
    );
    // 2- Transformation from world to body
    // (t is translation from world to drone)
    // (r is rotation from world to drone)
    // [ 1   0   0    0   0   0 ] vx
    // [ 0   1   0    0   0   0 ] vy
    // [ 0   0   1    0   0   0 ] vz
    // [ 0   0   0  r11 r12 r13 ] wx
    // [ 0   0   0  r21 r22 r23 ] wy
    // [ 0   0   0  r31 r32 r33 ] wz
    double qw = drone->data(self)->att._value.qw;
    double qx = drone->data(self)->att._value.qx;
    double qy = drone->data(self)->att._value.qy;
    double qz = drone->data(self)->att._value.qz;
    Mat r = (Mat_<float>(3,3) <<
        1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
            2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw,
            2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy
    );
    r = r.t();
    Mat B_T_W = (Mat_<float>(6,6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, r.at<float>(0,0), r.at<float>(0,1), r.at<float>(0,2),
        0, 0, 0, r.at<float>(1,0), r.at<float>(1,1), r.at<float>(1,2),
        0, 0, 0, r.at<float>(2,0), r.at<float>(2,1), r.at<float>(2,2)
    );
    control = B_T_W * control;
    // 3- Transformation from body to camera
    control = (*kalman)->C_T_B * control;
    // 4- Compute control matrix as function of (dt,X)
    // [ -dt   0   0     0 -dt*z  dt*y ]  [ vx ]
    // [   0 -dt   0  dt*z     0 -dt*x ]  [ vy ]
    // [   0   0 -dt -dt*y  dt*x     0 ]  [ vz ]
    // [  -1   0   0     0    -z     y ]  [ wx ]
    // [   0  -1   0     z     0    -x ]  [ wy ]
    // [   0   0  -1    -y     x     0 ]  [ wz ]
    float dt = arucotag_predict_period/1000.;
    float x = (*kalman)->state.at<float>(0);
    float y = (*kalman)->state.at<float>(1);
    float z = (*kalman)->state.at<float>(2);
    // (*kalman)->kf.controlMatrix = (Mat_<float>(6,6) <<
    //     -dt,  0,  0,    0,-dt*z, dt*y,
    //       0,-dt,  0, dt*z,    0,-dt*x,
    //       0,  0,-dt,-dt*y, dt*x,    0,
    //      -1,  0,  0,    0,   -z,    y,
    //       0, -1,  0,    z,    0,   -x,
    //       0,  0, -1,   -y,    x,    0
    // );
    // (*kalman)->state = (*kalman)->kf.predict(control);
    (*kalman)->state = (*kalman)->kf.predict();
    // CORRECT
    if ((*tags)->new_detection) {
        (*kalman)->state = (*kalman)->kf.correct((*tags)->measured_state);
        (*tags)->new_detection = false;
    }
    // UPDATE
    // (*kalman)->state is updated to statePre if no measurement, statePost if there is
    // idem for covariances but they are not published ATM.

    // PUBLISH
    pose->data(self)->pos._value.x = (*kalman)->state.at<float>(0);
    pose->data(self)->pos._value.y = (*kalman)->state.at<float>(1);
    pose->data(self)->pos._value.z = (*kalman)->state.at<float>(2);
    pose->data(self)->vel._value.vx = (*kalman)->state.at<float>(3);
    pose->data(self)->vel._value.vy = (*kalman)->state.at<float>(4);
    pose->data(self)->vel._value.vz = (*kalman)->state.at<float>(5);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    pose->data(self)->ts.sec = tv.tv_sec;
    pose->data(self)->ts.nsec = (tv.tv_usec / 1000.0) * 1000;
    pose->write(self);

    return arucotag_log;
}


/** Codel predict_log of task predict.
 *
 * Triggered by arucotag_log.
 * Yields to arucotag_pause_predict.
 */
genom_event
predict_log(const arucotag_predictor *kalman, arucotag_log_s **log,
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
                  tv.tv_sec, tv.tv_usec,
                  kalman->state.at<float>(0),
                  kalman->state.at<float>(1),
                  kalman->state.at<float>(2),
                  kalman->state.at<float>(3),
                  kalman->state.at<float>(4),
                  kalman->state.at<float>(5)
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
    return arucotag_pause_predict;
}
