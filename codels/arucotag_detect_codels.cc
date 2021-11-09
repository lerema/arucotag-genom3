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
#include <opencv2/core/eigen.hpp>

#include <string.h>
#include <cmath>

/* --- Helper func ------------------------------------------------------ */
void update_calib(const arucotag_intrinsics *intrinsics,
                  const arucotag_extrinsics *extrinsics,
                  arucotag_calib **calib, const genom_context self)
{
    // Init intr
    or_sensor_calibration* c = &(intrinsics->data(self)->calib);
    (*calib)->K_cv = (Mat_<float>(3,3) <<
        c->fx, c->gamma, c->cx,
            0,    c->fy, c->cy,
            0,        0,     1
    );
    cv2eigen((*calib)->K_cv, (*calib)->K);
    (*calib)->D = (Mat_<float>(5,1) <<
        intrinsics->data(self)->disto.k1,
        intrinsics->data(self)->disto.k2,
        intrinsics->data(self)->disto.k3,
        intrinsics->data(self)->disto.p1,
        intrinsics->data(self)->disto.p2
    );
    // Init extr
    (*calib)->B_p_C <<
        extrinsics->data(self)->trans.tx,
        extrinsics->data(self)->trans.ty,
        extrinsics->data(self)->trans.tz;
    float r = extrinsics->data(self)->rot.roll;
    float p = extrinsics->data(self)->rot.pitch;
    float y = extrinsics->data(self)->rot.yaw;
    (*calib)->B_R_C <<
        cos(p)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y), cos(r)*sin(p)*cos(y) + sin(r)*sin(y),
        cos(p)*sin(y), sin(r)*sin(p)*sin(y) + cos(r)*cos(y), cos(r)*sin(p)*sin(y) - sin(r)*cos(y),
              -sin(p),                        sin(r)*cos(p),                        cos(r)*cos(p);
}


static void arucotag_log_f(arucotag_log_s** log, int16_t out_frame, const char* tagid, int16_t u, int16_t v, or_pose_estimator_state* posedata)
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
            // Convert quaternion to roll/pitch/yaw
            double qw = posedata->att._value.qw;
            double qx = posedata->att._value.qx;
            double qy = posedata->att._value.qy;
            double qz = posedata->att._value.qz;

            double roll = atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy));
            double pitch = asin(2 * (qw*qy - qz*qx));
            double yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

            (*log)->req.aio_nbytes = snprintf(
                (*log)->buffer, sizeof((*log)->buffer),
                "%s" arucotag_log_fmt "\n",
                (*log)->skipped ? "\n" : "",
                posedata->ts.sec, posedata->ts.nsec,
                out_frame,                  // frame
                tagid,
                u, v,                       // pixel
                posedata->pos._value.x,     // p
                posedata->pos._value.y,
                posedata->pos._value.z,
                roll,                       // att (euler)
                pitch,
                yaw,
                posedata->pos_cov._value.cov[0], // Sigma_p
                posedata->pos_cov._value.cov[1],
                posedata->pos_cov._value.cov[2],
                posedata->pos_cov._value.cov[3],
                posedata->pos_cov._value.cov[4],
                posedata->pos_cov._value.cov[5],
                posedata->att_cov._value.cov[0], // Sigma_q
                posedata->att_cov._value.cov[1],
                posedata->att_cov._value.cov[2],
                posedata->att_cov._value.cov[3],
                posedata->att_cov._value.cov[4],
                posedata->att_cov._value.cov[5],
                posedata->att_cov._value.cov[6],
                posedata->att_cov._value.cov[7],
                posedata->att_cov._value.cov[8],
                posedata->att_cov._value.cov[9]
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
}


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
 * Yields to arucotag_wait, arucotag_poll.
 */
genom_event
detect_wait(const arucotag_frame *frame,
            const arucotag_intrinsics *intrinsics,
            const arucotag_extrinsics *extrinsics, float length,
            arucotag_calib **calib, const genom_context self)
{
    if (length > 0 &&
        intrinsics->read(self) == genom_ok && intrinsics->data(self) &&
        extrinsics->read(self) == genom_ok && extrinsics->data(self) &&
        frame->read(self) == genom_ok && frame->data(self) &&
        frame->data(self)->pixels._length > 0)
    {
        update_calib(intrinsics, extrinsics, calib, self);
        return arucotag_poll;
    }
    else
    {
        usleep(arucotag_pause_ms*1e3);
        return arucotag_wait;
    }
}


/** Codel detect_poll of task detect.
 *
 * Triggered by arucotag_poll.
 * Yields to arucotag_pause_poll, arucotag_poll, arucotag_main.
 */
genom_event
detect_poll(const sequence_arucotag_portinfo *ports,
            const arucotag_frame *frame, or_time_ts *last_ts,
            const genom_context self)
{
    if (!ports->_length)
        return arucotag_pause_poll;

    timeval start, stop;
    gettimeofday(&start, NULL);

    frame->read(self);
    if (frame->data(self)->ts.sec != last_ts->sec || frame->data(self)->ts.nsec != last_ts->nsec)
    {
        *last_ts = frame->data(self)->ts;
        return arucotag_main;

    }
    else
    {
        gettimeofday(&stop, NULL);
        double dt_ms = (stop.tv_sec + stop.tv_usec*1e-6 - start.tv_sec - start.tv_usec*1e-6)*1e3;
        if (dt_ms < arucotag_pause_ms)
            usleep(arucotag_pause_ms - dt_ms);
        return arucotag_poll;
    }
}


/** Codel detect_main of task detect.
 *
 * Triggered by arucotag_main.
 * Yields to arucotag_poll.
 */
genom_event
detect_main(const arucotag_frame *frame, float length,
            const arucotag_calib *calib, const arucotag_drone *drone,
            const arucotag_detector *tags,
            const sequence_arucotag_portinfo *ports,
            const arucotag_pose *pose,
            const arucotag_pixel_pose *pixel_pose, int16_t out_frame,
            arucotag_log_s **log, const genom_context self)
{
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
    aruco::detectMarkers(cvframe, tags->dict, corners, ids);

    // Publish empty messages for tags that are not detected
    for (uint16_t i=0; i<ports->_length; i++)
        if (find(ids.begin(), ids.end(), std::stoi(ports->_buffer[i])) == ids.end())
        {
            pose->data(ports->_buffer[i], self)->ts = fdata->ts;
            pose->data(ports->_buffer[i], self)->pos._present = false;
            pose->data(ports->_buffer[i], self)->pos_cov._present = false;
            pose->data(ports->_buffer[i], self)->att._present = false;
            pose->data(ports->_buffer[i], self)->att_cov._present = false;
            pose->data(ports->_buffer[i], self)->att_pos_cov._present = false;
            pose->write(ports->_buffer[i], self);

            pixel_pose->data(ports->_buffer[i], self)->ts = pose->data(ports->_buffer[i], self)->ts;
            pixel_pose->data(ports->_buffer[i], self)->pix._present = false;
            pixel_pose->write(ports->_buffer[i], self);
        }

    // Sleep if no detection was made
    if (ids.size() == 0)
        return arucotag_poll;

    // Estimate pose from corners
    vector<Vec3d> translations, rotations;
    aruco::estimatePoseSingleMarkers(corners, length, calib->K_cv, calib->D, rotations, translations);

    // Get state feedback
    Vector3d W_p_B;
    Matrix3d W_R_B;
    Quaterniond W_q_B;
    Matrix3d S_W_p_B;
    Matrix4d S_W_q_B;
    if (!(drone->read(self) == genom_ok && drone->data(self)))
    {
        // Give default value if unable to read from port
        W_p_B = Vector3d::Zero();
        W_R_B = Matrix3d::Identity();
        S_W_p_B = Matrix3d::Zero();
        S_W_q_B = Matrix4d::Zero();
    }
    else
    {
        or_pose_estimator_state* pom = drone->data(self);
        W_p_B << pom->pos._value.x, pom->pos._value.y, pom->pos._value.z;
        W_q_B = Quaterniond(pom->att._value.qw, pom->att._value.qx, pom->att._value.qy, pom->att._value.qz);
        W_R_B = W_q_B;
        S_W_p_B <<
            pom->pos_cov._value.cov[0], pom->pos_cov._value.cov[1], pom->pos_cov._value.cov[3],
            pom->pos_cov._value.cov[1], pom->pos_cov._value.cov[2], pom->pos_cov._value.cov[4],
            pom->pos_cov._value.cov[3], pom->pos_cov._value.cov[4], pom->pos_cov._value.cov[5];
        S_W_q_B <<
            pom->att_cov._value.cov[0], pom->att_cov._value.cov[1], pom->att_cov._value.cov[3], pom->att_cov._value.cov[6],
            pom->att_cov._value.cov[1], pom->att_cov._value.cov[2], pom->att_cov._value.cov[4], pom->att_cov._value.cov[7],
            pom->att_cov._value.cov[3], pom->att_cov._value.cov[4], pom->att_cov._value.cov[5], pom->att_cov._value.cov[8],
            pom->att_cov._value.cov[6], pom->att_cov._value.cov[7], pom->att_cov._value.cov[8], pom->att_cov._value.cov[9];
    }

    // Process detected tags
    for (uint16_t i=0; i<ids.size(); i++)
    {
        // Check that detected tags are among tracked markers
        const char* id = to_string(ids[i]).c_str();
        uint16_t j = 0;
        for (j=0; j<ports->_length; j++)
            if (!strcmp(ports->_buffer[j], id))
                break;
        if (j >= ports->_length) continue;

        // Get translation and rotation
        Vector3d C_p_M(translations[i][0], translations[i][1], translations[i][2]);
        Mat tmp = Mat::zeros(3,3, CV_64F);
        Rodrigues(rotations[i], tmp);
        Matrix3d C_R_M;
        cv2eigen(tmp, C_R_M);

        // Compute covariance
        // See Sec. VI.B in [Jacquet 2020] (10.1109/LRA.2020.3045654)
        float sigma_p = 3;              // Arbitrary isotropic pixel error
        Matrix<double,8,6> J;           // Jacobian of f^-1

        Matrix<double,3,4> c; c <<      // Coordinates of corners in marker frame
            -1,  1,  1, -1,
            -1, -1,  1,  1,
             0,  0,  0,  0;
        c = c * length/2;

        for (uint16_t i=0; i<4; i++)
        {
            Vector3d ci = c.col(i);
            Vector3d hi = calib->K * (C_R_M * ci + C_p_M);
            // Jacobian of pixellization (homogeneous->pixel) operation wrt homogeneous coordinates
            Matrix<double,2,3> J_pix; J_pix <<
                1/hi(2), 0, -hi(0)/hi(2)/hi(2),
                0, 1/hi(2), -hi(1)/hi(2)/hi(2);
            // Jacobian of projection
            Matrix<double,3,6> J_proj;
            // Jacobian of projection wrt translation
            J_proj.block(0,0,3,3) = calib->K;
            // Jacobian of projection wrt rotation
            Matrix3d skew; skew <<
                    0 , -ci(2),  ci(1),
                 ci(2),     0 , -ci(0),
                -ci(1),  ci(0),     0 ;
            J_proj.block(0,3,3,3) = -calib->K * C_R_M * skew;
            // Jacobian of projection wrt euclidean coordinates (chain rule)
            Matrix<double,2,6> J_full = J_pix*J_proj;
            // Stack in J
            J.block(i*2,0,2,6) = J_full;
        }

        // First order propagation
        // Cross (pos/rot) covariance is neglected since I dunno how to transform it into quaterion covariance
        Matrix<double,6,6> cov = sigma_p*sigma_p * (J.transpose() * J).inverse();
        Matrix3d cov_pos = cov.block(0,0,3,3);
        Matrix3d cov_rot = cov.block(3,3,3,3);

        // Transform to desired frame and propagate covariance
        Vector3d position;
        Quaterniond orientation;
        switch (out_frame)
        {
            case 0:  // Camera frame
                position = C_p_M;
                orientation = C_R_M;
                break;
            case 1:  // Body frame
                position = calib->B_R_C * C_p_M + calib->B_p_C;
                orientation = calib->B_R_C * C_R_M;
                cov_pos = calib->B_R_C * cov_pos * calib->B_R_C.transpose();
                cov_rot = calib->B_R_C * cov_rot * calib->B_R_C.transpose();
                break;
            case 2:  // World frame
                Vector3d B_p_M = calib->B_R_C * C_p_M + calib->B_p_C;
                position = W_R_B * B_p_M + W_p_B;
                orientation = W_R_B * calib->B_R_C * C_R_M;
                // Propagate to body frame
                cov_pos = calib->B_R_C * cov_pos * calib->B_R_C.transpose();
                cov_rot = calib->B_R_C * cov_rot * calib->B_R_C.transpose();
                // Propagate to world frame
                // The jacobian of the transformation wrt the quaternion W_q_B is given by Eq. 174 from [Solà 2017], see Sec. 4.3.2 therein
                // Available at: https://arxiv.org/abs/1711.02508
                Matrix3d B_p_M_skew; B_p_M_skew <<
                           0 , -B_p_M(2),  B_p_M(1),
                     B_p_M(2),        0 , -B_p_M(0),
                    -B_p_M(1),  B_p_M(0),        0 ;
                Matrix<double,3,4> J_R;
                J_R.col(0) = 2* (W_q_B.w()*B_p_M + W_q_B.vec().cross(B_p_M));
                J_R.block(0,1,3,3) = 2* ((W_q_B.vec().transpose() * B_p_M)(0) * Matrix3d::Identity() + W_q_B.vec() * B_p_M.transpose() - W_q_B.vec() * B_p_M.transpose() - W_q_B.w() * B_p_M_skew);
                cov_pos = W_R_B * cov_pos * W_R_B.transpose() + J_R * S_W_q_B * J_R.transpose() + S_W_p_B;
                cov_rot = W_R_B * cov_rot * W_R_B.transpose() + J_R * S_W_q_B * J_R.transpose();
                break;
        }

        // Convert rotation covariance (i.e. element of SO(3)) to quaternion covariance (i.e. element of R^4)
        // We consider the Jacobian of the inverse transformation T: q -> theta*u which is easier to compute
        // 1/qv_norm^3 is extracted out of the matrix to avoid numerical issue before (pseudo-)inversion, and reintegrated afterward
        Matrix<double,3,4> J_T; // Jacobian of T
        double qw = orientation.w();
        double qx = orientation.x();
        double qy = orientation.y();
        double qz = orientation.z();
        double qv_norm = orientation.vec().norm();
        double theta = 2*atan2(qv_norm, qw);
        J_T(0,0) = 2*pow(qv_norm,3)*qx/(qw*qw+qv_norm*qv_norm);
        J_T(1,0) = 2*pow(qv_norm,3)*qy/(qw*qw+qv_norm*qv_norm);
        J_T(2,0) = 2*pow(qv_norm,3)*qz/(qw*qw+qv_norm*qv_norm);
        J_T(0,1) = pow(qv_norm,2)*theta + pow(qx,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(1,1) = pow(qx,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(2,1) = pow(qx,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(0,2) = pow(qy,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(1,2) = pow(qv_norm,2)*theta + pow(qy,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(2,2) = pow(qy,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(0,3) = pow(qz,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(1,3) = pow(qz,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        J_T(2,3) = pow(qv_norm,2)*theta + pow(qz,2)*(theta-(2*qw*qv_norm)/(qw*qw+qv_norm*qv_norm));
        Matrix<double,4,3> J_Tm1; // Jacobian of T^-1: theta*u -> q
        Mat J_tmp;
        eigen2cv(J_T, J_tmp); // Switch to opencv for pseudoinversion because eigen is shit
        invert(J_tmp, J_tmp, DECOMP_SVD); // SVD handles pseudoinversion of rectangular matrices
        cv2eigen(J_tmp, J_Tm1);
        J_Tm1 = J_Tm1 * pow(qv_norm,3);
        // Potential workaround here is to declare J_T as a dynamic sized matrix (otherwise ComputeThinU will complain) and use the following lines:
        //  JacobiSVD<Matrix<double,3,4>> svd(J_T, ComputeThinU | ComputeThinV);
        //  double tolerance = std::numeric_limits<double>::epsilon() * std::max(J_T.cols(), J_T.rows()) *svd.singularValues().array().abs()(0);
        //  J_Tm1 = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

        // Propagation of covariance
        Matrix4d cov_q = J_Tm1 * cov_rot * J_Tm1.transpose();

        // Publish
        const char* tagid = to_string(ids[i]).c_str(); // not using the id variable since it gets fucked up by calling Mat J_pix = ... for some reason

        pose->data(tagid, self)->ts = fdata->ts;

        pose->data(tagid, self)->pos._present = true;
        pose->data(tagid, self)->pos._value.x = position(0);
        pose->data(tagid, self)->pos._value.y = position(1);
        pose->data(tagid, self)->pos._value.z = position(2);
        pose->data(tagid, self)->att._present = true;
        pose->data(tagid, self)->att._value.qw = orientation.w();
        pose->data(tagid, self)->att._value.qx = orientation.x();
        pose->data(tagid, self)->att._value.qy = orientation.y();
        pose->data(tagid, self)->att._value.qz = orientation.z();
        pose->data(tagid, self)->pos_cov._present = true;
        pose->data(tagid, self)->pos_cov._value =
        {
            cov_pos(0,0),
            cov_pos(1,0), cov_pos(1,1),
            cov_pos(2,0), cov_pos(2,1), cov_pos(2,2)
        };
        pose->data(tagid, self)->att_cov._present = true;
        pose->data(tagid, self)->att_cov._value =
        {
            cov_q(0,0),
            cov_q(1,0), cov_q(1,1),
            cov_q(2,0), cov_q(2,1), cov_q(2,2),
            cov_q(3,0), cov_q(3,1), cov_q(3,2), cov_q(3,3)
        };

        pose->write(tagid, self);

        // Compute centroid of tag in pixel coordinates
        Point2f center(0, 0);
        for(int p = 0; p < 4; p++)
            center += corners[i][p];
        center = center / 4.;

        pixel_pose->data(tagid, self)->ts = pose->data(tagid, self)->ts;
        pixel_pose->data(tagid, self)->pix._present = true;
        pixel_pose->data(tagid, self)->pix._value.x = round(center.x);
        pixel_pose->data(tagid, self)->pix._value.y = round(center.y);
        pixel_pose->write(tagid, self);

        // Save for logs
        // Convert quaternion to roll/pitch/yaw
        Vector3d rpy(
            atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy)),
            asin(2 * (qw*qy - qz*qx)),
            atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
        );

        // Log
        arucotag_log_f(log, out_frame, tagid, round(center.x), round(center.y), pose->data(tagid, self));
    }

    return arucotag_poll;
}


/* --- Activity add_marker ---------------------------------------------- */

/** Codel add_marker of activity add_marker.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_ether.
 * Throws arucotag_e_io.
 */
genom_event
add_marker(const char marker[16], sequence_arucotag_portinfo *ports,
           const arucotag_pose *pose,
           const arucotag_pixel_pose *pixel_pose,
           const genom_context self)
{
    // Check if marker is already in port list
    uint16_t i;
    for(i=0; i<ports->_length; i++)
        if (!strcmp(ports->_buffer[i], marker))
        {
            arucotag_e_io_detail d;
            snprintf(d.what, sizeof(d.what), "%s", "marker already tracked");
            warnx("io error: %s", d.what);
            return arucotag_e_io(&d,self);
        }

    // Add it
    if (i >= ports->_maximum)
        if (genom_sequence_reserve(ports, i + 1))
            return arucotag_e_sys_error("add", self);
    (ports->_length)++;
    strncpy(ports->_buffer[i], marker, 16);

    // Init new out ports
    pixel_pose->open(marker, self);
    pose->open(marker, self);

    // Publish empty message
    timeval tv;
    gettimeofday(&tv, NULL);

    pose->data(marker, self)->ts.sec = tv.tv_sec;
    pose->data(marker, self)->ts.nsec = tv.tv_usec*1000;
    pose->data(marker, self)->pos._present = false;
    pose->data(marker, self)->pos_cov._present = false;
    pose->data(marker, self)->att._present = false;
    pose->data(marker, self)->att_cov._present = false;
    pose->write(marker, self);

    pixel_pose->data(marker, self)->ts = pose->data(marker, self)->ts;
    pixel_pose->data(marker, self)->pix._present = false;
    pixel_pose->write(marker, self);

    warnx("tracking new marker: %s", marker);
    return arucotag_ether;
}


/* --- Activity remove_marker ------------------------------------------- */

/** Codel remove_marker of activity remove_marker.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_ether.
 * Throws arucotag_e_io.
 */
genom_event
remove_marker(const char marker[16], sequence_arucotag_portinfo *ports,
              const arucotag_pose *pose,
              const arucotag_pixel_pose *pixel_pose,
              const genom_context self)
{
    // Look for marker in port list
    uint16_t i;
    for(i=0; i<ports->_length; i++)
        if (!strcmp(ports->_buffer[i], marker))
            break;

    // If marker not found, throw exception
    if (i >= ports->_length)
    {
        arucotag_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "marker not tracked");
        warnx("io error: %s", d.what);
        return arucotag_e_io(&d,self);
    }
    // Otherwise, remove it:
    else
    {
        // Move all following ports one step back in the array
        for (uint16_t j=i; j<ports->_length; j++)
            strncpy(ports->_buffer[j], ports->_buffer[j+1], 16);
        // Remove last element of list
        (ports->_length)--;
    }

    // Closing will cause poster closed when other components will try to read on the port
    pixel_pose->close(marker, self);
    pose->close(marker, self);

    warnx("stop tracking marker: %s", marker);
    return arucotag_ether;
}


/* --- Activity set_calib ----------------------------------------------- */

/** Codel set_calib of activity set_calib.
 *
 * Triggered by arucotag_start.
 * Yields to arucotag_ether.
 * Throws arucotag_e_io.
 */
genom_event
set_calib(const arucotag_intrinsics *intrinsics,
          const arucotag_extrinsics *extrinsics,
          arucotag_calib **calib, const genom_context self)
{
    if (intrinsics->read(self) != genom_ok || !intrinsics->data(self) ||
        extrinsics->read(self) != genom_ok || !extrinsics->data(self))
    {
        arucotag_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "cannot read calibration input ports");
        warnx("io error: %s", d.what);
        return arucotag_e_io(&d,self);
    }
    update_calib(intrinsics, extrinsics, calib, self);
    return arucotag_ether;
}
