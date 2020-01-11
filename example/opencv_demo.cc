/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "apriltag_pose.h"
#include <pangolin/pangolin.h>
#include <sophus/se3.h>
#include <boost/format.hpp>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"
}


Eigen::Matrix3d cam_R;
Eigen::Vector3d cam_t;

typedef std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > VecSE3;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VecVec3d;

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points, const apriltag_detection_info_t &info);

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    cv::VideoCapture cap(1);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detection_info_t info;
    info.tagsize = 13.8; //cm
    info.fx = 9.7185090185201193e+02;
    info.fy = 9.3550247129566696e+02;
    info.cx = 3.2389824020348124e+02;
    info.cy = 2.1394131657430057e+02;

    double camera_matrix[] =
    {
        info.fx,    0.0,       info.cx,
        0.0,        info.fy,   info.cy,
        0.0,        0.0,       1.0
    };
    double dist_coeff[] = {-1.0351230255308908e+00, -1.9662350850900434e+00, 0.0, 0.0};

    cv::Mat m_camera_matrix = cv::Mat(3, 3, CV_64FC1, camera_matrix).clone();
    cv::Mat m_dist_coeff = cv::Mat(1, 4, CV_64FC1, dist_coeff).clone();


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    cv::Mat frame, gray;
    VecSE3 poses;
    VecVec3d points;

    while (true) {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        // cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            info.det = det; 
            apriltag_pose_t pose;
			double err = estimate_tag_pose(&info, &pose);
            double wx, wy, wz;

            double scale = 107.0/220.0;  //depth scale = realvalue / measurement = 107cm/220

            wx = pose.t->data[0]*scale;
            wy = pose.t->data[1]*scale;
            wz = pose.t->data[2]*scale;
			// cout << wx <<"  "<< wy << " " << wz << endl;
            
            double depth = sqrt( wx*wx + wy*wy + wz*wz );
            
            // cout << "depth = " << depth << endl;
			// cout << "R size:" << pose.R->nrows <<" "<< pose.R->ncols << endl;  //3*3
            // cout << "R:" << endl;
            // cout << pose.R->data[0] <<" "<< pose.R->data[1] << " " << pose.R->data[2] << endl;  //3*3
            // cout << pose.R->data[3] <<" "<< pose.R->data[4] << " " << pose.R->data[5] << endl;  //3*3
            // cout << pose.R->data[6] <<" "<< pose.R->data[7] << " " << pose.R->data[8] << endl;  //3*3
            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);
            cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0xff, 0xff), 2);

            cv::circle(frame, cv::Point(det->p[0][0], det->p[0][1]),
                2,
                cv::Scalar(0xff, 0xff, 0), 2); 

            std::stringstream ss;
            ss << det->id;
            cv::String text = ss.str();
            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);


            cam_t << wx, wy, wz;
            cam_R << pose.R->data[0], pose.R->data[1], pose.R->data[2],
                pose.R->data[3], pose.R->data[4], pose.R->data[5],
                pose.R->data[6], pose.R->data[7], pose.R->data[8];
            

            // three-dimensional cube test (cudePoints)
            std::vector< cv::Point3f > cubePoints;
            cubePoints.push_back(cv::Point3f(-0.5, -0.5, 0.0));
            cubePoints.push_back(cv::Point3f( 0.5, -0.5, 0.0));
            cubePoints.push_back(cv::Point3f( 0.5,  0.5, 0.0));
            cubePoints.push_back(cv::Point3f(-0.5,  0.5, 0.0));
            cubePoints.push_back(cv::Point3f(-0.5, -0.5, 1.0));
            cubePoints.push_back(cv::Point3f( 0.5, -0.5, 1.0));
            cubePoints.push_back(cv::Point3f( 0.5,  0.5, 1.0));
            cubePoints.push_back(cv::Point3f(-0.5,  0.5, 1.0));

            for ( auto &p:cubePoints ){
                p = p*5;
            }

            std::vector< cv::Point2f > imagePoints;
            bool 
            cv::Mat cv_cam_R, cv_cam_t;
            cv::eigen2cv(cam_R, cv_cam_R);
            cv::eigen2cv(cam_t, cv_cam_t);
            cv::projectPoints(cubePoints, cv_cam_R, cv_cam_t, m_camera_matrix, m_dist_coeff, imagePoints);
            // draw cube lines
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[1], imagePoints[2], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[2], imagePoints[3], cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, imagePoints[3], imagePoints[0], cv::Scalar(0, 0, 0xff), 2);

            cv::line(frame, imagePoints[4], imagePoints[5], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[5], imagePoints[6], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[6], imagePoints[7], cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, imagePoints[7], imagePoints[4], cv::Scalar(0xff, 0, 0), 2);

            cv::line(frame, imagePoints[0], imagePoints[4], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[1], imagePoints[5], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[2], imagePoints[6], cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, imagePoints[3], imagePoints[7], cv::Scalar(0, 0xff, 0), 2);


            // Note that every variable that we compute is proportional to the scale factor of H.
            // or pnp, cv::solvePnP  cv::Rodrigues

            double H00 = MATD_EL(det->H, 0, 0);
            double H01 = MATD_EL(det->H, 0, 1);
            double H02 = MATD_EL(det->H, 0, 2);
            double H10 = MATD_EL(det->H, 1, 0);
            double H11 = MATD_EL(det->H, 1, 1);
            double H12 = MATD_EL(det->H, 1, 2);
            double H20 = MATD_EL(det->H, 2, 0);
            double H21 = MATD_EL(det->H, 2, 1);
            double H22 = MATD_EL(det->H, 2, 2);
            // cout << "H = " << H00 << " " << H01 << endl;
            


            Sophus::SE3 SE3_Rt( cam_R, cam_t );
            poses.push_back( SE3_Rt );
            for( int i = 0; i < 4; i++ ){
                Eigen::Vector3d p_tag_c, p_tag_w;
                p_tag_c[2] = depth;
                p_tag_c[0] = ( det->p[i][0] - info.cx )*p_tag_c[2]/info.fx;
                p_tag_c[1] = ( det->p[i][1] - info.cy )*p_tag_c[2]/info.fy;
                p_tag_w = SE3_Rt.inverse()*p_tag_c;
                points.push_back( p_tag_w );
                if( i == 0 ){
                    // cout << "p_tag_c: " << p_tag_c.transpose() << endl;
                    // cout << "p_tag_w: " << p_tag_w.transpose() << endl;
                }
            }


        }
        


        apriltag_detections_destroy(detections);

        imshow("Tag Detections", frame);

        if (cv::waitKey(30) == 'q')
            break;
    }

    Draw(poses, points, info);
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } 


    getopt_destroy(getopt);

    return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points, const apriltag_detection_info_t &info) {
    if (poses.empty() || points.empty()) {
        std::cerr << "parameter is empty!" << std::endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // intrinsics
        float fx = info.fx;
        float fy = info.fy;
        float cx = info.cx;
        float cy = info.cy;

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
