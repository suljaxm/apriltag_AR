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

#include "opencv2/opencv.hpp"
#include "apriltag_pose.h"
#include <opencv2/viz.hpp>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"
}

using namespace std;
using namespace cv;

Mat r, t;

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
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
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
	info.tagsize = 0.02;
	info.fx = 611.963;
	info.fy = 611.963;
	info.cx = 309.007;
	info.cy = 242.101;


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    Mat frame, gray;
    double t[3] = {0}, R[9] = {1,0,0,0,1,0,0,0,1};
    while (true) {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

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
            wx = pose.t->data[0];
            wy = pose.t->data[1];
            wz = pose.t->data[2];
			cout << wx <<"  "<< wy << " " << wz << endl;
            double dist = sqrt( wx*wx + wy*wy + wz*wz );
            cout << "distance = " << dist << endl;
			cout << "R size:" << pose.R->nrows <<" "<< pose.R->ncols << endl;  //3*3
            cout << "R:" << endl;
            cout << pose.R->data[0] <<" "<< pose.R->data[1] << " " << pose.R->data[2] << endl;  //3*3
            cout << pose.R->data[3] <<" "<< pose.R->data[4] << " " << pose.R->data[5] << endl;  //3*3
            cout << pose.R->data[6] <<" "<< pose.R->data[7] << " " << pose.R->data[8] << endl;  //3*3
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
            
            t[0] = wx;
            t[1] = wy;
            t[2] = wz;
            for( int i = 0; i < 9; i++ ){
                R[i] = pose.R->data[i];
            }

        }
        apriltag_detections_destroy(detections);

        imshow("Tag Detections", frame);

        // visualization
        cv::viz::Viz3d vis ( "Visual Odometry" );
        cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
        cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
        cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
        vis.setViewerPose ( cam_pose );

        world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
        camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
        vis.showWidget ( "World", world_coor );
        vis.showWidget ( "Camera", camera_coor );

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                R[0], R[1], R[2],
                R[3], R[4], R[5],
                R[6], R[7], R[8]
                ),
            cv::Affine3d::Vec3 (
                t[0], t[1], t[2]
            )
        );

        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        

        if (waitKey(30) >= 0)
            break;
    }

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } 

    getopt_destroy(getopt);

    return 0;
}
