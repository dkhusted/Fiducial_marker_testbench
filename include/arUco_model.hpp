#pragma once

#include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "mqtt.hpp"
#include "globals.hpp"

void run_arUcoTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    aruco_settings_t &aruco_settings,
    settings_t &settings);

// void run_cctag(cv::VideoCapture capture);

// void run_stag(cv::VideoCapture capture);

// void run_apriltag(cv::VideoCapture capture);

// void run_pitag(cv::VideoCapture capture);