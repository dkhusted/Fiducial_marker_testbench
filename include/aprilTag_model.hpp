#pragma once

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>

#ifdef __linux__
    #include <unistd.h>
#endif

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"

#include "apriltag/common/getopt.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/zarray.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"

#define  HAMM_HIST_MAX 10

#include "mqtt.hpp"
#include "globals.hpp"

void run_aprilTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    apriltag_settings_t &apriltag_settings,
    settings_t &settings);

/**
 * @brief Draw the detected marker int the given image. The markers are drawn as a
 * square. It draws the square in green and center as a red dot.
 *
 * @param[in] markers The list of markers to draw.
 * @param[out] image The image in which to draw the markers.
 */
void drawAprilTagMarkers(const apriltag_detection_t &marker, cv::Mat& image);

