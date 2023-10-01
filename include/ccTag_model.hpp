/*
 * Copyright 2016, Simula Research Laboratory
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "cctag/Detection.hpp"
#include "cctag/utils/Exceptions.hpp"
#include "cctag/utils/FileDebug.hpp"
#include "cctag/utils/VisualDebug.hpp"
#include "mqtt.hpp"
#include "globals.hpp"

#ifdef CCTAG_WITH_CUDA
#include "cctag/cuda/debug_macros.hpp"
#include "cctag/cuda/device_prop.hpp"
#endif // CCTAG_WITH_CUDA

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/timer/timer.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifdef USE_DEVIL
#include <devil_cpp_wrapper.hpp>
#endif

#include <tbb/tbb.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define PRINT_TO_CERR

using namespace cctag;

namespace bfs = boost::filesystem;

/**
 * @brief Check if a string is an integer number.
 *
 * @param[in] s The string to check.
 * @return Return true if the string is an integer number
 */
bool ccTagisInteger(std::string& s);

/**
 * @brief Draw the detected marker int the given image. The markers are drawn as a
 * circle centered in the center of the marker and with its id. It draws the
 * well identified markers in green, the unknown / badly detected markers in red.
 *
 * @param[in] markers The list of markers to draw.
 * @param[out] image The image in which to draw the markers.
 */
bool ccTagdrawMarkers(const boost::ptr_list<CCTag>& markers, cv::Mat& image, bool showUnreliable);

/**
 * @brief Extract the cctag from an image.
 *
 * @param[in] frameId The number of the frame.
 * @param[in] pipeId The pipe id (used for multiple streams).
 * @param[in] src The image to process.
 * @param[in] params The parameters for the detection.
 * @param[in] bank The marker bank.
 * @param[out] markers The list of detected markers.
 * @param[out] outStream The output stream on which to write debug information.
 * @param[out] debugFileName The filename for the image to save with the detected
 * markers.
 */
void ccTagdetection(std::size_t frameId,
               int pipeId,
               const cv::Mat& src,
               const cctag::Parameters& params,
               const cctag::CCTagMarkersBank& bank,
               boost::ptr_list<CCTag>& markers,
               std::ostream& outStream,
               std::string debugFileName);


void run_ccTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    cctag_settings_t &cctag_settings,
    settings_t &settings);