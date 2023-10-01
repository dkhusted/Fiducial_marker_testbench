#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mqtt.hpp"
#include "globals.hpp"
#include "piTag/FiducialModelPi.h"
#include "piTag/FiducialDefines.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"


void run_piTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    pitag_settings_t &pitag_settings,
    settings_t &settings);