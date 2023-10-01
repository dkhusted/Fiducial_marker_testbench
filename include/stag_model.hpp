#include "mqtt.hpp"
#include "globals.hpp"

#include "../models/stag/src/Stag.h"
#include "opencv2/opencv.hpp"

void run_STag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    stag_settings_t &stag_settings,
    settings_t &settings);