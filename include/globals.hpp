#pragma once
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

#define TOTAL_FRAMES 30 // number of frames to capture for each position ~1s
extern std::mutex g_mutex;
extern std::condition_variable g_cv, g_angler_cv, g_slider_cv;
extern std::condition_variable g_master;
extern std::mutex g_write_mutex;
extern bool g_ready;
extern bool g_master_ready;



typedef struct{
    int frames_to_capture;
    bool enable_no_marker_skip;
    int skipping_counter;
    bool save_picture_with_marker;
    int max_distance;
    int max_angle;
    int distance_step;
    int angle_step;
    char *camera_id;
    bool save_picture_with_markers;
} settings_t;

typedef struct{
    bool enable_stag;
    bool debug_stag;
    int stag_libraryHD;
    int stag_errorCorrection;
}stag_settings_t;

typedef struct{
    bool debug_aruco;
    bool enable_aruco;
}aruco_settings_t;

typedef struct{
    bool debug_cctag;
    bool enable_cctag;
    int ccTag_nRings;
}cctag_settings_t;

typedef struct{
    bool enable_pitag;
    bool debug_pitag;    
}pitag_settings_t;

typedef struct{
    bool enable_apriltag;
    bool debug_apriltag;
}apriltag_settings_t;

