#include "../include/mqtt.hpp"
#include "../models/stag/src/Stag.h"
#include "../include/arUco_model.hpp"
#include "../include/globals.hpp"
#include "../include/aprilTag_model.hpp"
#include "../include/stag_model.hpp"
#include "../include/piTag_model.hpp"
#include <thread>
#include "../include/ccTag_model.hpp"
#include <opencv2/core/persistence.hpp>

/*
    TODO:
    * Add config file feature:
        - Import Apriltag familyname as string
        - Import Pitag model filepath as string
    
    BUG:
        - ccTag_tracking.csv contains corrupt ids
*/


// Default Settings. Change to settings should happen in the config.json.

apriltag_settings_t *apriltag_settings_pt;
pitag_settings_t *pitag_settings_pt;
cctag_settings_t *cctag_settings_pt;
aruco_settings_t *aruco_settings_pt;
stag_settings_t *stag_settings_pt;
settings_t *settings_pt;

int arUco_id_array[] = {1, 2, 3};
int aprilTag_id_array[] = {0, 1, 2};
int stag_id_array[] = {0, 1, 2}; 
int piTag_id_array[] = {0, 1, 2};
int ccTag_id_array[] = {0, 1, 2};

char buffer[1500];

// PiTag model to use. You can find more models in ../models/piTag/algorithms/
int camera_width = 480;
int camera_height = 640;
int camera_fps = 30;
std::mutex g_mutex, g_write_mutex;
std::condition_variable g_cv, g_angler_cv, g_slider_cv, g_master;
bool g_ready = false;
bool g_master_ready = false;

int load_config(char *json_file);

using namespace cv;
using std::cout; using std::cerr; using std::endl;

const char TAG[] = "Main";

int main(int argc, char *argv[]){

    if (argc != 2)
    {
        puts("Please provide the path config.json file");
        return 1;
    }

    /*############## Settings ################################3*/

    char *json_file = argv[1];


    apriltag_settings_t apriltag_settings = {
    .enable_apriltag = false,
    .debug_apriltag = false
    };

    pitag_settings_t pitag_settings = {
        .enable_pitag = false,
        .debug_pitag = false,
    };

    cctag_settings_t cctag_settings = {
        .debug_cctag = false,
        .enable_cctag = false,
        .ccTag_nRings = 3
    };

    aruco_settings_t aruco_settings = {
        .debug_aruco = false,
        .enable_aruco = false
    };

    stag_settings_t stag_settings = {
        .enable_stag = false,
        .debug_stag = false,
        .stag_libraryHD = 17,
        .stag_errorCorrection = 7
    };

    settings_t settings = {
        .frames_to_capture = 30,
        .enable_no_marker_skip = false,
        .skipping_counter = 3,
        .save_picture_with_marker = false,
        .max_distance = 20,
        .max_angle = 20,
        .distance_step = 1,
        .angle_step = 1,
        .camera_id = ""
    };

    apriltag_settings_pt = &apriltag_settings;
    pitag_settings_pt = &pitag_settings;
    cctag_settings_pt = &cctag_settings;
    aruco_settings_pt = &aruco_settings;
    stag_settings_pt = &stag_settings;
    settings_pt = &settings;

    if(load_config(json_file) == 1){
        printf("%s: Error loading config\n", TAG);
        return 1;
    } 
    printf("%s: Config loaded\n", TAG);

    // printf("%d", )

    // /*############ MQTT SETUP ############################################################*/
    struct mosquitto *publisher;
    struct mosquitto *subscriber;
    int err;

    printf("Setting up MQTT\n");
    err = mosquitto_lib_init();
    if(err != MOSQ_ERR_SUCCESS){
        printf("Error: %s\n", mosquitto_strerror(err));
        exit(1);
    }

	publisher = mosquitto_new(NULL, true, NULL);
	if(publisher == NULL){
		fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}
    subscriber = mosquitto_new(NULL, true, NULL);
	if(subscriber == NULL){
		fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}

    if(mqtt_config(publisher, subscriber) != 0){
        printf("Error: mqtt_config failed\n");
        return 1;
    }
    // {
    //     std::lock_guard<std::mutex> lock_printf(g_write_mutex);
    //     printf("Main: Waiting for nodes to connect\n");
    // }

    
    
    // std::unique_lock<std::mutex> lck(g_mutex); // create a lock for the mutex and locks it
    // while(!g_ready) g_cv.wait(lck); 
    // // Unlocks the mutex and wait for a notifycation via g_cv(condition variable). 
    // // When it gets notified relocks the mutex, so we have to unlock it, otherwise it can not be accuired later.
    // g_ready = false;
    // lck.unlock();

    
    {
        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
        printf("MQTT setup done\n");
    }

    
    /*############ CAMERA Settings ###################################################*/


    // cv::FileStorage camera_parameters("camera_parameters.yml", cv::FileStorage::Mode::FORMAT_YAML);
    
    // cv::Mat camera_matrix, extrinsic_parameters, image_points;
    // cv::Mat dist_coeff;

    // if(!camera_parameters.isOpened()){

    //     {
    //         std::lock_guard<std::mutex> lock_printf(g_write_mutex);
    //         cerr << "Warning: Can't open camera config file, running without camera calibration. This will impact the results" << endl;
    //     }
        
    // }
    // else{
    //     camera_parameters["camera_matrix"] >> camera_matrix;
    //     camera_parameters["distortion_coefficients"] >> dist_coeff;
    //     camera_parameters["extrinsic_parameters"] >> extrinsic_parameters;
    //     camera_parameters["image_points"] >> image_points;
    //     // cout << camera_matrix << endl;
    //     // cout << dist_coeff << endl;
    // }

    {
        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
        cout << "Opening camera..." << endl;
    }

    //VideoCapture capture("/dev/testBench", CAP_V4L); // open the testBench camera. See README for how to set the camera up. CAP_V4L = Video4Linux
    // VideoCapture capture("/dev/cam_1080_C920", CAP_V4L);
    VideoCapture capture("/dev/video2", CAP_V4L2);
    if (!capture.isOpened())
    {
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            cerr << "ERROR: Can't initialize camera capture" << endl;

        }
        return 1;
    }

    {
        std::lock_guard<std::mutex> lock_printf(g_write_mutex);

        if(!capture.set(CAP_PROP_FPS, camera_fps)){
            printf("%s: Could set FPS to %d\n", TAG, camera_fps);
        }
        // else if(capture.get(CAP_PROP_FPS) != camera_fps){
        //     printf("%s: %d FPS was not set by camera, maybe not supported?\n", TAG, camera_fps);
        // }

        if(!capture.set(CAP_PROP_FRAME_HEIGHT, camera_height)){
            printf("%s: Could not set height to %dpx\n", TAG, camera_height);
        }
        // else if(capture.get(CAP_PROP_FRAME_HEIGHT != camera_height)){
        //     printf("%s: Height was not set to %d by camera, maybe not supported?\n", TAG, camera_height);
        // }

        if(!capture.set(CAP_PROP_FRAME_WIDTH, camera_width)){
            printf("%s: Could not set width to %dpx\n", TAG, camera_width);
        }
        // else if(capture.get(CAP_PROP_FRAME_WIDTH != camera_width)){
        //     printf("%s: Width was not set to %d by camera, maybe not supported?\n", TAG, camera_width);
        // }

        cout << "Frame width: " << capture.get(CAP_PROP_FRAME_WIDTH) << endl;
        cout << "     height: " << capture.get(CAP_PROP_FRAME_HEIGHT) << endl;
        cout << "Capturing FPS: " << capture.get(CAP_PROP_FPS) << endl;
    }

    // cout << endl << "Press 'ESC' to quit." << endl;

    // VideoCapture *capture_p = &capture;

    /*############ Model flow ############################################################*/
    {
        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
        printf("Waiting for start cmd from Node-Red UI\n");

    }

    std::unique_lock<std::mutex> lck(g_mutex); // create a lock for the mutex and locks it
    while(!g_master_ready) g_master.wait(lck);
    g_master_ready = false;
    lck.unlock();

    if(aruco_settings.enable_aruco){
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("Running arUco model\n");
        }

        // std::thread arUco_thread(run_arUcoTag, capture, 100, 90, 10, 10); // Start the arUco thread

        std::thread arUcoTag_thread(run_arUcoTag, 
            std::ref(capture), 
            publisher,
            arUco_id_array,
            std::ref(aruco_settings),
            std::ref(settings)); // Start the arUco thread

        // while(!g_ready) g_cv.wait(lck); // Wait for the arUco thread to finish
        // g_ready = false;
        // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // add time for thread to exit scope, so all resources are freed

        arUcoTag_thread.join(); // wait for the thread to finish
        if(apriltag_settings.enable_apriltag){
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("arUco model done. Change markers to Apriltag. Press start in Node-Red UI to continue\n");

            }

            lck.lock();
            while(!g_master_ready) g_master.wait(lck); // Wait for node-red continue signal
            g_master_ready = false;
            lck.unlock();
        }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if(apriltag_settings.enable_apriltag){
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("Running AprilTag model\n");
        }

        //TODO: add parameters for aprilTag. Should be loaded from a config file
        std::thread aprilTag_thread(run_aprilTag, 
            std::ref(capture), 
            publisher,
            aprilTag_id_array,
            std::ref(apriltag_settings),
            std::ref(settings)); 

        // while(!g_ready) g_cv.wait(lck); 
        // g_ready = false;
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        aprilTag_thread.join();

        if(stag_settings.enable_stag){
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("AprilTag model done. Change markers to STag. Press start in Node-Red UI to continue\n");
            }

            lck.lock();
            while(!g_master_ready) g_master.wait(lck); // Wait for the arUco thread to finish
            g_master_ready = false;
            lck.unlock();
        }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if(stag_settings.enable_stag)
    {
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("Running STag model\n");
        }

        //TODO: add parameters for stag. Should be loaded from a config file
        std::thread stag_thread(run_STag, 
            std::ref(capture), 
            publisher,
            stag_id_array,
            std::ref(stag_settings),
            std::ref(settings)); 

        // while(!g_ready) g_cv.wait(lck); 
        // g_ready = false;
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        stag_thread.join();

        if(pitag_settings.enable_pitag){
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("STag model done. Change markers to piTag. Press start in Node-Red UI to continue\n");
            }
            lck.lock();
            while(!g_master_ready) g_master.wait(lck); // Wait for the arUco thread to finish
            g_master_ready = false;
            lck.unlock();

        }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));


    if(pitag_settings.enable_pitag){
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("Running piTag model\n");
        }
        std::thread piTag_thread(run_piTag, 
            std::ref(capture), 
            publisher,
            piTag_id_array,
            std::ref(pitag_settings),
            std::ref(settings));
        // while(!g_ready) g_cv.wait(lck); 
        // g_ready = false;

        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        piTag_thread.join(); // wait for the thread to finish

        if(cctag_settings.enable_cctag){
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("piTag model done. Change markers to ccTag. Press start in Node-Red UI to continue\n");
            }

            lck.lock();
            while(!g_master_ready) g_master.wait(lck); // Wait for the arUco thread to finish
            g_master_ready = false;
            lck.unlock();
        }
    }

    if(cctag_settings.enable_cctag){
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("Running ccTag model\n");
        }

        std::thread ccTag_thread(run_ccTag, 
            std::ref(capture), 
            publisher,
            ccTag_id_array,
            std::ref(cctag_settings),
            std::ref(settings));

        ccTag_thread.join(); // wait for the thread to finish

        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("ccTag model done.\n");
        }
    }

    {
        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
        printf("Exiting testBench program.\n");
    }

    /*############ CLEANUP ############################################################*/
	err = mosquitto_loop_stop(subscriber, true);
	if(err != MOSQ_ERR_SUCCESS){
		mosquitto_destroy(subscriber);
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(err));
		return 1;
	}

    mosquitto_lib_cleanup();

    return 0;
}

int load_config(char *json_file){

    FILE *fp = fopen(json_file, "r");
    if( fp == NULL){
        printf("%s: Unable to open file.\n");
        return 1;
    }

    int len = fread(buffer, 1, sizeof(buffer), fp);
    fclose(fp);
  
    // parse the JSON data
    cJSON *json = cJSON_Parse(buffer);
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Error: %s\n", error_ptr);
        }
        cJSON_Delete(json);
        return 1;
    }

    cJSON *arUcoTag_json = cJSON_GetObjectItem(json, "arUco");
    cJSON *cctag_json = cJSON_GetObjectItem(json, "cctag");
    cJSON *apriltag_json = cJSON_GetObjectItem(json, "apriltag");
    cJSON *stag_json = cJSON_GetObjectItem(json, "stag");
    cJSON *pitag_json = cJSON_GetObjectItem(json, "pitag");

    settings_pt->angle_step = cJSON_GetObjectItem(json, "angle_step")->valueint;
    // settings_pt->camera_id = cJSON_Print(cJSON_GetObjectItem(json, "camera_id"));
    settings_pt->distance_step = cJSON_GetObjectItem(json, "distance_step")->valueint;
    settings_pt->frames_to_capture = cJSON_GetObjectItem(json, "frames_to_capture")->valueint;
    settings_pt->max_angle = cJSON_GetObjectItem(json, "max_angle")->valueint;
    settings_pt->max_distance = cJSON_GetObjectItem(json, "max_distance")->valueint;
    settings_pt->skipping_counter = cJSON_GetObjectItem(json, "skipping_counter")->valueint;

    camera_fps = cJSON_GetObjectItem(json, "camera_fps")->valueint;
    camera_height = cJSON_GetObjectItem(json, "camera_height")->valueint;
    camera_width = cJSON_GetObjectItem(json, "camera_width")->valueint;

    cJSON *enable = NULL;

    enable = cJSON_GetObjectItemCaseSensitive(json, "save_picture_with_markers");
    settings_pt->save_picture_with_marker = cJSON_IsTrue(enable) ? true: false;

    enable = cJSON_GetObjectItemCaseSensitive(json, "enable_no_marker_skip");
    settings_pt->enable_no_marker_skip = cJSON_IsTrue(enable) ? true: false;

    enable = cJSON_GetObjectItemCaseSensitive(arUcoTag_json, "enable");
    aruco_settings_pt->enable_aruco = cJSON_IsTrue(enable) ? true : false;
    enable = cJSON_GetObjectItemCaseSensitive(arUcoTag_json, "debug");
    aruco_settings_pt->debug_aruco = cJSON_IsTrue(enable) ? true : false;

    enable = cJSON_GetObjectItemCaseSensitive(cctag_json, "debug");
    cctag_settings_pt->debug_cctag = cJSON_IsTrue(enable) ? true : false;
    enable = cJSON_GetObjectItemCaseSensitive(cctag_json, "enable");
    cctag_settings_pt->enable_cctag = cJSON_IsTrue(enable) ? true : false;
    cctag_settings_pt->ccTag_nRings = cJSON_GetObjectItem(cctag_json, "Rings_number")->valueint;

    enable = cJSON_GetObjectItemCaseSensitive(stag_json, "enable");
    stag_settings_pt->enable_stag = cJSON_IsTrue(enable) ? true : false; 
    enable = cJSON_GetObjectItemCaseSensitive(stag_json, "debug");
    stag_settings_pt->debug_stag = cJSON_IsTrue(enable) ? true : false; 
    stag_settings_pt->stag_errorCorrection = cJSON_GetObjectItem(stag_json, "error_correction")->valueint;
    stag_settings_pt->stag_libraryHD = cJSON_GetObjectItem(stag_json, "library_id")->valueint;

    enable = cJSON_GetObjectItemCaseSensitive(apriltag_json, "enable");
    apriltag_settings_pt->enable_apriltag = cJSON_IsTrue(enable) ? true : false; 
    enable = cJSON_GetObjectItemCaseSensitive(apriltag_json, "debug");
    apriltag_settings_pt->debug_apriltag = cJSON_IsTrue(enable) ? true : false; 

    enable = cJSON_GetObjectItemCaseSensitive(pitag_json, "enable");
    pitag_settings_pt->enable_pitag = cJSON_IsTrue(enable) ? true : false; 
    enable = cJSON_GetObjectItemCaseSensitive(pitag_json, "debug");
    pitag_settings_pt->debug_pitag = cJSON_IsTrue(enable) ? true : false; 

    // PiTag model to use. You can find more models in ../models/piTag/algorithms/
    // piTag_model_filename = cJSON_GetObjectItem(pitag_json, "piTag_model_filename")->valuestring;

    //Apriltag_model.cpp fails strcmp, maybe leading/trailing chars?
    // aprilTag_famname = cJSON_GetObjectItem(apriltag_json, "family")->valuestring;
    // char *test = strcat("/dev/", settings_pt->camera_id);
    // printf("%s: Camera Id: %s\n", TAG, test);
    // free(test);

    // printf("%d frames to capture\n", settings_pt->frames_to_capture);
    // printf("debug aruco %s", (aruco_settings_pt->debug_aruco ? "true" : "false"));

    cJSON_Delete(json);
    
}