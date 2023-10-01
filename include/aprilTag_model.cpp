#include "aprilTag_model.hpp"

#include <thread>
#include <chrono>
#include <iostream>
// std::mutex aprilTag_mutex;

const char *famname = "tagStandard41h12"; //tag family to use for aprilTag


int apriltag_save_picture(int image_number, const char *folder, cv::Mat Image){
    char buffer[50] = "";
    char path[80] = "";

    if(sprintf(buffer, "%d.png", image_number) == 0){
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("ArUco: Failed to make folder path, not saving photo\n");
        }
        return 1;
    }
    else{

        if(strcat(path, folder) == NULL){
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("ArUco: Failed strcat folder path\n");
            }
            return 1;
        }
        else{
            if(strcat(path, buffer) == NULL){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("ArUco: Failed strcat picture path\n");
                }
                return 1;
            }
            else{

                // printf("%s\n", path);
                bool result = false;
                try
                {
                    result = imwrite(path, Image);
                }
                catch (const cv::Exception& ex)
                {
                    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
                    return 1;
                }
                if (result){
                    printf("Saved PNG file.\n");
                    return 0;
                }
                else{
                    printf("ERROR: Can't save PNG file.\n");
                    return 1;
                }
            }
        }

    }
}


void drawAprilTagMarkers(const apriltag_detection_t &marker, cv::Mat& image)
{

    const cv::Point center = cv::Point(marker.c[0], marker.c[1]);
    const int radius = 1;
    const int fontSize = 3;

    const cv::Scalar center_colour = cv::Scalar(0, 255, 0, 255);

    const cv::Scalar line_colour = cv::Scalar(180, 255, 125, 255);

    cv::circle(image, center, radius, center_colour, 3);
    cv::putText(image, std::to_string(marker.id), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, center_colour, 3);
    for(int i = 0; i < 4; i++){

        if(i == 3){
            cv::line(image, 
                    cv::Point(marker.p[3][0], marker.p[3][1]), 
                    cv::Point(marker.p[0][0], marker.p[0][1]),
                    line_colour, 2);
        }else{
            cv::line(image, 
                cv::Point(marker.p[i][0], marker.p[i][1]), 
                cv::Point(marker.p[i+1][0], marker.p[i+1][1]),
                line_colour, 2);
        }
        // {
        //     std::lock_guard<std::mutex> lock_printf(g_write_mutex);
        //     printf("Apriltag: corner%d: %.2f %.2f to %.2f %.2f\n",i , marker.p[i][0], marker.p[i][1], marker.p[i+1][0], marker.p[i+1][1]);
        // }
    }


}

// TODO: Add option for multiple families

void run_aprilTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    apriltag_settings_t &apriltag_settings,
    settings_t &settings){

    // creating fp for the csv output file
    
    char filename[] = "../tracking_data/apriltag/apriltag_tracking.csv";
    char picture_folder[] = "../tracking_data/apriltag/pictures/";

    int image_number = 0;

    int array_length = sizeof(id_array) / sizeof(id_array[0]);

    
    FILE *fp = fopen(filename, "w");
    if(fp == NULL){
        printf("Error: Could not open file %s\n, skipping tracking functionality", filename);
        return;
    }
    else
    {
        std::unique_lock<std::mutex> lck(g_mutex);
        lck.unlock();
        fprintf(fp, "distance,angle,id,valid\n");

        // printf("AprilTag: Setting up AprilTag detector\n");
        // apriltag_family_t *tf = tagStandard41h12_create();

        apriltag_family_t *tf = NULL;

        if (!strcmp(famname, "tag36h11")) {
            tf = tag36h11_create();
        } else if (!strcmp(famname, "tag25h9")) {
            tf = tag25h9_create();
        } else if (!strcmp(famname, "tag16h5")) {
            tf = tag16h5_create();
        } else if (!strcmp(famname, "tagCircle21h7")) {
            tf = tagCircle21h7_create();
        } else if (!strcmp(famname, "tagCircle49h12")) {
            tf = tagCircle49h12_create();
        } else if (!strcmp(famname, "tagStandard41h12")) {
            tf = tagStandard41h12_create();
        } else if (!strcmp(famname, "tagStandard52h13")) {
            tf = tagStandard52h13_create();
        } else if (!strcmp(famname, "tagCustom48h12")) {
            tf = tagCustom48h12_create();
        } else {
            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
            }
            fclose(fp);
            return;
        }



        apriltag_detector_t *td = apriltag_detector_create();
        int bit_err = 1; // Detect tags with up to this many bit errors
        apriltag_detector_add_family_bits(td, tf, bit_err);

        td->quad_decimate = 2.0; // Decimate input image by this factor
        td->quad_sigma = 0.0; // Apply low-pass blur to input; negative sharpens
        td->nthreads = 1; // Use this many CPU threads
        td->debug = 0; // Enable debugging output (slow)
        td->refine_edges = 1; // Spend more time trying to align edges of tags

        cv::Mat gray_frame;
        cv::Mat frame;

        zarray_t *detections = NULL;
        apriltag_detection_t *det = NULL;

        image_u8_t apriltag_img = {
                        .width = 0,
                        .height = 0,
                        .stride = 0,
                        .buf = 0
                    };

        int no_marker_count_angler = 0;
        int no_marker_count_slider = 0;

        std::vector<int> found_ids;

        // printf("AprilTag: AprilTag detector setup complete\n");

        for(int dist = 0; dist < (settings.max_distance + settings.distance_step); dist += settings.distance_step){

            if(dist != 0){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("AprilTag: Moving slider to %d\n", dist);
                }

                publish_set_slider_relative(publisher, settings.distance_step); // increase slider distance
                lck.lock();
                while(!g_ready) g_slider_cv.wait(lck); // Wait for the angler node to reach position
                g_ready = false;
                lck.unlock();
            }

            for(int angle = 0; angle < (settings.max_angle + settings.angle_step); angle += settings.angle_step){

                if(angle != 0){

                    {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("AprilTag: Moving angle to %d\n", angle);
                    }
                    publish_set_angle_relative(publisher, settings.angle_step); // increase viewing angle

                    lck.lock();
                    while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
                    g_ready = false;
                    lck.unlock();
                }
                

                for(int frame_count = 0; frame_count < settings.frames_to_capture; frame_count++){
                    capture >> frame;
                    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

                    image_u8_t apriltag_img = {
                        .width = gray_frame.cols,
                        .height = gray_frame.rows,
                        .stride = gray_frame.cols,
                        .buf = gray_frame.data
                    };

                    detections = apriltag_detector_detect(td, &apriltag_img);

                    
                    cv::Mat outputImage = frame.clone();

                    if(zarray_size(detections) != 0){
                        for (int i = 0; i < zarray_size(detections); i++){
                            zarray_get(detections, i, &det);

                            if(settings.save_picture_with_marker){
                                drawAprilTagMarkers(*det, outputImage);
                            }
                            
                            fprintf(fp, "%d,%d,%d,1\n", dist, angle, det->id);

                            found_ids.push_back(det->id);

                        }
                        if(zarray_size(detections) < array_length){
                            for(int i = 0; i < array_length; i++){ // check which of the ids were not found
                                if(std::find(found_ids.begin(), found_ids.end(), id_array[i]) == found_ids.end()){
                                    fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                                }

                            }
                        }

                        if(settings.save_picture_with_marker){
                            if(apriltag_save_picture(image_number, picture_folder, outputImage) == 1){
                                break;
                            }
                        }
                        
                    }
                    else{
                        //When we dont detect anymore markers we can stop the loop and move to next postion.
                        //If it happens 3 times exit the thread and stop the program.
                        no_marker_count_angler++;
                        for(int i = 0; i < array_length; i++){
                            fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                        }
                    }
                    found_ids.clear();
                    image_number++;


                }
                if(((no_marker_count_angler % settings.frames_to_capture) == 0) && (settings.enable_no_marker_skip)){
                    {
                        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                        printf("AprilTag: No markers detected, exiting angle loop\n");
                    }
                    no_marker_count_slider++;
                    break;
                }

            }


            if((no_marker_count_slider == settings.skipping_counter) && (settings.enable_no_marker_skip))  {
                publish_set_angle(publisher, 0); // reset viewing angle back to default
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("AprilTag: %d times no markers detected, exiting slider loop\n", settings.skipping_counter);
                }
                break; // no reason to continue if we dont detect any markers   
            }

            publish_set_angle(publisher, 0); // reset viewing angle back to default

            lck.lock();
            while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
            g_ready = false;
            lck.unlock();

        }
        fclose(fp);

        //  publish_set_angle_relative(publisher, -max_viewing_angle); // reset viewing angle back to default
        // while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
        // g_ready = false;

        publish_set_slider(publisher, 0); // reset slider back to default
        lck.lock();
        while(!g_ready) g_slider_cv.wait(lck); // Wait for the angler node to reach position
        g_ready = false;
        lck.unlock();

        //Clean up
        apriltag_detections_destroy(detections);
        apriltag_detector_destroy(td);
        tagStandard41h12_destroy(tf);
        // g_cv.notify_one();

        return;

    }
}



