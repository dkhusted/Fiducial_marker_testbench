#include "arUco_model.hpp"

#include <thread>
#include <chrono>
#include <iostream>
//std::mutex arUco_mutex;

int aruco_save_picture(int image_number, const char *folder, cv::Mat Image){
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

void run_arUcoTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    aruco_settings_t &aruco_settings,
    settings_t &settings) {
    
    //Setting up the arUco model
    cv::Mat frame;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    
    // printf("ArUco: Setting up arUco model\n");
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); 
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    // printf("ArUco: Done setting up arUco model\n");

    // creating fp for the csv output file
    
    char filename[] = "../tracking_data/arUco/arUcoTag_tracking.csv";
    char picture_folder[] = "../tracking_data/arUco/pictures/";

    int image_number = 0;
    int no_marker_count_angler = 0;
    int no_marker_count_slider = 0;
    
    FILE *fp = fopen(filename, "w");
    if(fp == NULL){
        printf("Error: Could not open file %s\n, skipping tracking functionality", filename);
    }
    else
    {
        std::unique_lock<std::mutex> lck(g_mutex);
        lck.unlock();
        fprintf(fp, "distance,angle,id,valid\n");

        int array_length = sizeof(id_array) / sizeof(id_array[0]);

        for(int dist = 0; dist < (settings.max_distance + settings.distance_step); dist += settings.distance_step){
            
            if(dist != 0){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("ArUco: Moving slider to %d\n", dist);
                }

                publish_set_slider_relative(publisher, settings.distance_step); // increase slider distance
                lck.lock();
                while(!g_ready) g_slider_cv.wait(lck); // Wait for the angler node to reach position
                g_ready = false;
                lck.unlock();
            }

            for(int angle = 0; angle < (settings.max_angle +settings.angle_step ); angle += settings.angle_step){

                if(angle != 0){

                    {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("ArUco: Moving angle to %d\n", angle);
                    }
                    publish_set_angle_relative(publisher, settings.angle_step); // increase viewing angle

                    lck.lock();
                    while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
                    g_ready = false;
                    lck.unlock();
                }

                
                // capture 30 frames to get a good reading. Maybe needs to be increased?

                for(int frame_count = 0; frame_count < settings.frames_to_capture; frame_count++){
                    capture >> frame;


                    detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
                    // cv::Mat outputImage = frame.clone();
                    // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

                    if(markerIds.size() != 0){
                        
                        if(settings.save_picture_with_marker){
                            cv::Mat outputImage = frame.clone();
                            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);     
                            if(aruco_save_picture(image_number, picture_folder, outputImage) == 1){
                                break;
                            }
                        }
                        
                        for(int found_markers = 0; found_markers < markerIds.size(); found_markers++){
                            fprintf(fp, "%d,%d,%d,1\n", dist, angle, markerIds.at(found_markers));
                            {
                                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                                printf("Found ID: %d\n", markerIds.at(found_markers));
                            }
                        }
                        if(markerIds.size() < array_length){
                            for(int idsToCheck = 0; idsToCheck < array_length; idsToCheck++){
                                if((std::find(markerIds.begin(), markerIds.end(), id_array[idsToCheck])) == markerIds.end()){
                                    fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[idsToCheck]); // find markers which were not found
                                }
                            }

                        }
                    }
                    else{
                        no_marker_count_angler++;
                        for(int i = 0; i < array_length; i++){
                            fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                        }
                    }
                    markerIds.clear();
                    markerCorners.clear();
                    image_number++;

                }
                //When we dont detect anymore markers we can stop the loop and move to next postion.
                //If it happens 3 times exit the thread and stop the program.
                if(((no_marker_count_angler % settings.frames_to_capture) == 0) && (settings.enable_no_marker_skip)){
                    {
                        std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                        printf("arUcoTag: No markers detected, exiting angle loop\n");
                    }
                    no_marker_count_slider++;
                    break;
                }

            }

            if((no_marker_count_slider == settings.skipping_counter) && (settings.enable_no_marker_skip)){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("ArUco: %d times no markers detected, exiting slider loop\n", settings.skipping_counter);
                }
                publish_set_angle(publisher, 0); 
                
                break; // no reason to continue if we dont detect any markers  
            } 

            lck.lock();
            publish_set_angle(publisher, 0); // reset viewing angle back to default
            while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
            g_ready = false;
            lck.unlock();

            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("ArUco: Moving slider to %d\n", dist);
            }

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
        // printf("ArUco: Done running the model\n");
        // g_cv.notify_one();

    }

}
