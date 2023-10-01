#include "stag_model.hpp"

#include <thread>
#include <chrono>
#include <iostream>
// std::mutex g_mutex;


int stag_save_picture(Stag stag,int image_number, const char *folder){
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
            stag.drawMarkers(path);
            }
        }

    }
}



void drawSTagMarkers(Stag stag, cv::Mat& image)
{

    const int radius = 1;
    const int fontSize = 3;

    const cv::Scalar center_colour = cv::Scalar(0, 255, 0, 255);

    const cv::Scalar line_colour = cv::Scalar(180, 255, 125, 255);

    for(int tag_idx = 0; tag_idx < stag.markers.size(); tag_idx++){

        const cv::Point center = cv::Point(stag.markers[tag_idx].center.x, stag.markers[tag_idx].center.y);

        cv::circle(image, center, radius, center_colour, 3);

        cv::putText(image, std::to_string(stag.markers[tag_idx].id), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, center_colour, 3);

        for (int corner_idx = 0; corner_idx < stag.markers[tag_idx].corners.size(); corner_idx++){
            if(corner_idx == stag.markers[tag_idx].corners.size()){
                cv::line(image, 
                        cv::Point(stag.markers[tag_idx].corners[(stag.markers[tag_idx].corners.size() - 1)].x, stag.markers[(stag.markers[tag_idx].corners.size() - 1)].corners[corner_idx].y), 
                        cv::Point(stag.markers[tag_idx].corners[0].x, stag.markers[tag_idx].corners[0].y),
                        line_colour, 2);
            }else{
                cv::line(image, 
                    cv::Point(stag.markers[tag_idx].corners[corner_idx].x, stag.markers[tag_idx].corners[corner_idx].y),
                    cv::Point(stag.markers[tag_idx].corners[corner_idx + 1].x, stag.markers[corner_idx + 1].corners[corner_idx].y), 
                    line_colour, 2);
            }

            {
                std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                printf("STag: corner%d: %.2f %.2f to %.2f %.2f\n",corner_idx , stag.markers[tag_idx].corners[corner_idx].x, stag.markers[tag_idx].corners[corner_idx].y, stag.markers[tag_idx].corners[corner_idx + 1].x, stag.markers[tag_idx].corners[corner_idx + 1].y);
            }
        }
    }


}


void run_STag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    stag_settings_t &stag_settings,
    settings_t &settings){

    // creating fp for the csv output file

    char filename[] = "../tracking_data/stag/stag_tracking.csv";
    char picture_folder[] = "../tracking_data/stag/pictures/";

    int image_number = 0;
    
    FILE *fp = fopen(filename, "w");
    if(fp == NULL){
        printf("Error: Could not open file %s\n, skipping tracking functionality", filename);
    }
    else
    {
        std::unique_lock<std::mutex> lck(g_mutex);
        lck.unlock();

        fprintf(fp, "distance,angle,id,valid\n");

        {
			std::lock_guard<std::mutex> lock_printf(g_write_mutex);
			
            printf("STag: Setting up detector\n");
		}

        int array_length = sizeof(id_array) / sizeof(id_array[0]);
        
        cv::Mat gray_frame;
        cv::Mat frame;

        Stag stag(stag_settings.stag_libraryHD, stag_settings.stag_errorCorrection, false);

        std::vector<int> found_ids;

        int no_marker_count_angler = 0;
        int no_marker_count_slider = 0;

        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            printf("STag: Detector setup complete\n");

        }


        for(int dist = 0; dist < (settings.max_distance + settings.distance_step); dist += settings.distance_step){
            if(dist != 0){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("STag: Moving slider to %d\n", dist);
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
                    printf("STag: Moving angle to %d\n", angle);
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
                    int stag_detected_markers = stag.detectMarkers(gray_frame);

                    if(stag_detected_markers != 0){

                        if(settings.save_picture_with_marker){
                            cv::Mat outputImage = frame.clone();
                            // drawSTagMarkers(stag, outputImage);
                            if(stag_save_picture(stag ,image_number, picture_folder) == 1){
                                break;
                            }
                        }


                        for(int i = 0; i < stag.markers.size(); i++){
                            fprintf(fp, "%d,%d,%d,1\n", dist, angle, stag.markers[i].id);

                            found_ids.push_back(stag.markers[i].id);
                        }
                        if(found_ids.size() < array_length){
                            for(int i = 0; i < array_length; i++){ // check which of the ids were not found
                                if(std::find(found_ids.begin(), found_ids.end(), id_array[i]) == found_ids.end()){
                                    fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                                }

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
                    stag.markers.clear();
                    found_ids.clear();
                    image_number++;

                }

                if(((no_marker_count_angler % settings.frames_to_capture) == 0) && (settings.enable_no_marker_skip)){
                    // printf("CCTag: No markers detected, exiting thread\n");
                    no_marker_count_slider++;
                    break;
                }

            }
            if((no_marker_count_slider == settings.skipping_counter) && (settings.enable_no_marker_skip)) {
                publish_set_angle(publisher, 0);
                break; // no reason to continue if we dont detect any markers'
            }

            publish_set_angle(publisher, 0); // reset viewing angle back to default
            lck.lock();
            while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
            g_ready = false;
            lck.unlock();

        }
        fclose(fp);


        publish_set_slider(publisher, 0); // reset slider back to default
        lck.lock();
        while(!g_ready) g_slider_cv.wait(lck); // Wait for the angler node to reach position
        g_ready = false;
        lck.unlock();


    }
}



