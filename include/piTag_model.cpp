#include "piTag_model.hpp"

using namespace ipa_Fiducials;

#include <thread>
#include <chrono>
#include <iostream>
// std::mutex g_mutex;

const char *model_filename = "../models/piTag/algorithms/fpiTagIni_0.xml"; 


int pitag_save_picture(int image_number, const char *folder, cv::Mat Image){
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

void drawPiTagMarkers(std::vector<t_points> vec_points, cv::Mat& image)
{
    const int radius = 1;
    const int fontSize = 3;

    const cv::Scalar center_colour = cv::Scalar(0, 255, 0, 255);

    const cv::Scalar line_colour = cv::Scalar(180, 255, 125, 255);

    // // for(int i = 0; i < vec_points.size(); i++){
    // //     std::cout << "Detected Tag " << vec_points[i].id << ": " << vec_points[i].marker_points << std::endl;
    // // }


    for(int tag_idx = 0; tag_idx < vec_points.size(); tag_idx++){
        float x_sum = 0;
        float y_sum = 0;


        for(int point_idx = 0; point_idx < vec_points[tag_idx].image_points.size(); point_idx++){

            x_sum += vec_points[tag_idx].image_points.at(point_idx).x;

            y_sum += vec_points[tag_idx].image_points.at(point_idx).y;

            if(point_idx == (vec_points[tag_idx].image_points.size() - 1)){
                // vec_points[tag_idx].image_points.
                cv::line(image, 
                        cv::Point(vec_points[tag_idx].image_points.at(point_idx).x, vec_points[tag_idx].image_points.at(point_idx).y), 
                        cv::Point(vec_points[tag_idx].image_points.at(0).x, vec_points[tag_idx].image_points.at(0).y), 
                        line_colour, 2);
            }else{
                cv::line(image, 
                    cv::Point(vec_points[tag_idx].image_points.at(point_idx).x, vec_points[tag_idx].image_points.at(point_idx).y), 
                        cv::Point(vec_points[tag_idx].image_points.at(point_idx + 1).x, vec_points[tag_idx].image_points.at(point_idx + 1).y), 
                    line_colour, 2);
            }

            x_sum = x_sum / vec_points[tag_idx].image_points.size();

            y_sum = y_sum / vec_points[tag_idx].image_points.size();


            // const cv::Point center = cv::Point(x_sum, y_sum);

            // cv::circle(image, center, radius, center_colour, 3);

            // cv::putText(image, std::to_string(vec_points[tag_idx].id), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, center_colour, 3);

        }  

    }

}

void run_piTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    pitag_settings_t &pitag_settings,
    settings_t &settings){

    // creating fp for the csv output file
    
    char filename[] = "../tracking_data/pitag/pitag_tracking.csv";
    char picture_folder[] = "../tracking_data/pitag/pictures/";

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

        // printf("piTag: Setting up detector\n");
        
        cv::Mat frame;

        std::shared_ptr<AbstractFiducialModel> tag_detector;
        cv::Mat camera_matrix;
        std::vector<t_pose> tags_vec;
        
        tag_detector = std::shared_ptr<FiducialModelPi>(new FiducialModelPi());

        int array_length = sizeof(id_array) / sizeof(id_array[0]);
        
        camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
        
        if (tag_detector->Init(camera_matrix, model_filename, false) & RET_FAILED)
        {
            std::lock_guard<std::mutex> lock_printf(g_write_mutex);
            std::cout << "Initializing fiducial detector with camera matrix [FAILED]" << std::endl;
            fclose(fp);
            
        }

        std::vector<int> found_ids;
        std::vector<t_points> vec_points;

        int no_marker_count_angler = 0;
        int no_marker_count_slider = 0;


        // printf("piTag: Detector setup complete\n");

        for(int dist = 0; dist < (settings.max_distance + settings.distance_step); dist += settings.distance_step){
            if(dist != 0){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("PiTag: Moving slider to %d\n", dist);
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
                    printf("PiTag: Moving angle to %d\n", angle);
                    }
                    publish_set_angle_relative(publisher, settings.angle_step); // increase viewing angle

                    lck.lock();
                    while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
                    g_ready = false;
                    lck.unlock();
                }

                for(int frame_count = 0; frame_count < settings.frames_to_capture; frame_count++){
                    capture >> frame;
                    

                    if (tag_detector->GetPoints(frame, vec_points) == RET_OK){

                        if(settings.save_picture_with_marker){
                            cv::Mat outputImage = frame.clone();
                            drawPiTagMarkers(vec_points, outputImage);
                            if(pitag_save_picture(image_number, picture_folder, outputImage) == 1){
                                break;
                            }
                        }

                        for (unsigned int i=0; i<vec_points.size(); i++)
                        {
                            // std::cout << "Detected Tag " << vec_points[i].id << ": " << vec_points[i].image_points << std::endl;

                            found_ids.push_back(vec_points[i].id);

                            fprintf(fp, "%d,%d,%d,1\n", dist, angle, vec_points[i].id);

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
                    vec_points.clear();
                    found_ids.clear();
                    image_number++;
                        
                }

                if(((no_marker_count_angler % settings.frames_to_capture) == 0) && (settings.enable_no_marker_skip)){
                    // printf("piTag: No markers detected, exiting thread\n");
                    no_marker_count_slider++;
                    break;
                }

            }
            if((no_marker_count_slider == settings.skipping_counter) && (settings.enable_no_marker_skip)){
                publish_set_angle(publisher, 0);
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
        // printf("piTag: Done running the model\n");

        // g_cv.notify_one();
    }


}



