#include "ccTag_model.hpp"

#include <thread>
#include <chrono>
#include <iostream>

/*
    NOTE: This is made to run without CUDA support. If this model is gonna be used further a rewriting of the code is needed. 
    Use the ccTag-test.cpp as a reference.
*/
const char *TAG = "CCTag";

int cctag_save_picture(int image_number, const char *folder, cv::Mat Image){
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


bool ccTagisInteger(std::string& s) { return (s.size() == 1 && std::isdigit(s[0])); }

bool ccTagdrawMarkers(const boost::ptr_list<CCTag>& markers, cv::Mat& image, bool showUnreliable)
{
    bool reliable_flag = false;
    for(const cctag::CCTag& marker : markers)
    {
        const int radius = 10;
        const int fontSize = 3;
        if(marker.getStatus() == status::id_reliable)
        {
            reliable_flag = true;

            const cv::Point center = cv::Point(marker.x(), marker.y());
            const cv::Scalar color = cv::Scalar(0, 255, 0, 255);
            const auto rescaledOuterEllipse = marker.rescaledOuterEllipse();

            cv::circle(image, center, radius, color, 3);
            cv::putText(image, std::to_string(marker.id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
            cv::ellipse(image,
                        center,
                        cv::Size(rescaledOuterEllipse.a(), rescaledOuterEllipse.b()),
                        rescaledOuterEllipse.angle() * 180 / boost::math::constants::pi<double>(),
                        0,
                        360,
                        color,
                        3);
        }
        // else if(showUnreliable)
        // {
        //     const cv::Scalar color = cv::Scalar(0, 0, 255, 255);
        //     cv::circle(image, center, radius, color, 2);
        //     cv::putText(image, std::to_string(marker.id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
        // }
    }

    return reliable_flag;
}

void ccTagdetection(std::size_t frameId,
               int pipeId,
               const cv::Mat& src,
               const cctag::Parameters& params,
               const cctag::CCTagMarkersBank& bank,
               boost::ptr_list<CCTag>& markers,
               std::ostream& outStream,
               std::string debugFileName)
{
    if(debugFileName.empty())
    {
        debugFileName = "00000";
    }

    // Process markers detection
    boost::timer::cpu_timer t;

    CCTagVisualDebug::instance().initBackgroundImage(src);
    CCTagVisualDebug::instance().setImageFileName(debugFileName);
    CCTagFileDebug::instance().setPath(CCTagVisualDebug::instance().getPath());

    static cctag::logtime::Mgmt* durations = nullptr;

    // Call the main CCTag detection function
    cctagDetection(markers, pipeId, frameId, src, params, bank, true, durations);

    if(durations)
    {
        durations->print(std::cerr);
    }

    CCTagFileDebug::instance().outPutAllSessions();
    CCTagFileDebug::instance().clearSessions();
    CCTagVisualDebug::instance().outPutAllSessions();
    CCTagVisualDebug::instance().clearSessions();

    std::cout << "Total time: " << t.format() << std::endl;
    CCTAG_COUT_NOENDL("Id : ");

    std::size_t counter = 0;
    std::size_t nMarkers = 0;
    outStream << "#frame " << frameId << '\n';
    outStream << "Detected " << markers.size() << " candidates" << '\n';

    for(const cctag::CCTag& marker : markers)
    {
        outStream << marker.x() << " " << marker.y() << " " << marker.id() << " " << marker.getStatus() << '\n';
        ++counter;
        if(marker.getStatus() == status::id_reliable)
            ++nMarkers;
    }

    counter = 0;
    for(const cctag::CCTag& marker : markers)
    {
        if(counter == 0)
        {
            CCTAG_COUT_NOENDL(marker.id() + 1);
        }
        else
        {
            CCTAG_COUT_NOENDL(", " << marker.id() + 1);
        }
        ++counter;
    }

    std::cout << std::endl << nMarkers << " markers detected and identified" << std::endl;
}

void run_ccTag(cv::VideoCapture& capture,
    mosquitto *publisher,
    int id_array[],
    cctag_settings_t &cctag_settings,
    settings_t &settings)
{
    //Setting up the ccTag model
    // printf("CCTag: Setting up model");
    const std::size_t nCrowns = cctag_settings.ccTag_nRings;
    cctag::Parameters params(nCrowns);

    CCTagMarkersBank bank(params._nCrowns);

    cv::Mat frame;

    cv::Mat imgGray;

    std::size_t frameId = 0;
    boost::ptr_list<CCTag> markers;


    const int pipeId = 0;
    static cctag::logtime::Mgmt* durations = nullptr;


    char filename[] = "../tracking_data/cctag/cctag_tracking.csv";
    char picture_folder[] = "../tracking_data/cctag/pictures/";

    int image_number = 0;

    int array_length = sizeof(id_array) / sizeof(id_array[0]);

    std::vector<int> found_ids;

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

        for(int dist = 0; dist < (settings.max_distance + settings.distance_step); dist += settings.distance_step){
            if(dist != 0){
                {
                    std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                    printf("CCTag: Moving slider to %d\n", dist);
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
                    printf("CCTag: Moving angle to %d\n", angle);
                    }
                    publish_set_angle_relative(publisher, settings.angle_step); // increase viewing angle

                    lck.lock();
                    while(!g_ready) g_angler_cv.wait(lck); // Wait for the angler node to reach position
                    g_ready = false;
                    lck.unlock();
                }

                for(int frame_count = 0; frame_count < settings.frames_to_capture; frame_count++){
                    capture >> frame;
                     if(frame.channels() == 3 || frame.channels() == 4)
                        cv::cvtColor(frame, imgGray, cv::COLOR_BGR2GRAY);
                    else
                        frame.copyTo(imgGray);
                    
                    cctagDetection(markers, pipeId, frameId, imgGray, params, bank, false, durations);


                    if(markers.size() != 0){
                        
                        if(settings.save_picture_with_marker){
                            cv::Mat outputImage = frame.clone();
                            if(ccTagdrawMarkers(markers, outputImage, false)){
                                if(cctag_save_picture(image_number, picture_folder, outputImage) == 1){
                                    break;
                                }
                            }
                            // cv::imshow("cctag", outputImage);
                            // cv::waitKey(0);
                        }

                        for(const cctag::CCTag& marker : markers)
                        {
                            if(marker.getStatus() == status::id_reliable){
                                fprintf(fp, "%d,%d,%d,1\n", dist, angle, marker.id());
                                found_ids.push_back(marker.id());
                                // {
                                //     std::lock_guard<std::mutex> lock_printf(g_write_mutex);
                                //     printf("%s: id found: %d\n", TAG, marker.id());
                                // }
                            }
                        }
                        if(found_ids.size() < array_length){
                            for(int i = 0; i < array_length; i++){ // check which of the ids were not found
                                if(std::find(found_ids.begin(), found_ids.end(), id_array[i]) == found_ids.end()){
                                    fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                                    // printf("%s: ids missing: %d\n", TAG, id_array[i]);
                                }

                            }
                        }
                    }
                    else{
                        //When we dont detect anymore markers we can stop the loop and move to next postion.
                        no_marker_count_angler++;
                        for(int i = 0; i < array_length; i++){
                            fprintf(fp, "%d,%d,%d,0\n", dist, angle, id_array[i]);
                        }
                    }
                    found_ids.clear();
                    markers.clear();
                    image_number++;

                }
                if(((no_marker_count_angler % settings.frames_to_capture) == 0) && (settings.enable_no_marker_skip)){
                    // printf("CCTag: No markers detected, exiting thread\n");
                    no_marker_count_slider++;
                    break;
                }
            }
            if((no_marker_count_slider == settings.skipping_counter) && (settings.enable_no_marker_skip)){
                
                publish_set_angle(publisher, 0);
                break;
            } // no reason to continue if we dont detect any markers

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
        // printf("CCtag: Done running the model\n");
        // g_cv.notify_one();

    }



}