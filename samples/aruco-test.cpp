//Needs to use opencv4 in order to use arUco

#include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main(int argc, char** argv) {
    
        if (argc != 2)
    {
        puts("Please provide the path to the imagefile");
        return 1;
    }

    std::string image_filename = argv[1];

    
    cv::Mat imageBGR = cv::imread(image_filename, cv::IMREAD_COLOR);
    cv::Mat imageGray;
    cv::cvtColor(imageBGR, imageGray, cv::COLOR_BGR2GRAY);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    detector.detectMarkers(imageGray, markerCorners, markerIds, rejectedCandidates);
    cv::Mat outputImage = imageBGR.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    if(markerIds.size() != 0){
        for(int i = 0; i < markerIds.size(); i++){
            printf("Found ID: %d\n", markerIds.at(i));
        }
    }

    cv::namedWindow("img", cv::WINDOW_KEEPRATIO);
    cv::imshow("img", outputImage);
    cv::resizeWindow("img", 1500, 1500);

    // cout << "Frame width: " << capture.get(CAP_PROP_FRAME_WIDTH) << endl;
    // cout << "     height: " << capture.get(CAP_PROP_FRAME_HEIGHT) << endl;
    // cout << "Capturing FPS: " << capture.get(CAP_PROP_FPS) << endl;

    // cout << endl << "Press 'ESC' to quit, 'space' to toggle frame processing" << endl;
    // cout << endl << "Start grabbing..." << endl;

    // for(;;){
    //     capture >> frame;
    //     detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

    //     cv::Mat outputImage = frame.clone();
    //     cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    //     if(markerIds.size() != 0){
    //         for(int i = 0; i < markerIds.size(); i++){
    //             printf("Found ID: %d\n", markerIds.at(i));
    //         }
    //     }
    //     // printf("Found ID: %d\n", markerIds.at(0));

    //     cv::imshow("img", outputImage);

    cv::waitKey(0);
    //     if (key == 27/*ESC*/)
    //         break;
    // }

    return 0;
}