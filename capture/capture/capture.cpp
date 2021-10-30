// capture.cpp: Definiert den Einstiegspunkt für die Anwendung.
//
// -- requiered for https
//#define CPPHTTPLIB_OPENSSL_SUPPORT

#include "capture.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread> 

#include "../httplib.h"

using namespace cv;

//used to sync main thread and http-server-thread (one read / one write, so i neglected sync so first)
static bool FoundEmergencyVehicle = false;
const int MAXPIXEL = 100000;

//----------- used by Algo Color detection
// HSV range to detect RED color
int minH = 23, maxH = 199;
int minS = 226, maxS = 255;
int minV = 0, maxV = 220;


void blobDetection(Mat3b& inputImage, Mat3b& markersImage) {
    
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 40;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(inputImage, keypoints);
    std::cout <<"Keypoints:" << keypoints.size() << "\n";

    drawKeypoints(inputImage, keypoints, markersImage, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("my markers", markersImage);
}

void server()
{
    // HTTP only
    httplib::Server svr;

    svr.Get("/register", [](const httplib::Request&, httplib::Response& res) {
        while (!FoundEmergencyVehicle) {}; // [let the client wait until send RSP]
                                          // timeout should not be a problem because the vehicle will be in a zone just for rather short time
                                          // maybe this should just be used for an limited amount of clients
        res.set_content("<html><body text=\"green\" bgcolor=\"orange\"><h1>Emergency Vehicle!</h1></body></html>", "text/html");
        });

    svr.listen("0.0.0.0", 8800);
}

void triggerAlarm() {
    std::cout << "triggerAlarm!";
    FoundEmergencyVehicle = true;
}

void resetAlarm() {
    std::cout << "resetAlarm!";
    FoundEmergencyVehicle = false;
}

void validateMask(Mat1b& resultMask) {
    double highPixels = cv::sum(resultMask)[0];
    std::cout << "sum high_pixels:" << highPixels << "\n";

    if (highPixels > MAXPIXEL) {
        triggerAlarm();
        //all registered clients get triggered at once. reset can be triggered according to hysteresis
        resetAlarm();
    }

}

void AlgodetectRedVehicle(Mat& frame) {
    Mat3b bgr_inv = ~frame;
    Mat3b hsv_inv, markers;

    cvtColor(bgr_inv, hsv_inv, COLOR_BGR2HSV);

    Mat1b mask, masklow, maskup;
    inRange(hsv_inv, (minH, minS, minV), Scalar(maxH, maxS, maxV), masklow);


    auto full_mask = masklow;// + maskup; //we still might need to use multiple ranges in the HSV space for red
    cv::bitwise_not(full_mask, full_mask);

    imshow("hsv_inv", hsv_inv);
    imshow("maskW", full_mask);

    validateMask(full_mask);
}

int main(int argc, char** argv)
{
    Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    auto const MASK_WINDOW = "Mask Settings";
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API

    cv::namedWindow(MASK_WINDOW);
    
    // Create trackbars of mask settings window
    cv::createTrackbar("Min Hue", MASK_WINDOW, &minH, 255);
    cv::createTrackbar("Max Hue", MASK_WINDOW, &maxH, 255);
    cv::createTrackbar("Min Sat", MASK_WINDOW, &minS, 255);
    cv::createTrackbar("Max Sat", MASK_WINDOW, &maxS, 255);
    cv::createTrackbar("Min Val", MASK_WINDOW, &minV, 255);
    cv::createTrackbar("Max Val", MASK_WINDOW, &maxV, 255);

    // open selected camera using selected API
    cap.open(deviceID, apiID);
    
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl
        << "Press any key to terminate" << std::endl;
    
    std::thread myHttpServer(server);
    
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;

        AlgodetectRedVehicle(frame);

       

    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}