#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>


int main() {
    cv::VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat frame;
    cap >> frame;
    cv::imshow("Bin", frame);

    int hMin = 72;
    int hMax = 92;

    int sMin = 110;
    int sMax = 255;

    int vMin = 48;
    int vMax = 255;

    int cannyThreshold = 0;
    int eccentricityThreshold = 10;
    int areaThreshold = 10;

    int chMax = 255;

    cv::createTrackbar("H min", "Bin", &hMin, chMax);
    cv::createTrackbar("H max", "Bin", &hMax, chMax);

    cv::createTrackbar("S min", "Bin", &sMin, chMax);
    cv::createTrackbar("S max", "Bin", &sMax, chMax);

    cv::createTrackbar("V min", "Bin", &vMin, chMax);
    cv::createTrackbar("V max", "Bin", &vMax, chMax);

    cv::createTrackbar("Canny", "Bin", &cannyThreshold, 100);
    cv::createTrackbar("Eccentricity", "Bin", &eccentricityThreshold, 100);
    cv::createTrackbar("Area", "Bin", &areaThreshold, 100);

    cv::Mat image, detectedEdges;

    cv::RNG rng(12345);

    while (true) {
        cap >> image;
        cv::imshow("Image", image);
        cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
        cv::Scalar lower(hMin, sMin, vMin);
        cv::Scalar upper(hMax, sMax, vMax);
        cv::inRange(image, lower, upper, image);

        cv::OutputArray edges();
        cv::blur(image, detectedEdges, cv::Size(5, 5));
        cv::Canny(detectedEdges, detectedEdges, cannyThreshold, cannyThreshold * 3.0);

        cv::Mat result(cv::Mat::zeros(image.size(), CV_8UC3));

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(detectedEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        auto areaThresholdInterpolated = (int)((double)areaThreshold / 100.0) * image.total();

        for (auto contour : contours) {
            if (contour.size() < 6) continue;

            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

            auto ellipse = cv::fitEllipse(contour);
            auto width = ellipse.boundingRect().width;
            auto height = ellipse.boundingRect().height;

            auto majorAxisLength = std::max(width, height);
            auto minorAxisLength = std::min(width, height);

            auto eccentricity = std::sqrt(1 - std::pow(minorAxisLength / majorAxisLength, 2));
            if (eccentricity > (double)eccentricityThreshold / 100.0) continue;
            if (cv::contourArea(contour) < areaThresholdInterpolated) continue;

            cv::ellipse(result, ellipse, color, 2);
        }

        cv::imshow("Bin", result);
        cv::imshow("Canny", detectedEdges);

        if (cv::waitKey(10) == 27) { // если нажата ESC - выходим
            break;
        }
    }

    /*
        edges = cv2.Canny(img, lower, upper)
        cv2.imshow('Edges', edges)
        _, contours, _ = cv2.findContours(edged, cv2.RETR_TREE, 1)
        rep = cv2.drawContours(img1, contours, -1, (0, 255, 0), 3)
        cv2.imshow(Contours',rep)

    //for (auto countour : cnt)
    cnt = contours
    for i in range(0, len(cnt)):
        ellipse = cv2.fitEllipse(cnt[i])
        (center,axes,orientation) =ellipse
        majoraxis_length = max(axes)
        minoraxis_length = min(axes)
        eccentricity=(np.sqrt(1-(minoraxis_length/majoraxis_length)**2))
        cv2.ellipse(img2,ellipse,(0,0,255),2)

        */
    cv::destroyAllWindows();
}