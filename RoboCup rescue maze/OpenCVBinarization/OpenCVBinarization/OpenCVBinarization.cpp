// OpenCVBinarization.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    unique_ptr<tesseract::TessBaseAPI> myOCR = make_unique<tesseract::TessBaseAPI>();
    myOCR->Init(NULL, "eng");
    
    VideoCapture camera(0);
    if (!camera.isOpened()) return 1;

    Mat3b frame;
    Pix* pix = nullptr;
    bool written = false;

    while (true) {
        camera >> frame;

        double alpha = 2.5;
        int beta = -20;
        for (int y = 0; y < frame.rows; y++) {
            for (int x = 0; x < frame.cols; x++) {
                for (int c = 0; c < frame.channels(); c++) {
                    frame.at<Vec3b>(y, x)[c] =
                        saturate_cast<uchar>(alpha * frame.at<Vec3b>(y, x)[c] + beta);
                }
            }
        }

        imshow("Video", frame);

        if (pix == nullptr) {
            pix = pixCreate(frame.size().width, frame.size().height, 8);
        }

        //cout << frame.cols << " " << frame.rows << " " << frame. << endl;
        for (int y = 0; y < frame.rows; y++) {
            for (int x = 0; x < frame.cols; x++) {
                uchar b = frame.data[frame.channels() * (frame.cols * y + x)];
                uchar g = frame.data[frame.channels() * (frame.cols * y + x) + 1];
                uchar r = frame.data[frame.channels() * (frame.cols * y + x) + 2];
                pixSetPixel(pix, x, y, (l_uint32)(r << 16 | g << 8 | b << 0));
            }
        }

        if (!written) {
            pixWrite("frame.png", pix, IFF_PNG);
            written = true;
        }

        myOCR->SetImage(pix);
        myOCR->SetSourceResolution(70);

        char *result = myOCR->GetUTF8Text();
        cout << result[0] << "\r";

        if (waitKey(10) == 27) {
            break;
        }
    }

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
