// NAO.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cmath>
#include <thread>
#include <string>

#include <Windows.h>

#include <vector>
#include <alcommon/almodulecore.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

#include <alvision/alvisiondefinitions.h>

#include <alproxies/alproxies.h>

#include <alvision/alimage.h>

#include <opencv2/opencv.hpp>

const auto ip = "192.168.1.154";

//const auto ip = "169.254.198.169";

#define LOW_RANGE {160, 43, 46}
#define HIGH_RANGE {180, 255, 255}
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define ARG_WIDTH 1.064127244940942743//60.97/180*pi
#define ARG_HEIGHT 0.83147485565009861//47.64/180*pi

auto radius = 30;

void ReduceNoise(cv::Mat& img)
{
    //访问二值图像每个点的值   
    uchar* pp;
    //------------搜索二值图中的轮廓，并从轮廓树中删除面积小于某个阈值minarea的轮廓-------------//   
    auto color = cv::Scalar(255, 0, 0);//CV_RGB(128,0,0);   
    std::vector<std::vector<cv::Point> > contours;
    auto contourOutput = img.clone();
    cv::findContours(contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    //开始遍历轮廓树   
    cv::Rect rect;
    double minarea = HUGE_VALD;
    for (const auto& contour : contours)
    {
        auto tmparea = fabs(cv::contourArea(contour));
        rect = cv::boundingRect(contour);
        if (tmparea < minarea/*||tmparea>4900*/)
        {
            //当连通域的中心点为黑色时，而且面积较小则用白色进行填充   
            pp = (uchar*)(img.data + img.step * (rect.y + rect.height / 2) + rect.x + rect.width / 2);
            if (pp[0] == 255)
            {
                for (int y = rect.y; y < rect.y + rect.height; y++)
                {
                    for (int x = rect.x; x < rect.x + rect.width; x++)
                    {
                        pp = (uchar*)(img.data + img.step * y + x);

                        if (pp[0] == 255)
                        {
                            pp[0] = 0;
                        }
                    }
                }
            }
        }
    }
}

cv::Vec3f RedballDetect(cv::Mat& img)
{
    bool flag = false;
    cv::Mat gaus, hsv, th, th_rgb, blured, gray, binary, circles;
    cv::GaussianBlur(img, gaus, { 7,7 }, 1.5);
    cv::cvtColor(gaus, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(LOW_RANGE), cv::Scalar(HIGH_RANGE), th);
    cv::cvtColor(th, th_rgb, cv::COLOR_GRAY2BGR);

    cv::blur(th_rgb, blured, { 10,10 });
    cv::cvtColor(blured, gray, cv::COLOR_BGR2GRAY);
    auto ret = cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
    cv::HoughCircles(binary, circles, CV_HOUGH_GRADIENT, 1, 100, 15, 7, 8, 30);
    if (circles.empty())
    {
        return { -1,1,0 };
    }
    cv::imshow("binary", binary);
    //float min0 = std::fabsf(circles.at<cv::Vec3f>(0, 0)[2] - radius);
    //int index = 0;
    //for (int i = 0; i < circles.cols; ++i)
    //{
    //    float min = std::fabsf(circles.at<cv::Vec3f>(0, i)[2] - radius);
    //    if (min<min0)
    //    {
    //        index = i;
    //        min0 = min;
    //    }
    //}
    float max0 = circles.at<cv::Vec3f>(0, 0)[2];
    int index = 0;
    for (int i = 0; i < circles.cols; ++i)
    {
        float max = circles.at<cv::Vec3f>(0, i)[2];
        if (max > max0)
        {
            index = i;
            max0 = max;
        }
    }
    return circles.at<cv::Vec3f>(0, index);
}


int main()
{
    const std::string toSay("complete!");
    try
    {

        //auto im = cv::imread("imm.jpg");
        //RedballDetect(im);
        //return 0;
        boost::shared_ptr<AL::ALBroker> broker =
            AL::ALBroker::createBroker("MyBroker", "", 0, ip, 9559);

        AL::ALRobotPostureProxy rp(broker);
        rp.goToPosture("StandInit", 0.5);

        AL::ALVideoDeviceProxy vd(broker);
        switch (vd.getActiveCamera())
        {
        case AL::kTopCamera:
            std::cout << "kTopCamera\n";
            break;
        case AL::kBottomCamera:
            std::cout << "kBottomCamera\n";
            break;
        default:
            std::cout << "Other\n";
            break;
        }

        if (vd.setActiveCamera(AL::kTopCamera))
        {
            std::cout << "successfully!\n";
        }
        else
        {
            std::cout << "failed!\n";
        }
        //AL::ALRedBallDetectionProxy rbd(broker);

        AL::ALMotionProxy motion(broker);


        vd.setActiveCamera(AL::kBottomCamera);
        /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
        const auto clientName = vd.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 5);
        if (clientName == "test_1")
        {
            vd.unsubscribe("test_0");
            vd.unsubscribe("test_1");
            std::cout << "unsubscribe last subscribe and exit\n";
            return 0;
        }
        std::cout << "client name: " << clientName << '\n';
        /** Create an cv::Mat header to wrap into an opencv image.*/
        auto imgHeader = cv::Mat(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3);

        /** Create a OpenCV window to display the images. */
        cv::namedWindow("images");
        auto iii = 0;
        /** Main loop. Exit when pressing ESC.*/
        while ((char)cv::waitKey(30) != 27)
        {
            if (iii++ > 6)
            {
                //break;
            }
            /** Retrieve an image from the camera.
            * The image is returned in the form of a container object, with the
            * following fields:
            * 0 = width
            * 1 = height
            * 2 = number of layers
            * 3 = colors space index (see alvisiondefinitions.h)
            * 4 = time stamp (seconds)
            * 5 = time stamp (micro seconds)
            * 6 = image buffer (size of width * height * number of layers)
            */

            auto img = vd.getImageRemote(clientName);

            /** Access the image buffer (6th field) and assign it to the opencv image
            * container. */

            std::memcpy(imgHeader.data, (uchar*)img[6].GetBinary(), imgHeader.dataend - imgHeader.datastart);

            cv::imwrite("imm.jpg", imgHeader);
            /** Tells to ALVideoDevice that it can give back the image buffer to the
            * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
            vd.releaseImage(clientName);

            auto circle = RedballDetect(imgHeader);
            float x = circle[0], y = circle[1], r = circle[2];
            radius = r;
            cv::circle(imgHeader, { (int)x,(int)y }, (int)r, { 0,255,0 }, 2);
            std::cout << x << ", " << y << ", " << r << '\n';
            cv::imshow("images", imgHeader);

            auto alpha = std::atan2f((IMG_WIDTH / 2 - x) * std::tanf(ARG_WIDTH / 2), 200);
            auto beta = std::atan2f((IMG_HEIGHT / 2 - y) * std::tanf(ARG_HEIGHT / 2), 200);

            std::cout << "alpha: " << alpha << ", beta: " << beta << '\n';

            AL::ALValue names = "HeadPitch";
            AL::ALValue changes = -beta;
            float fractionMaxSpeed = 0.05f;
            //motion.changeAngles(names, changes, fractionMaxSpeed);


            ////Sleep(10000);
            //motion.moveTo(0.08f, 0.0f, alpha);
        }

        /** Cleanup.*/
        vd.unsubscribe(clientName);


        AL::ALMemoryProxy m(broker);
        auto result = m.getData("LandmarkDetected");
        std::cout << result.serializeToText() << '\n';

        AL::ALTextToSpeechProxy tts(broker);
        tts.say(toSay);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Caught exception: " << ex.what() << '\n';
        exit(1);
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
