/**
 * @file icar.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief ��������-��ȫģ����-�����ܣ�TOP��
 * @version 0.1
 * @date 2022-03-19
 * @note ��ӭ��λ��ȫģ�����С��飬��ͬ�����������
 * @copyright Copyright (c) 2022
 *
 */
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <opencv2/highgui.hpp>                  //OpenCV�ն˲���
#include <opencv2/opencv.hpp>                   //OpenCV�ն˲���
#include "../include/uart.hpp"                  //����ͨ������
#include "../include/detection.hpp"             //�ٶ�Paddle����ƶ��˲���
#include "../include/common.hpp"                //�����෽���ļ�
#include "image_preprocess.cpp"                 //ͼ��Ԥ������
#include "recognition/track_recognition.cpp"    //����ʶ�������
#include "controlcenter_cal.cpp"                //�������ļ�����
#include "motion_controller.cpp"                //���ܳ��˶�������
#include "recognition/cross_recognition.cpp"    //ʮ�ֵ�·ʶ����·���滮��
#include "recognition/garage_recognition.cpp"   //���⼰������ʶ����
#include "recognition/freezone_recognition.cpp" //������ʶ����
#include "detection/busy_detection.cpp"         //ʩ����AI�����·���滮��
#include "detection/slope_detection.cpp"        //�µ�AI�����·���滮��
#include "detection/gas_detection.cpp" 

using namespace std;
using namespace cv;



void callbackSignal(int signum);
void displayWindowDetailInit(void);
bool free(void);
std::shared_ptr<Driver> driver = nullptr; //��ʼ����������

enum RoadType
{
    BaseHandle = 0,   //������������
    RingHandle,       //������������
    CrossHandle,      //ʮ�ֵ�·����
    FreezoneHandle,   //����������
    GarageHandle,     //���⴦��
    GasstationHandle, //����վ����
    BusyareaHandle,   //ʩ��������
    SlopeHandle       //�µ�����
};
int counting = 0;
int main(int argc, char const* argv[])
{
    std::shared_ptr<Detection> detection = nullptr; //��ʼ��AIԤ��ģ��
    ImagePreprocess imagePreprocess;                //ͼ��Ԥ������
    TrackRecognition trackRecognition;              //����ʶ��
    ControlCenterCal controlCenterCal;              //������ʻ·�������
    MotionController motionController;              //�˶�����
    CrossroadRecognition crossroadRecognition;      //ʮ�ֵ�·����
    GarageRecognition garageRecognition;            //����ʶ��
    FreezoneRecognition freezoneRecognition;        //������ʶ����
    BusyareaDetection busyareaDetection;            //ʩ�������
    GasStationDetection gasStationDetection;         //����վ���
    SlopeDetection slopeDetection;                  //�µ����ţ������
    uint16_t counterRunBegin = 1;                   //���ܳ��������������ȴ�����ͷͼ��֡�ȶ�
    RoadType roadType = RoadType::BaseHandle;       //��ʼ����������
    uint16_t counterOutTrackA = 0;                  //�����������������A
    uint16_t counterOutTrackB = 0;                  //�����������������B
    uint16_t circlesThis = 1;                       //���ܳ���ǰ���е�Ȧ��
    uint16_t countercircles = 0;                    //Ȧ��������
    // USBת���ڵ��豸��Ϊ / dev/ttyUSB0
    driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
    if (driver == nullptr)
    {
        std::cout << "Create Uart-Driver Error!" << std::endl;
        return -1;
    }
    //���ڳ�ʼ�����򿪴����豸�����ô������ݸ�ʽ
    int ret = driver->open();
    if (ret != 0)
    {
        std::cout << "Uart Open failed!" << std::endl;
        return -1;
    }

    ipm.init(Size(COLSIMAGE, ROWSIMAGE), Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM��͸�ӱ任��ʼ��

    signal(SIGINT, callbackSignal); //�����˳��ź�

    motionController.loadParams(); //��ȡ�����ļ�
    trackRecognition.rowCutUp = motionController.params.rowCutUp;
    trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
    garageRecognition.disGarageEntry = motionController.params.disGarageEntry;
    if (motionController.params.GarageEnable) //�����ʹ��
        roadType = RoadType::GarageHandle;    //��ʼ����Ԫ��Ϊ����

    if (motionController.params.debug) //����ģʽ
    {
        displayWindowDetailInit();                                                                              //��ʾ���ڳ�ʼ��
        detection = Detection::DetectionInstance("/dev/video0", "../res/model/mobilenet-ssd-v1"); // ��Ƶ����Դ��������Ƶ | AIģ���ļ�
        printAiEnable = true;                                                                                   // ����AI�����ͼ����ƣ���ʱ
    }
    else //����ģʽ
    {

        // while (!driver->receiveStartSignal()) //���ڽ�����λ��-������ʼ�ź�
        // {
        //     ;
        // }

        cout << "--------- System start!!! -------" << endl;
        cout << "�ȴ�����!!!" << endl;
        detection = Detection::DetectionInstance("/dev/video0", "../res/model/mobilenet-ssd-v1"); // ��Ƶ����Դ��������Ƶ | AIģ���ļ�
        printAiEnable = false;                                                                    // �ر�AI�����ͼ����ƣ���ʡ����

        for (int i = 0; i < 10; i++) // 3��󷢳�
        {
            driver->carControl(0, PWMSERVOMID); //���ܳ�ֹͣ�˶�|������λ��ͨ��
            waitKey(100);
        }
    }

    while (1)
    {
        bool imshowRec = false; //��������ͼ����ʾ��־

        // ����֡ʱ����⣺��ʾ��֡ʱ��
        if (motionController.params.debug)
        {
            static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
          //  cout << "run frame time : " << startTime - preTime << "ms" << endl;
            preTime = startTime;
        }

        //[01] ��ƵԴѡ��
        std::shared_ptr<DetectionResult> resultAI = detection->getLastFrame(); //��ȡPaddle���߳�ģ��Ԥ������
        Mat frame = resultAI->rgb_frame;                                       //��ȡԭʼ����ͷͼ��
        if (motionController.params.debug)
        {
            // imshow("frame", resultAI->det_render_frame);
            savePicture(resultAI->det_render_frame); //����AIʶ��ͼ��
        }
        else
        {
            if (motionController.params.saveImage) //����ԭʼͼ��
                savePicture(frame);
        }

        //[02] ͼ��Ԥ����
        // Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // �������
        Mat imgaeCorrect = frame;
        Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

        //[03] ��������ʶ���������Track����������
        trackRecognition.trackRecognition(imageBinary); //������ʶ��
        if (motionController.params.debug)
        {
            Mat imageTrack = imgaeCorrect.clone();  // RGB
            trackRecognition.drawImage(imageTrack); //ͼ����ʾ������ʶ����
            imshow("imageTrack", imageTrack);
            savePicture(imageTrack);
        }

        // [04] ��������ʶ����·���滮
     // [04] ��������ʶ����·���滮
        if (motionController.params.GarageEnable) //����Ԫ���Ƿ�ʹ��
        {
            if (roadType == RoadType::GarageHandle || roadType == RoadType::BaseHandle)
            {
                countercircles++; //Ȧ������
                if (countercircles > 200)
                {
                    countercircles = 200;
                }
                if (garageRecognition.startingCheck(resultAI->predictor_results)) //��⵽���
                {
                    busyareaDetection.reset();   //ʩ��������ʼ��
                    freezoneRecognition.reset(); //������ʶ��λ
                    gasStationDetection.reset();



                    //cout<<SCcounter<<endl;

                    if (countercircles > 60)
                    {
                        circlesThis++;
                        countercircles = 0;
                    }
                }

                if (circlesThis >= motionController.params.circles && countercircles > 100) //���ʹ�ܣ�����NȦ
                    garageRecognition.entryEnable = true;

                if (garageRecognition.garageRecognition(trackRecognition, resultAI->predictor_results))
                {
                    roadType = RoadType::GarageHandle;

                    //  if (garageRecognition.garageStep == garageRecognition.GarageEntryFinish) //������
                    if (circlesThis >= 3)
                    {
                        if (countercircles > 0 && countercircles < 10)
                        {
                            driver->carControl(1.55, 820);
                            //   cout << "success_1" << endl;
                        }
                        else if (countercircles > 10 && countercircles < 26)
                        {
                            driver->carControl(1.55, 970);
                            //  cout << "success_2" << endl;
                        }
                        else if (countercircles > 26)
                        {
                            cout << ">>>>>>>   ������ !!!!!" << endl;
                            callbackSignal(0);
                        }
                    }
                    if (motionController.params.debug)
                    {
                        Mat imageGarage = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                        garageRecognition.drawImage(trackRecognition, imageGarage);
                        imshow("imageRecognition", imageGarage);
                        imshowRec = true;
                        savePicture(imageGarage);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }

        }
        //[05] ʩ�������
        if (motionController.params.BusyAreaEnable) //����Ԫ���Ƿ�ʹ��
        {
            if (roadType == RoadType::BusyareaHandle || roadType == RoadType::BaseHandle)
            {
                if (busyareaDetection.busyareaDetection(trackRecognition, resultAI->predictor_results))
                {
                    if (motionController.params.debug)
                    {
                        Mat imageBusyarea = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                        busyareaDetection.drawImage(trackRecognition, imageBusyarea);
                        imshow("imageRecognition", imageBusyarea);
                        imshowRec = true;
                        savePicture(imageBusyarea);

                        // ��ʾ�����������ͼ��
                        // Mat imageIpm;
                        // ipm.homography(imageBusyarea, imageIpm); //ͼ�����͸�ӱ任
                        // imshow("imageIpm", imageIpm);
                        // savePicture(imageIpm);
                    }
                    roadType = RoadType::BusyareaHandle;
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }


        // ����վ���
        if (motionController.params.GasStationEnable) //����Ԫ���Ƿ�ʹ��
        {
            if (roadType == RoadType::GasstationHandle || roadType == RoadType::BaseHandle)
            {
                if (gasStationDetection.gasStationDetection(trackRecognition, resultAI->predictor_results))
                {
                    if (motionController.params.debug)
                    {
                        Mat imageGasStation = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                        gasStationDetection.drawImage(trackRecognition, imageGasStation);
                        imshow("imageRecognition", imageGasStation);
                        imshowRec = true;
                        savePicture(imageGasStation);

                        // ��ʾ�����������ͼ��
                        // Mat imageIpm;
                        // ipm.homography(imageBusyarea, imageIpm); //ͼ�����͸�ӱ任
                        // imshow("imageIpm", imageIpm);
                        // savePicture(imageIpm);
                    }
                    roadType = RoadType::GasstationHandle;
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }



        // [06] �µ����ţ������·���滮
        if (motionController.params.SlopEnable) //����Ԫ���Ƿ�ʹ��
        {
            if (roadType == RoadType::SlopeHandle || roadType == RoadType::BaseHandle)
            {
                if (slopeDetection.slopeDetection(trackRecognition, resultAI->predictor_results))
                {
                    roadType = RoadType::SlopeHandle;
                    if (motionController.params.debug)
                    {
                        Mat imageFreezone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                        slopeDetection.drawImage(trackRecognition, imageFreezone);
                        imshow("imageRecognition", imageFreezone);
                        imshowRec = true;
                        savePicture(imageFreezone);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        //2022-08-13
       // [07] �����������ʶ�𣺷�AI��ʽ
        if (roadType == RoadType::FreezoneHandle || roadType == RoadType::BaseHandle)
        {
            if (freezoneRecognition.freezoneRecognition(trackRecognition, resultAI->predictor_results))
            {
                roadType = RoadType::FreezoneHandle;

                // cout<<SCcounter<<endl;
              //  free();
                if (motionController.params.debug)
                {
                    Mat imageFreezone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                    freezoneRecognition.drawImage(trackRecognition, imageFreezone);
                    imshow("imageRecognition", imageFreezone);
                    imshowRec = true;
                    savePicture(imageFreezone);
                }
            }
            else
                roadType = RoadType::BaseHandle;
        }

        // [08] ʮ�ֵ�·����
        if (motionController.params.CrossEnable) //����Ԫ���Ƿ�ʹ��
        {
            if (roadType == RoadType::CrossHandle || roadType == RoadType::BaseHandle)
            {
                if (crossroadRecognition.crossroadRecognition(trackRecognition))
                {
                    roadType = RoadType::CrossHandle;
                    if (motionController.params.debug)
                    {
                        Mat imageCross = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                        crossroadRecognition.drawImage(trackRecognition, imageCross);
                        imshow("imageRecognition", imageCross);
                        imshowRec = true;
                        savePicture(imageCross);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }


        // [09] �������ļ���
        if (trackRecognition.pointsEdgeLeft.size() < 30 && trackRecognition.pointsEdgeRight.size() < 30 && roadType != RoadType::FreezoneHandle && roadType != RoadType::SlopeHandle) //��ֹ�����������
        {
            counterOutTrackA++;
            counterOutTrackB = 0;
            if (counterOutTrackA > 20)
                //    cout << endl;
                callbackSignal(0);
        }
        else
        {
            counterOutTrackB++;
            if (counterOutTrackB > 50)
            {
                counterOutTrackA = 0;
                counterOutTrackB = 50;
            }
        }
        controlCenterCal.controlCenterCal(trackRecognition); //����������Ե��Ϣ����˶�·�����������ģ�

        // [10] �˶�����
        if (counterRunBegin > 20) //���ܳ�������ʱ��ǰ����ͼ��+AI�����ȶ�
        {
            //���������������
            motionController.pdController(controlCenterCal.controlCenter); // PD��������̬����
           //  cout<< GarageStep.garageStep <<endl;
            //���������ٶȿ���
            switch (roadType)
            {
            case RoadType::FreezoneHandle:                                               //�����������ٶ�
                if (motionController.params.FreezoneEnable)                              // AI������ٶ�
                    motionController.motorSpeed = motionController.params.speedFreezone; //���ٿ���
                else                                                                     //��AI������ٶ�
                    motionController.speedController(true, controlCenterCal);            //����ٿ���
                break;
            case RoadType::GasstationHandle:                                        //����վ�ٶ�
                motionController.motorSpeed = motionController.params.speedGasBusy; //���ٿ���
                break;
            case RoadType::BusyareaHandle:                                          //ʩ�����ٶ�
                motionController.motorSpeed = motionController.params.speedGasBusy; //���ٿ���
                break;
            case RoadType::SlopeHandle:                                          //�µ��ٶ�
                motionController.motorSpeed = motionController.params.speedSlop; //���ٿ���
                break;

            default:                                                      //����Ѳ�� | ʮ�� |�����ٶ�
                motionController.speedController(true, controlCenterCal); //����ٿ���
                break;
            }

            if (motionController.params.debug && circlesThis < 3)
                //     &&!freezoneRecognition.freezoneRecognition(trackRecognition, resultAI->predictor_results))//!free()) //����ģʽ�²����Ƴ����˶�
            {

                driver->carControl(motionController.motorSpeed, motionController.servoPwm); //����ͨ�ţ���̬���ٶȿ���
            //      cout<< "garageStep" << garageRecognition.garageStep<<endl;
               //    cout << "garageStep" <<garageRecognition.garageStep<< endl;
            //    cout << "circlesThis" <<circlesThis<< endl;
           //     cout << "countercircles" <<countercircles<< endl;   
            }
        }
        else
            counterRunBegin++;

        // [11]����ģʽ��ͼ����ʾ�ʹ�ͼ
        if (motionController.params.debug)
        {
            controlCenterCal.drawImage(trackRecognition, imgaeCorrect);
            switch (roadType)
            {
            case RoadType::BaseHandle:                                                                                             //������������
                putText(imgaeCorrect, "[1] Track", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::RingHandle:                                                                                            //������������
                putText(imgaeCorrect, "[1] Ring", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::CrossHandle:                                                                                            //ʮ�ֵ�·����
                putText(imgaeCorrect, "[1] Cross", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::FreezoneHandle:                                                                                            //����������
                putText(imgaeCorrect, "[1] Freezone", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::GarageHandle:                                                                                            //���⴦��
                putText(imgaeCorrect, "[1] Garage", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::GasstationHandle:                                                                                            //����վ����
                putText(imgaeCorrect, "[1] Gasstation", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::BusyareaHandle:                                                                                            //ʩ��������
                putText(imgaeCorrect, "[1] Busyarea", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            case RoadType::SlopeHandle:                                                                                           //�µ�����
                putText(imgaeCorrect, "[1] Slop", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾ����ʶ������
                break;
            }

            putText(imgaeCorrect, "v: " + formatDoble2String(motionController.motorSpeed, 2), Point(COLSIMAGE - 60, 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); //����

            string str = to_string(circlesThis) + "/" + to_string(motionController.params.circles);
            putText(imgaeCorrect, str, Point(COLSIMAGE - 50, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //��ʾȦ��
            if (!imshowRec)                                                                                                                    //���ֵ���ͼ��洢˳�����ʾһ����
            {
                Mat imageNone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //��ʼ��ͼ��
                imshow("imageRecognition", imageNone);
                savePicture(imageNone);
            }
            imshow("imageControl", imgaeCorrect);
            savePicture(imgaeCorrect);

            char c = waitKey(10);
        }
    }

    return 0;
}

/**
 * @brief OpenCVͼ����ʾ���ڳ�ʼ������ϸ����/Debugģʽ��
 *
 */
void displayWindowDetailInit(void)
{
    //[1] ��ֵ��ͼ��Gray
    string windowName = "imageTrack";
    cv::namedWindow(windowName, WINDOW_NORMAL); //ͼ������
    cv::resizeWindow(windowName, 320, 240);     //�ֱ���
    cv::moveWindow(windowName, 10, 10);         //����λ��

    //[2] ������Եͼ��RGB
    windowName = "imageRecognition";
    cv::namedWindow(windowName, WINDOW_NORMAL); //ͼ������
    cv::resizeWindow(windowName, 320, 240);     //�ֱ���
    cv::moveWindow(windowName, 10, 320);        //����λ��

    //[3] ԭʼͼ��/������RGB
    windowName = "imageControl";
    cv::namedWindow(windowName, WINDOW_NORMAL); //ͼ������
    cv::resizeWindow(windowName, 640, 480);     //�ֱ���
    cv::moveWindow(windowName, 350, 20);        //����λ��
}

bool free(void)
{

    for (counting = 0; counting < 200; counting++)
    {

        //   cout<<"success"<<endl;

                //  roadType = RoadType::FreezoneHandle;
        if (counting > 0 && counting < 4)
        {
            driver->carControl(1.6, 720);
        }
        else if (counting > 4 && counting < 8)
        {
            driver->carControl(1.6, 850);
        }
        else if (counting > 8 && counting < 200)
        {
            driver->carControl(1.6, 770);
        }
        else if (counting > 200 && counting < 210)
        {
            driver->carControl(1.6, 840);
        }
    }
    if (counting >= 1 && counting <= 200)
        return true;
    else
        return false;
}
/**
 * @brief ϵͳ�źŻص�������ϵͳ�˳�(����ctrl+c)
 *
 * @param signum �ź���
 */
void callbackSignal(int signum)
{
    driver->carControl(0, PWMSERVOMID); //���ܳ�ֹͣ�˶�
    cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
    exit(signum);
}
