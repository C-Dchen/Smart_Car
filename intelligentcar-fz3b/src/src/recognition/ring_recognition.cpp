#pragma once
/**
 * @file freezone_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief ������ʶ����·���滮
 * @version 0.1
 * @date 2022-03-30
 * @copyright Copyright (c) 2022
 * @note �����������裺
 *                      [1] ʶ��������ڱ�־���ȱ������μ�⣩
 *                      [2] ��������������&·������
 *                      [3] �뷺�������
 *                      [4] ʶ���������ڱ�־���ȱ������μ�⣩
 *                      [5] ��������������&·������
 *                      [6] �����������
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognition/track_recognition.cpp"

using namespace cv;
using namespace std;

int dajiao;
int budajiao;
int CircleNum= 0;




class RingRecognition
{
public:
    enum RingStep
    {
        None = 0,            //�뿪Բ��
        RingReady,
        RingComing,
        RingEntering,    //�뻷
        InRing, //����
        Over,
    };
    RingStep ringStep = RingStep::None; //������״̬
    int ring_to_in = 0;
    int ring__to_in = 0;
    int flag_ = 0;
    int rBRD = 0;              //rowBreakRightDown
    int rBLD = 0;
    int RD_Flag = 0;          //None->Ready
    int Flagone = 0;            //���߱�־λ
    int Flagtwo = 0;            //��Ǳ�־λ
    int chongzhi = 0;
    int Flagthr = 0;   //��ʼλ

    /**
     * @brief Բ��ʶ���ʼ��
     *
     */
    void reset(void)
    {
        cout << Flagthr << endl;
        ringStep = RingStep::None; //��������ʹ״̬
        ring_to_in = 0;
        ring__to_in = 0;
        RDFlag = 0;
        RD_Flag = 0;
        Flagone = 0;            //���߱�־λ
        Flagtwo = 0;            //��Ǳ�־λ
        Flagthr = 0;
        chongzhi = 0;
    }

    /**
   * @brief ����ʮ������ͻ���У����£�
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
    int RDFlag = 0;
    void searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) //Ѱ���ұ������
        {
            if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRightDown].y)
            {
                rowBreakRightDown = i;
                counter = 0;

            }
            else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRightDown].y) //ͻ������
            {

                counter++;

                if (counter > 5)
                {
                    RDFlag++;
                    rBRD = rowBreakRightDown;

                }
            }
        }

        // return rowBreakRightDown;
    }


    /**
    * @brief ����ʮ������ͻ���У����£�
    *
    * @param pointsEdgeLeft
    * @return uint16_t
    */
    int LDFlag = 0;
    void searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++) //Ѱ����������
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeftUp].y)
            {
                rowBreakLeftUp = i;
                counter = 0;

            }
            else if (pointsEdgeLeft[i].y < pointsEdgeLeft[rowBreakLeftUp].y) //ͻ������
            {
                counter++;

                if (counter > 5)
                {
                    LDFlag++;
                    rBLD = rowBreakLeftUp;
                }
            }
        }

        // return rowBreakLeftUp;
    }
    
    
    
    
  
    /**
   * @brief �ȱ������μ���
   *
   * @param peaks ����������������������
   * @param ipm ͸�ӱ任����
   * @return sigma �߳�����
   */
    vector<POINT> peaks;
    vector<POINT> peakTriangleIpm; //�����εĶ���(IPM)
    double regularTriangleCheck(vector<POINT> peaks)
    {
        if (peaks.size() != 3)
            return false;

        for (size_t i = 0; i < peaks.size(); i++) //����͸�ӱ任
        {
            Point2d peak = Point2d(peaks[i].y, peaks[i].x);
            peak = ipm.homography(peak);
            peaks[i] = POINT(peak.y, peak.x);
        }
        peakTriangleIpm = peaks;

        int vectorX, vectorY;
        vector<int> length;
        vectorX = pow(peaks[0].x - peaks[1].x, 2);
        vectorY = pow(peaks[0].y - peaks[1].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //�߳���A

        vectorX = pow(peaks[1].x - peaks[2].x, 2);
        vectorY = pow(peaks[1].y - peaks[2].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //�߳���B

        vectorX = pow(peaks[2].x - peaks[0].x, 2);
        vectorY = pow(peaks[2].y - peaks[0].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //�߳���C

        double _sigma = sigma(length);
       // cout << "lengthA:" << length[0] << " | lengthB:" << length[1] << " | lengthC:" << length[2] << " | sigma:" << _sigma << endl;

        return _sigma;
    }
    /**
     * @brief Բ����ʶ����·���滮
     *
     * @param track ����ʶ����
     */



    bool ringRecognition(TrackRecognition& track,vector<PredictResult> predict)
    {
    
      

    
    
        
        uint16_t rowBreakRightDown = 0;                              //ʮ��ͻ���У����£�
        uint16_t rowBreakLeftUp = 0;
        double k = 0, b = 0;                                         //ֱ��б��
        int t = 0;                                                     //�ж϶���
        peakTriangleIpm.clear();
        
        
        ring_to_in = 0;
        switch (ringStep)
        {
        case(RingStep::None):
        {
          if (track.spurroad.size() >= 1) //��·��־
          {
            int indexSpurroad = track.spurroad.size() - 1;
            if (track.spurroad[indexSpurroad].x < track.pointsEdgeRight[rBRD].x && track.spurroad[indexSpurroad].x < track.pointsEdgeLeft[rBLD].x)
                {
                    //�ȱ������α߳�����
                    vector<POINT> peaks;
                    peaks.push_back(track.pointsEdgeLeft[rBLD]);
                    peaks.push_back(track.spurroad[indexSpurroad]);
                    peaks.push_back(track.pointsEdgeRight[rBRD]);
                
                  if (regularTriangleCheck(peaks) < 30.0) //������⵽�ȱ�������
                  {
                     Flagthr = 1; 
                  }
              } 
           }   
                               
        
        
        
        
        
        
             //  cout << "track.stdevRight=====" << track.stdevRight << endl;
            //  cout<<"Flagone="<<Flagone<<"|"<<"Flagtwo="<<Flagtwo<<endl;
             // cout<<Flagthr<<endl;
            if (Flagtwo == 0  && Flagthr ==1)
            {
                // cout << "track.stdevLeft=====" << track.stdevLeft << endl;
               // cout << "track.stdevRight=====" << track.stdevRight << endl;
                for (auto i : track.pointsEdgeLeft)
                {
                    if (i.y == 0) t++;
                    // cout << "track.stdevLeft=====" << track.stdevLeft << endl;
                     // cout << "t=====" << t << endl;
                    if (t >50  && track.stdevLeft < 20 && track.stdevRight > 90)
                    {
                        searchBreakRightDown(track.pointsEdgeRight); //����ʮ������ͻ���У����£�
                       // cout<<"RDFlag====="<<RDFlag<<endl;
                        if (RDFlag > 10)
                        {
                            // cout<<"---------------------------------"<<endl;
                             //RD_Flag++;

                            Flagone = 1;
                            // cout<<"dajiao======================="<<dajiao<<endl;
                             //cout<< "RD_Flag==================="<<RD_Flag<<endl;
                            k = (float)(track.pointsEdgeRight[1].y - track.pointsEdgeRight[rBRD].y) / (float)(track.pointsEdgeRight[1].x - track.pointsEdgeRight[rBRD].x);

                            b = track.pointsEdgeRight[1].y - k * track.pointsEdgeRight[1].x;

                            for (int i = rBRD; i <= track.pointsEdgeRight.size(); i++)
                            {
                                track.pointsEdgeRight[i].y = min((int)(k * track.pointsEdgeRight[i].x + b), COLSIMAGE - 1);
                            }
                            //dajiao++;
                        }



                    }


                }

                if (RDFlag > 10)
                {
                    dajiao++;

                    
                    if (CircleNum == 1)
                    {
                        if (dajiao > 60)
                        {
                            cout << "It's CircleOne!" << endl;
                            ringStep = RingStep::RingEntering;
                            dajiao = 0;
                            Flagtwo = 1;
                            Flagthr = 0;
                        }
                    }

                    if (CircleNum == 2)
                    {
                        if (dajiao > 52)
                        {
                            cout << "It's CircleTwo!" << endl;
                            ringStep = RingStep::RingEntering;
                            dajiao = 0;
                            Flagtwo = 1;
                            Flagthr = 0;
                        }
                    }
                    
                    

                }
            }





            else if (Flagone == 1 && Flagtwo == 1)
            {
                searchBreakLeftDown(track.pointsEdgeLeft);
                //cout << "LDFlag===============" << LDFlag << endl;
                //cout << "chongzhi:" << chongzhi << endl;
                if (LDFlag > 1)
                {
                   // cout << "OutOutOut!!!" << endl;


                    k = (float)(track.pointsEdgeRight[track.pointsEdgeRight.size() - 10].y - track.pointsEdgeLeft[rBLD].y) / (float)(track.pointsEdgeRight[track.pointsEdgeRight.size() - 10].x - track.pointsEdgeLeft[rBLD].x);

                    b = track.pointsEdgeLeft[rBLD].y - k * track.pointsEdgeLeft[rBLD].x;

                    // cout<<"k="<<k<<"|"<<"b="<<b<<endl;

                    for (int i = rBLD; i <= track.pointsEdgeLeft.size(); i++)
                    {
                        track.pointsEdgeLeft[i].y = max((int)(k * track.pointsEdgeLeft[i].x + b), 0);
                    }
                    chongzhi++;



                    if (chongzhi > 38)
                    {
                        Flagone = 0;
                        Flagtwo = 0;
                    }
                }


            }

            break;


        }
        case(RingStep::RingEntering):
        {
            ringStep = RingStep::RingEntering;

            budajiao++;
            // cout<<"budajiao==============="<<budajiao<<endl;

            if (budajiao > 20)  ringStep = RingStep::None;
            break;


        }
  

        }
        //����ʶ����
        if (ringStep == RingStep::None || ringStep == RingStep::Over)
            return false;
        else
            return true;
    }

    void drawImage(TrackRecognition track, Mat& imageRingarea)
    {

        //��ʾʩ����״̬
        string state = "None";
        switch (ringStep)
        {
            // case RingStep::RingReady:
                 //state = "RingReady";
                 //break;
        case RingStep::RingComing:
            state = "RingComing";
            break;
        case RingStep::RingEntering:
            state = "Enter";
            break;
        case RingStep::InRing:
            state = "InRing";
            break;
        }
        putText(imageRingarea, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

};
