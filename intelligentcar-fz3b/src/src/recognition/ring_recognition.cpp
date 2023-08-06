#pragma once
/**
 * @file freezone_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 泛行区识别与路径规划
 * @version 0.1
 * @date 2022-03-30
 * @copyright Copyright (c) 2022
 * @note 泛行区处理步骤：
 *                      [1] 识别泛行区入口标志（等边三角形检测）
 *                      [2] 赛道补偿点搜索&路径处理
 *                      [3] 入泛行区完成
 *                      [4] 识别泛行区出口标志（等边三角形检测）
 *                      [5] 赛道补偿点搜索&路径处理
 *                      [6] 出泛行区完成
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
        None = 0,            //离开圆环
        RingReady,
        RingComing,
        RingEntering,    //入环
        InRing, //环内
        Over,
    };
    RingStep ringStep = RingStep::None; //泛行区状态
    int ring_to_in = 0;
    int ring__to_in = 0;
    int flag_ = 0;
    int rBRD = 0;              //rowBreakRightDown
    int rBLD = 0;
    int RD_Flag = 0;          //None->Ready
    int Flagone = 0;            //补线标志位
    int Flagtwo = 0;            //打角标志位
    int chongzhi = 0;
    int Flagthr = 0;   //起始位

    /**
     * @brief 圆环识别初始化
     *
     */
    void reset(void)
    {
        cout << Flagthr << endl;
        ringStep = RingStep::None; //泛行区行使状态
        ring_to_in = 0;
        ring__to_in = 0;
        RDFlag = 0;
        RD_Flag = 0;
        Flagone = 0;            //补线标志位
        Flagtwo = 0;            //打角标志位
        Flagthr = 0;
        chongzhi = 0;
    }

    /**
   * @brief 搜索十字赛道突变行（右下）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
    int RDFlag = 0;
    void searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) //寻找右边跳变点
        {
            if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRightDown].y)
            {
                rowBreakRightDown = i;
                counter = 0;

            }
            else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRightDown].y) //突变点计数
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
    * @brief 搜索十字赛道突变行（左下）
    *
    * @param pointsEdgeLeft
    * @return uint16_t
    */
    int LDFlag = 0;
    void searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++) //寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeftUp].y)
            {
                rowBreakLeftUp = i;
                counter = 0;

            }
            else if (pointsEdgeLeft[i].y < pointsEdgeLeft[rowBreakLeftUp].y) //突变点计数
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
   * @brief 等边三角形检验
   *
   * @param peaks 输入三角形三个顶点坐标
   * @param ipm 透视变换参数
   * @return sigma 边长方差
   */
    vector<POINT> peaks;
    vector<POINT> peakTriangleIpm; //三角形的顶点(IPM)
    double regularTriangleCheck(vector<POINT> peaks)
    {
        if (peaks.size() != 3)
            return false;

        for (size_t i = 0; i < peaks.size(); i++) //坐标透视变换
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
        length.push_back(sqrt(vectorX + vectorY)); //边长：A

        vectorX = pow(peaks[1].x - peaks[2].x, 2);
        vectorY = pow(peaks[1].y - peaks[2].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //边长：B

        vectorX = pow(peaks[2].x - peaks[0].x, 2);
        vectorY = pow(peaks[2].y - peaks[0].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //边长：C

        double _sigma = sigma(length);
       // cout << "lengthA:" << length[0] << " | lengthB:" << length[1] << " | lengthC:" << length[2] << " | sigma:" << _sigma << endl;

        return _sigma;
    }
    /**
     * @brief 圆环区识别与路径规划
     *
     * @param track 赛道识别结果
     */



    bool ringRecognition(TrackRecognition& track,vector<PredictResult> predict)
    {
    
      

    
    
        
        uint16_t rowBreakRightDown = 0;                              //十字突变行（右下）
        uint16_t rowBreakLeftUp = 0;
        double k = 0, b = 0;                                         //直线斜率
        int t = 0;                                                     //判断丢线
        peakTriangleIpm.clear();
        
        
        ring_to_in = 0;
        switch (ringStep)
        {
        case(RingStep::None):
        {
          if (track.spurroad.size() >= 1) //岔路标志
          {
            int indexSpurroad = track.spurroad.size() - 1;
            if (track.spurroad[indexSpurroad].x < track.pointsEdgeRight[rBRD].x && track.spurroad[indexSpurroad].x < track.pointsEdgeLeft[rBLD].x)
                {
                    //等边三角形边长计算
                    vector<POINT> peaks;
                    peaks.push_back(track.pointsEdgeLeft[rBLD]);
                    peaks.push_back(track.spurroad[indexSpurroad]);
                    peaks.push_back(track.pointsEdgeRight[rBRD]);
                
                  if (regularTriangleCheck(peaks) < 30.0) //连续检测到等边三角形
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
                        searchBreakRightDown(track.pointsEdgeRight); //搜索十字赛道突变行（右下）
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
        //返回识别结果
        if (ringStep == RingStep::None || ringStep == RingStep::Over)
            return false;
        else
            return true;
    }

    void drawImage(TrackRecognition track, Mat& imageRingarea)
    {

        //显示施工区状态
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
