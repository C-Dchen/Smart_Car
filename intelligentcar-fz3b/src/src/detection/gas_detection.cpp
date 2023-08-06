#pragma once
/**
 * @file GasStation_detection.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief ʩ���������·���滮
 * @version 0.1
 * @date 2022-03-30
 *
 * @copyright Copyright (c) 2022
 *
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

int Label_White;
int Label_Blue = 1;
int counterRec_W1;
        int counterRec_W2;
        int counterRec_B1;
        int counterRec_B2;
        
class GasStationDetection
{
public:
    bool slowDown = false; //����ʹ��

    /**
     * @brief ����վ����ʼ��
     *
     */
    void reset(void)
    {
        gasStationStep = GasStationStep::GasStationNone;
        counterSession = 0;         //ͼ�񳡴μ�����
        counterRec = 0;             //ʩ������־��������
        counterRec_W1 = 0;
        counterRec_W2 = 0;
        counterRec_B1 = 0;
        counterRec_B2 = 0;
        lastPointsEdgeLeft.clear(); //��¼��һ����Ե�㼯����ʧ�ߣ�
        lastPointsEdgeRight.clear();
        Label_White = 0;
        Label_Blue = 1;
    }

    /**
     * @brief ����վ�����·���滮
     *
     * @param track ����ʶ����
     * @param detection AI�����
     */
    bool gasStationDetection(TrackRecognition& track, vector<PredictResult> predict)
    {
        slowDown = false;
        _pointNearCone = POINT(0, 0);
        pointEdgeDet.clear();

        switch (gasStationStep)
        {
        case GasStationStep::GasStationNone: //[01] ����վ��־���
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_GASSTATION) //����վ��־���
                {
                    counterRec++;
                    break;
                }
            }
            if (counterRec)
            {
                counterSession++;
                if (counterRec > 3 && counterSession < 8)
                {
                    gasStationStep = GasStationStep::GasStationEnable; //����վʹ��
                    counterRec = 0;
                    counterSession = 0;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            break;

        case GasStationStep::GasStationEnable: //[02] ����վʹ��
        {
            vector<POINT> pointsCone = searchCones(predict);
            _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointsCone); //��������׶Ͱ
            if (_pointNearCone.x > ROWSIMAGE * 0.1)                               //��������ʼ�����ұ�׶Ͱ��׼�����
            {
                counterRec++;
                if (counterRec >= 2)
                {
                    gasStationStep = GasStationStep::GasStationEnter; //��վʹ��
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            else if (_pointNearCone.x > 0.1 && _pointNearCone.x < ROWSIMAGE * 0.4)
            {
                slowDown = true; //��չ����
            }
            
            
            
                for (int i = 0; i < predict.size(); i++)
                {
                    if (predict[i].label == LABEL_WRITEONE) //����վ��־���
                    {
                        counterRec_W1++;
                        break;
                    }

                    if (predict[i].label == LABEL_WRITETWO) //����վ��־���
                    {
                        counterRec_W2++;
                        break;
                    }
                    
                }
                if (counterRec_W1 || counterRec_W2)
                {
                    counterSession++;
                    if (counterRec_W1 > 1 && counterSession < 8)
                    {
                        Label_White = 1;
                      //  cout<<"White One!!!!!!"<<endl;
                        counterRec_W1 = 0;
                        counterSession = 0;
                    }
                    else if (counterRec_W2 > 1 && counterSession < 8)
                    {
                        Label_White = 2;
                       // cout<<"White Two!!!!!!"<<endl;
                        counterRec_W2 = 0;
                        counterSession = 0;
                    }
                    else if (counterSession >= 8)
                    {
                        counterRec_W1 = 0;
                        counterRec_W2 = 0;
                        counterSession = 0;
                    }
                }

            break;
        }

        case GasStationStep::GasStationEnter: //[03] ��վʹ��
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2) //��һ�׶Σ���������Ե����ʱ
            {
                vector<POINT> pointsCone = searchCones(predict);                      //����׶Ͱ������
                _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointsCone); //��������׶Ͱ
                if (_pointNearCone.x > 0)                                             //������Ч
                {
                    POINT startPoint = POINT((_pointNearCone.x + ROWSIMAGE) / 2, (_pointNearCone.y + COLSIMAGE) / 2); //����㣺��
                    double k = 0, b = 0;
                    k = (float)(_pointNearCone.y - startPoint.y) / (float)(_pointNearCone.x - startPoint.x);
                    b = _pointNearCone.y - k * _pointNearCone.x;

                    if (b < 0)
                        b = 0;
                    else if (b >= COLSIMAGE)
                        b = COLSIMAGE - 1;
                    POINT endPoint = POINT(0, b);    //�����յ㣺��
                    POINT midPoint = _pointNearCone; //�����е�
                    vector<POINT> input = { startPoint, midPoint, endPoint };
                    vector<POINT> repair = Bezier(0.02, input);
                    track.pointsEdgeRight = repair;
                    track.pointsEdgeLeft.clear();

                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
                    }
                }
            }
            else //�ڶ��׶Σ��������׶Ͱ��������Ѳ������
            {
                vector<POINT> pointsCone = searchCones(predict);       //����׶Ͱ������
                POINT coneRightDown = searchRightDownCone(pointsCone); //���·�׶Ͱ
                _pointNearCone = coneRightDown;
                counterSession++;
                if ((coneRightDown.x > ROWSIMAGE / 3 && coneRightDown.y > COLSIMAGE - 80) || counterSession > 20)
                {
                    counterRec++;
                    if (counterRec >= 2)
                    {
                        gasStationStep = GasStationStep::GasStationCruise; //Ѳ��ʹ��
                        counterRec = 0;
                        counterSession = 0;
                    }
                }
                if (coneRightDown.x > 0) //��վ����
                {
                    POINT startPoint = POINT((coneRightDown.x + ROWSIMAGE) / 2, (coneRightDown.y + COLSIMAGE) / 2); //����㣺��
                    double k = 0, b = 0;
                    k = (float)(coneRightDown.y - startPoint.y) / (float)(coneRightDown.x - startPoint.x);
                    b = coneRightDown.y - k * coneRightDown.x;

                    if (b < 0)
                        b = 0;
                    else if (b >= COLSIMAGE)
                        b = COLSIMAGE - 1;
                    POINT endPoint = POINT(0, b);   //�����յ㣺��
                    POINT midPoint = coneRightDown; //�����е�
                    vector<POINT> input = { startPoint, midPoint, endPoint };
                    vector<POINT> repair = Bezier(0.02, input);
                    track.pointsEdgeRight = repair;
                    track.pointsEdgeLeft.clear();

                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
                    }
                }



            }

            break;
        }

        case GasStationStep::GasStationCruise: //[04] Ѳ��ʹ��
        {
            cout<<"It's Cruise"<<endl;
            vector<POINT> pointsCone = searchCones(predict);      //����׶Ͱ������
            vector<POINT> conesLeft = searchLeftCone(pointsCone); //������׶Ͱ

            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_BLUEONE) //����վ��־���
                {
                  //  cout<<"have found Blue one"<<endl;
                    counterRec_B1++;
                    break;
                }

                if (predict[i].label == LABEL_BLUETWO) //����վ��־���
                {
                   // cout<<"have found Blue two"<<endl;
                    counterRec_B2++;
                    break;
                }

            }
            if (counterRec_B1 || counterRec_B2)
            {
                counterSession++;
                if (counterRec_B1 > 3 && counterSession < 8)
                {
                    Label_Blue = 1;
                    //cout<<"Blue One!!!!!!"<<endl;
                    counterRec_B1 = 0;
                    counterSession = 0;
                }
                else if (counterRec_B2 > 2 && counterSession < 8)
                {
                    Label_Blue = 2;
                    //cout<<"Blue Two!!!!!!"<<endl;
                    counterRec_B2 = 0;
                    counterSession = 0;
                }
                else if (counterSession >= 8)
                {
                    counterRec_B1 = 0;
                    counterRec_B2 = 0;
                    counterSession = 0;
                }
            }

              
            // if (pointsCone.size() < 2 && track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2)
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 6 && track.pointsEdgeRight.size() > ROWSIMAGE / 6)
            {
                slowDown = true; //��վ����
                counterRec++;
                if (counterRec >= 2 && Label_White == Label_Blue)
                {
                    cout<<"1Label_White="<<Label_White<<"|"<<"1Label_Blue="<<Label_Blue<<endl;
                    gasStationStep = GasStationStep::GasStationExit; //��վʹ��
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            else
                counterRec = 0;

            if (conesLeft.size() >= 2)
            {
                int indexMin = 0;
                int indexMax = 0;
                for (int i = 0; i < conesLeft.size(); i++)
                {
                    if (conesLeft[i].x > conesLeft[indexMax].x)
                        indexMax = i;
                    if (conesLeft[i].x < conesLeft[indexMin].x)
                        indexMin = i;
                }

                if (indexMin != indexMax) //��ʼ����
                {
                    double k = 0, b = 0;
                    k = (float)(conesLeft[indexMax].y - conesLeft[indexMin].y) / (float)(conesLeft[indexMax].x - conesLeft[indexMin].x);
                    b = conesLeft[indexMax].y - k * conesLeft[indexMax].x;

                    if (k != 0 && b != 0)
                    {
                        POINT startPoint = POINT(-b / k, 0);                                                          //������㣺��
                        POINT endPoint = POINT(0, b);                                                                 //�����յ㣺��
                        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //�����е�
                        vector<POINT> input = { startPoint, midPoint, endPoint };
                        vector<POINT> repair = Bezier(0.02, input);

                        track.pointsEdgeRight = predictEdgeRight(repair); //������Ԥ���ұ�Ե

                        if (repair.size() > 10) //���Ե���У������ҹ�����
                        {
                            int index = repair.size() * 0.3;
                            track.pointsEdgeLeft.clear();
                            for (int i = index; i < repair.size(); i++)
                            {
                                track.pointsEdgeLeft.push_back(repair[i]);
                            }
                        }
                        else
                            track.pointsEdgeLeft = repair;

                        lastPointsEdgeLeft = track.pointsEdgeLeft;
                    }
                }
            }
            else if (pointsCone.size() > 3)
            {
                track.pointsEdgeLeft = lastPointsEdgeLeft;
                track.pointsEdgeRight = predictEdgeRight(track.pointsEdgeLeft); //������Ԥ���ұ�Ե
            }



            //����վ���
            counterSession++;
            POINT coneRightDown = searchRightDownCone(pointsCone);                               //���·�׶Ͱ
            if (((coneRightDown.x < ROWSIMAGE / 2 && counterSession > 12) || counterSession > 30) && Label_White == Label_Blue) //���·�׶Ͱ������
            {
                cout<<"Label_White="<<Label_White<<"|"<<"Label_Blue="<<Label_Blue<<endl;
                gasStationStep = GasStationStep::GasStationExit; //��վʹ��
                counterRec = 0;
                counterSession = 0;
            }

            



            break;
        }
        // case BusyareaStep::BusyareaExit: //[05] ��վʹ��
        // {
        //     slowDown = true;                                                                                 //��վ����
        //     if (track.pointsEdgeLeft.size() > ROWSIMAGE / 6 && track.pointsEdgeRight.size() > ROWSIMAGE / 6) //�ұ�Ե���У�������վ�������
        //     {
        //         for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        //         {
        //             if (track.pointsEdgeRight[i].y > COLSIMAGE * 0.6)
        //                 track.pointsEdgeRight[i].y = COLSIMAGE * 0.6;
        //         }
        //     }
        //     else
        //     {
        //         track.pointsEdgeLeft.clear();
        //         track.pointsEdgeRight.clear();
        //     }

        //     counterRec++;
        //     if (counterRec > 10)
        //     {
        //         busyareaStep = BusyareaStep::BusyareaNone; //ʩ��������
        //         counterRec = 0;
        //         counterSession = 0;
        //     }
        //     break;
        // }
        case GasStationStep::GasStationExit: //[05] ��վʹ��
        {
            vector<POINT> pointsCone = searchCones(predict);  //����׶Ͱ������
            POINT coneLeftUp = searchRightUpCone(pointsCone); //�������Ϸ���׶Ͱ���ڲ���

            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 4 && track.pointsEdgeRight.size() > ROWSIMAGE / 4 && pointsCone.size() < 3)
            {
                gasStationStep = GasStationStep::GasStationNone; //��վ����
                counterRec = 0;
                counterSession = 0;
            }
            else
            {
                if (coneLeftUp.x > 0)
                {
                    POINT p1 = POINT(ROWSIMAGE - 10, coneLeftUp.y / 2);
                    POINT p2 = POINT((coneLeftUp.x + ROWSIMAGE) / 2, coneLeftUp.y / 2);
                    POINT p3 = coneLeftUp;
                    POINT p4 = POINT(coneLeftUp.x / 2, (coneLeftUp.y + COLSIMAGE) / 2);
                    vector<POINT> input = { p1, p2, p3, p4 };
                    vector<POINT> repair = Bezier(0.02, input);

                    track.pointsEdgeLeft = repair;
                    lastPointsEdgeLeft = repair;
                    track.pointsEdgeRight.clear();
                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeRight.push_back(POINT(repair[i].x, COLSIMAGE - 1));
                    }
                    lastPointsEdgeRight = track.pointsEdgeRight;
                }
                else
                {
                    track.pointsEdgeLeft = lastPointsEdgeLeft;
                    track.pointsEdgeRight = lastPointsEdgeRight;
                }
            }

            break;
        }
        }

        if (gasStationStep == GasStationStep::GasStationNone) //���ؼ���վ����ģʽ��־
            return false;
        else
            return true;
    }







    /**
     * @brief ʶ����ͼ�����
     *
     */
    void
        drawImage(TrackRecognition track, Mat& imageGasStation)
    {
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            circle(imageGasStation, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2, Scalar(92, 92, 205), -1); //׶Ͱ���꣺��ɫ
        }

        // ������Ե
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(imageGasStation, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                Scalar(0, 255, 0), -1); //��ɫ��
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(imageGasStation, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                Scalar(0, 255, 255), -1); //��ɫ��
        }

        //��ʾ����վ״̬
        string state = "None";
        switch (gasStationStep)
        {
        case GasStationEnable:
            state = "Enable";
            break;
        case GasStationEnter:
            state = "Enter";
            break;
        case GasStationCruise:
            state = "Cruise";
            break;
        case GasStationExit:
            state = "Exit";
            break;
        }
        putText(imageGasStation, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        if (_pointNearCone.x > 0)
            circle(imageGasStation, Point(_pointNearCone.y, _pointNearCone.x), 3, Scalar(200, 200, 200), -1);
    }

private:
    POINT _pointNearCone;
    vector<POINT> pointEdgeDet;       // AIԪ�ؼ���Ե�㼯
    vector<POINT> lastPointsEdgeLeft; //��¼��һ����Ե�㼯����ʧ�ߣ�
    vector<POINT> lastPointsEdgeRight;

    enum GasStationStep
    {
        GasStationNone = 0, //δ����
        GasStationEnable,   //����վ����ʹ�ܣ���־ʶ��ɹ���
        GasStationEnter,    //����վ��վ
        GasStationCruise,   //����վѲ��
        GasStationExit      //����վ��վ
    };

    GasStationStep gasStationStep = GasStationStep::GasStationNone;
    uint16_t counterSession = 0; //ͼ�񳡴μ�����
    uint16_t counterRec = 0;     //����վ��־��������

    /**
     * @brief ��AI������м���׶Ͱ���꼯��
     *
     * @param predict AI�����
     * @return vector<POINT>
     */
    vector<POINT> searchCones(vector<PredictResult> predict)
    {
        vector<POINT> cones;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CONE) //׶Ͱ���
            {
                cones.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }

        pointEdgeDet = cones;
        return cones;
    }

    /**
     * @brief ���������������Ե�����׶Ͱ����
     *
     * @param pointsEdgeLeft ������Ե�㼯
     * @param predict AI�����
     * @return POINT
     */
    POINT searchNearestCone(vector<POINT> pointsEdgeLeft, vector<POINT> pointsCone)
    {
        POINT point(0, 0);
        double disMin = 50; //�ұ�Ե׶Ͱ���������Ե��С����

        if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
            return point;

        POINT a = pointsEdgeLeft[pointsEdgeLeft.size() / 4];
        POINT b = pointsEdgeLeft[pointsEdgeLeft.size() / 2];

        for (int i = 0; i < pointsCone.size(); i++)
        {
            double dis = distanceForPoint2Line(a, b, pointsCone[i]);
            if (dis < disMin && pointsCone[i].x > point.x)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    /**
     * @brief �������·���׶Ͱ����
     *
     * @param pointsCone
     * @return POINT
     */
    POINT searchRightDownCone(vector<POINT> pointsCone)
    {
        POINT point(0, 0);

        if (pointsCone.size() <= 0)
            return point;

        int index = 0;
        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y > COLSIMAGE / 2 && pointsCone[i].x > point.x)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    /**
     * @brief �����󷽵�׶Ͱ����
     *
     * @param pointsCone
     * @return vector<POINT>
     */
    vector<POINT> searchLeftCone(vector<POINT> pointsCone)
    {
        vector<POINT> points;

        if (pointsCone.size() <= 0)
            return points;

        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y < COLSIMAGE / 2)
            {
                points.push_back(pointsCone[i]);
            }
        }

        return points;
    }

    /**
     * @brief �������Ϸ���׶Ͱ����
     *
     * @param pointsCone
     * @return vector<POINT>
     */
    POINT searchRightUpCone(vector<POINT> pointsCone)
    {
        POINT point(0, 0);

        if (pointsCone.size() <= 0)
            return point;

        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y > point.y && pointsCone[i].x < ROWSIMAGE * 0.8)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    vector<POINT> predictEdgeRight(vector<POINT> pointsEdgeLeft)
    {
        int offset = 200; //�ұ�Եƽ�Ƴ߶�
        vector<POINT> pointsEdgeRight;
        POINT startPoint(0, 0);
        POINT endPoint(0, 0);

        if (pointsEdgeLeft.size() < 3)
            return pointsEdgeRight;

        // Start
        Point2d startIpm = ipm.homography(Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); //͸�ӱ任
        Point2d prefictRight;
        if (startIpm.x + offset >= COLSIMAGEIPM) //ƽ�Ʊ�Ե
            return pointsEdgeRight;
        else
            prefictRight = Point2d(startIpm.x + offset, startIpm.y);

        Point2d startIipm = ipm.homographyInv(prefictRight); //��͸�ӱ任
        startPoint = POINT(startIipm.y, startIipm.x);

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y, pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); //͸�ӱ任
        prefictRight = Point2d(endIpm.x + offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictRight); //��͸�ӱ任
        endPoint = POINT(endtIipm.y, endtIipm.x);

        //����
        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //�����е�
        vector<POINT> input = { startPoint, midPoint, endPoint };
        vector<POINT> repair = Bezier(0.02, input);

        for (int i = 0; i < repair.size(); i++)
        {
            if (repair[i].x >= ROWSIMAGE)
                repair[i].x = ROWSIMAGE - 1;

            else if (repair[i].x < 0)
                repair[i].x = 0;

            else if (repair[i].y >= COLSIMAGE)
                repair[i].y = COLSIMAGE - 1;
            else if (repair[i].y < 0)
                repair[i].y = 0;

            pointsEdgeRight.push_back(repair[i]);
        }

        return pointsEdgeRight;
    }
};