// PluginBezier.hpp
// giy.hands@gmail.com (giy.hands@gmail.com)

#pragma once

#include <iostream>
#include <algorithm>
#include "SC_PlugIn.hpp"
#include "Vector2.h"


namespace Bezier {

class Bezier : public SCUnit {
public:
    Bezier();
    void makeCurveTable(int resolution);
//    Vector2* equidistantPoints(int howMany);
    void equidistantPoints(int howMany);
//    void makeCurve(Vector2 cp);
//    Vector2 makeCurve(float amt);
    
    Vector2 pointAtParameter(float t);
    double map(double value, double fromLow, double fromHigh, double toLow, double toHigh);
    
    // Destructor
     ~Bezier();

private:
    // Calc function
    void next(int nSamples);
    
    // Member variables
    Vector2 startPoint, endPoint;
    Vector2 controlPoint;
//    float* mCurveX = nullptr;
    float* mCurveY = nullptr;
    float* mArcLengths = nullptr;
    
    const int RESOLUTION = 2048;
    
    float RES_OVER_SR, HALF_RES;
    float curIdxPhase = 0;
    int mCount = 0;
    float mCtrlX = 0.f;
    float mFreq = 100.0;
    float freq = 0.0;
    float ctrlX, ctrlY;
    float arcLength = 0;
    float curveLength = 0;
    
//    Vector2* mEquiDistPoints;
};

} // namespace Bezier
