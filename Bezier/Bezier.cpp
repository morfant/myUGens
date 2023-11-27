// PluginBezier.cpp
// giy.hands@gmail.com (giy.hands@gmail.com)

#include "SC_PlugIn.hpp"
#include "Bezier.hpp"

static InterfaceTable* ft;

namespace Bezier {

Bezier::Bezier() {
    res = in0(0);
    ctrlX = in0(1);
    ctrlY = in0(2);
    std::cout << "res: " << res << " / ctrlX: " << ctrlX << " / ctrlY: " << ctrlY << std::endl;
    
    startPoint = Vector2(0, 0);
    endPoint = Vector2(res, 0);
    controlPoint = Vector2(ctrlX, ctrlY);
    
    std::cout << controlPoint.getX() << "/ " << controlPoint.getY() << std::endl;
    
    // [참고] https://doc.sccode.org/Guides/WritingUGens.html
    // you'll need to define unit in order to use ClearUnitIfMemFailed
    Unit* unit = (Unit*) this;
    
    // 2. Allocate memory
    mCurveX = (float*) RTAlloc(unit->mWorld, res * sizeof(float));
    mCurveY = (float*) RTAlloc(unit->mWorld, res * sizeof(float));
    mArcLengths = (float*) RTAlloc(unit->mWorld, (res + 1) * sizeof(float));
    
    // 3. Clear unit if any allocation failed
    ClearUnitIfMemFailed(mCurveX && mCurveY && mArcLengths);
    
    // 4. Feel free to access memory now
    // 0 으로 초기화
    memset(mCurveX, 0, res * sizeof(float));
    memset(mCurveY, 0, res * sizeof(float));
    memset(mArcLengths, 0, (res + 1) * sizeof(float));
    
    
    makeCurveTable();
//    for (int i = 0; i < res; i += 1)
//    {
//        std::cout << "mArcLengths[" << i << "]: " << mArcLengths[i] << std::endl;
//    }


    mEquiDistPoints = equidistantPoints(res);
    
//    for (int i = 0; i < res; i += 1)
//    {
//        std::cout <<"mEquiDistPoints[" << i << "]: " << mEquiDistPoints[i].getX() << " / " << mEquiDistPoints[i].getY() << std::endl;
//    }

    
    mCalcFunc = make_calc_function<Bezier, &Bezier::next>();
    next(1);
}


Bezier::~Bezier()
{
    // 5. Free your allocated memory
    RTFree(mWorld, mCurveX);
    RTFree(mWorld, mCurveY);
    RTFree(mWorld, mArcLengths);
}



//https://github.com/processing/processing-docs/tree/master/content/examples/Topics/Curves/ArcLengthParametrization
void Bezier::makeCurveTable()
{
    // we will keep current length along the curve here
    arcLength = 0;
    
    Vector2 prev = startPoint;
    
    // i goes from 0 to res
    for (int i = 0; i <= res; i++) {
        
        // map index from range (0, res) to parameter in range (0.0, 1.0)
        float t = (float) i / res;

        // get point on the curve at this parameter value
        Vector2 point = pointAtParameter(t);

        // get distance from previous point
        Vector2 diff = point - prev;
        diff = Vector2(diff.getX(), diff.getY());
        float distanceFromPrev = diff.getMag();

        // add arc length of last segment to total length
        arcLength += distanceFromPrev;
        
        // save current arc length to the look up table
        mArcLengths[i] = arcLength;

        // keep this point to compute length of next segment
        prev = point;
    }
    
    // Here we have sum of all segment lengths, which should be
    // very close to the actual length of the curve. The more
    // segments we use, the more accurate it becomes.
    curveLength = arcLength;
}
  

// Returns an array of equidistant point on the curve
Vector2* Bezier::equidistantPoints(int howMany)
{
     // you'll need to define unit in order to use ClearUnitIfMemFailed
    Unit* unit = (Unit*) this;
    
    // 2. Allocate memory
    Vector2* resultPoints  = (Vector2*) RTAlloc(unit->mWorld, howMany* sizeof(Vector2));
    
    // 3. Clear unit if any allocation failed
    ClearUnitIfMemFailed(resultPoints);
    
    // 4. Feel free to access memory now
    // 0 으로 초기화
    memset(resultPoints, 0, howMany * sizeof(Vector2));
  
    // we already know the beginning and the end of the curve
    resultPoints[0] = startPoint;
    resultPoints[howMany - 1] = endPoint;

    int arcLengthIndex = 1;
    for (int i = 1; i < howMany - 1; i++) {

        // compute wanted arc length
        float fraction = (float) i / (howMany - 1);
        float wantedLength = fraction * curveLength;

        // move through the look up table until we find greater length
        while (wantedLength > mArcLengths[arcLengthIndex] && arcLengthIndex < res + 1) {
          arcLengthIndex++;
        }

        // interpolate two surrounding indexes
        int nextIndex = arcLengthIndex;
        int prevIndex = arcLengthIndex - 1;
        float prevLength = mArcLengths[prevIndex];
        float nextLength = mArcLengths[nextIndex];
        float mappedIndex = map(wantedLength, prevLength, nextLength, prevIndex, nextIndex);

        // map index from range (0, SEGMENT_COUNT) to parameter in range (0.0, 1.0)
        float parameter = mappedIndex / res;

        resultPoints[i] = pointAtParameter(parameter);
        
        mCurveY[i] = resultPoints[i].getY();
    }

    return resultPoints;
}


Vector2 Bezier::pointAtParameter(float t) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;

    Vector2 point;
    float x = uu * startPoint.getX() + 2 * u * t * controlPoint.getX() + tt * endPoint.getX();
    float y = uu * startPoint.getY() + 2 * u * t * controlPoint.getY() + tt * endPoint.getY();
    point = Vector2(x, y);

//    std::cout << "pointAtParameter(): " << point.getX() << " / " << point.getY() << std::endl;
    return point;
}


void Bezier::makeCurve(Vector2 cp)
{
    std::cout << "makeCurve()" << std::endl;
    float amt = 0.f;
    
    for (int i = 0; i < res; i++) {
        amt = i / (float)res; // int / float => float

        Vector2 p0 = startPoint;
        Vector2 p1 = controlPoint;
        Vector2 p2 = endPoint;

        Vector2 q0 = p0 + Vector2::scalarMultiplication(amt, (p1 - p0));
        Vector2 q1 = p1 + Vector2::scalarMultiplication(amt, (p2 - p1));

        Vector2 r = q0 + Vector2::scalarMultiplication(amt, (q1 - q0));


        mCurveX[i] = r.getX();
        mCurveY[i] = r.getY();
    }
}



void Bezier::next(int nSamples) {

    // Output buffer
    float* outbuf = out(0);
    
    // copy bezier result to outbuf by nSamples at a time.
    // faster than for loop
    size_t len = mCount * nSamples;
    std::copy(mCurveY + len, mCurveY + len + nSamples, outbuf);

    mCount++;
    if (mCount * nSamples >= res) mCount = 0;
}


// Processing의 map()과 유사한 함수
double Bezier::map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return std::clamp((value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow, std::min(toLow, toHigh), std::max(toLow, toHigh));
}


} // namespace Bezier

PluginLoad(BezierUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<Bezier::Bezier>(ft, "Bezier", false);
}

