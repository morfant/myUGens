// PluginBezier.cpp
// giy.hands@gmail.com (giy.hands@gmail.com)

#include "SC_PlugIn.hpp"
#include "Bezier.hpp"
//#define DEBUG

static InterfaceTable* ft;

namespace Bezier {

Bezier::Bezier() {
    
    RES_OVER_SR = RESOLUTION / sampleRate();
    HALF_RES = RESOLUTION / 2;
    freq = in0(0);
    ctrlX = in0(1) * HALF_RES; // 0 ~ 1.0 ==> 0 ~ RESOLUTION / 2.0
    ctrlY = 2.0;
//    std::cout << "RES_OVER_SR: " << RES_OVER_SR << " / freq: " << freq << " / ctrlX: " << ctrlX << " / ctrlY: " << ctrlY << std::endl;
    
    mFreq = freq;
    mCtrlX = ctrlX;
    
    startPoint = Vector2(0, 0);
    endPoint = Vector2(RESOLUTION, 0);
    controlPoint = Vector2(mCtrlX, ctrlY);
    
    // [참고] https://doc.sccode.org/Guides/WritingUGens.html
    // you'll need to define unit in order to use ClearUnitIfMemFailed
    Unit* unit = (Unit*) this;
    
    // 2. Allocate memory
//    mCurveX = (float*) RTAlloc(unit->mWorld, RESOLUTION * sizeof(float));
    mCurveY = (float*) RTAlloc(unit->mWorld, RESOLUTION * sizeof(float));
    mArcLengths = (float*) RTAlloc(unit->mWorld, (RESOLUTION + 1) * sizeof(float));
    
    // 3. Clear unit if any allocation failed
    ClearUnitIfMemFailed(mCurveY && mArcLengths);
    
    // 4. Feel free to access memory now
    // 0 으로 초기화
//    memset(mCurveX, 0, RESOLUTION * sizeof(float));
    memset(mCurveY, 0, RESOLUTION * sizeof(float));
    memset(mArcLengths, 0, (RESOLUTION + 1) * sizeof(float));
    
    
    // curve 위의 점들 간 거리를 모두 더한 테이블(배열)을 만든다
    makeCurveTable(HALF_RES);

    // 베지어 커브는 '비균일'
    // 그러나 우리는 샘플간 간격(말하자면 x축 간격)이 일정한 상황에서 그에 상응하는 Y 값이 필요하다
    // 이를 위해 그래프가 만들어 내는 거리를 균등한 간격으로 분할하는 x값을 찾는 과정
    // 실제로는 mCurveY 배열에 균등한 간격의 x 값이 적용된 y 값을 넣어 둔다
    equidistantPoints(HALF_RES);

    // RESOLUTION / 2 크기로 양수 베지어 곡선을 그리고, 버퍼의 나머지 반에는 음수로 뒤집혀 대칭되는 값들을 넣는다
    // copy positive as negative second half
    int centerIdx = HALF_RES;
    for (size_t i = 0; i < centerIdx; i++)
    {
        mCurveY[centerIdx + i] = -1.0 * mCurveY[centerIdx - i];
    }
       
    mCalcFunc = make_calc_function<Bezier, &Bezier::next>();
    next(1);
}


Bezier::~Bezier()
{
    // 5. Free your allocated memory
//    RTFree(mWorld, mCurveX);
    RTFree(mWorld, mCurveY);
    RTFree(mWorld, mArcLengths);
}



//https://github.com/processing/processing-docs/tree/master/content/examples/Topics/Curves/ArcLengthParametrization
void Bezier::makeCurveTable(int res)
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
        // arcLength에는 베지어 곡선을 이루는 점과 점 사이의 거리들이 누적된다
        arcLength += distanceFromPrev;
        
        // save current arc length to the look up table
        // mArcLengths 배열에는 누적된 거리들이 저장된다
        mArcLengths[i] = arcLength;

        // keep this point to compute length of next segment
        prev = point;
    }
    
    // Here we have sum of all segment lengths, which should be
    // very close to the actual length of the curve. The more
    // segments we use, the more accurate it becomes.
    curveLength = arcLength;
}
  

// Make an array of equidistant point on the curve
void Bezier::equidistantPoints(int howMany)
{

    int arcLengthIndex = 1;
    for (int i = 1; i < howMany - 1; i++) {

        // compute wanted arc length
        float fraction = (float) i / (howMany - 1);
        float wantedLength = fraction * curveLength;

        // move through the look up table until we find greater length
        while (wantedLength > mArcLengths[arcLengthIndex] && arcLengthIndex < HALF_RES + 1) {
          arcLengthIndex++;
        }
            // interpolate two surrounding indexes
        int nextIndex = arcLengthIndex;
        int prevIndex = arcLengthIndex - 1;
       
        float prevLength = mArcLengths[prevIndex];
        float nextLength = mArcLengths[nextIndex];
        float mappedIndex = map(wantedLength, prevLength, nextLength, prevIndex, nextIndex);
        
        // map index from range (0, SEGMENT_COUNT) to parameter in range (0.0, 1.0)
        float parameter = mappedIndex / HALF_RES;
        
#ifdef DEBUG
        std::cout << "fraction(" << i << "): " << fraction << std::endl;
        std::cout << "wantedLength (" << i << "): " << wantedLength << std::endl;
        std::cout << "arcLengthIndex(" << i << "): " << arcLengthIndex << std::endl;
        std::cout << "prevIndex: " << prevIndex << std::endl;
        std::cout << "nextIndex: " << nextIndex << std::endl;
        std::cout << "prevLength: " << prevLength << std::endl;
        std::cout << "nextLength: " << nextLength << std::endl;
        std::cout << "parameter: " << parameter << std::endl;
#endif
        mCurveY[i] = pointAtParameter(parameter).getY();
    }
}


// 수식에 대한 설명
// https://lee-seokhyun.gitbook.io/game-programming/client/easy-mathematics/gdc2012/2
Vector2 Bezier::pointAtParameter(float t) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;

    Vector2 point;

    float x = uu * startPoint.getX() + 2 * u * t * controlPoint.getX() + tt * (endPoint.getX()/2.0);
    float y = uu * startPoint.getY() + 2 * u * t * controlPoint.getY() + tt * endPoint.getY();
    point = Vector2(x, y);

    return point;
}


//Vector2 Bezier::pointAtParameter(float amt)
//{
//    std::cout << "makeCurve()" << std::endl;
////    float amt = 0.f;
//
////    for (int i = 0; i < RESOLUTION; i++) {
////        amt = i / (float)RESOLUTION; // int / float => float
//
//        Vector2 p0 = startPoint;
//        Vector2 p1 = controlPoint;
//        Vector2 p2 = Vector2(endPoint.getX()/2, endPoint.getY());
//
//        Vector2 q0 = p0 + Vector2::scalarMultiplication(amt, (p1 - p0));
//        Vector2 q1 = p1 + Vector2::scalarMultiplication(amt, (p2 - p1));
//
//        Vector2 r = q0 + Vector2::scalarMultiplication(amt, (q1 - q0));
//
////        mCurveX[i] = r.getX();
////        mCurveY[i] = r.getY();
//    return r;
////    }
//}
//


void Bezier::next(int nSamples) {
    
    // Output buffer
    float* outbuf = out(0);
    
    freq = in0(0);
    ctrlX = in0(1) * HALF_RES; // 0 ~ 1.0 ==> 0 ~ RESOLUTION / 2.0
   
    if (freq != mFreq)
    {
        mFreq = freq;
    }
    
    if (ctrlX != mCtrlX)
    {
        mCtrlX = ctrlX;
        
        // 베지어 곡선을 다시 만들어 준다
        controlPoint = Vector2(mCtrlX, ctrlY);
        makeCurveTable(HALF_RES);
        equidistantPoints(HALF_RES);
        int centerIdx = HALF_RES;
        for (size_t i = 0; i < centerIdx; i++)
        {
            mCurveY[centerIdx + i] = -1.0 * mCurveY[centerIdx - i];
        }
    }
    
    // using for-loop way
    // outbuf에 주파수에 따라 다른 간격으로 mCurveY 배열에 있는 값을 넣는 과정
    for (size_t i = 0; i < nSamples; i++)
    {
        outbuf[i] = mCurveY[(int)curIdxPhase];
        curIdxPhase += (mFreq * RES_OVER_SR);
        
        if (curIdxPhase >= RESOLUTION) {
            curIdxPhase = curIdxPhase - RESOLUTION - 1;
        }
    }
 
    
    // 단순하게 차례대로 있는 값들을 output으로 복사할 경우에는
    // std::copy 메소드를 쓰는 것이 for-loop으로 처리하는 것보다 더 빠르다.
    // copy bezier result to outbuf by nSamples at a time.
//    size_t len = mCount * nSamples;
//    std::copy(mCurveY + len, mCurveY + len + nSamples, outbuf);
    
//    mCount++;
//    if (mCount * nSamples >= RESOLUTION) mCount = 0;
}


// Processing의 map(), Max의 scale과 유사한 함수
double Bezier::map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return std::clamp((value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow, std::min(toLow, toHigh), std::max(toLow, toHigh));
}


} // namespace Bezier

PluginLoad(BezierUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<Bezier::Bezier>(ft, "Bezier", false);
}

