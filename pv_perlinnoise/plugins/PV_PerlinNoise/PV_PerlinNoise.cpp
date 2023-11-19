// PluginPV_PerlinNoise.cpp
// giy (giy.hands@gmail.com)

#include "PV_PerlinNoise.hpp"

//static InterfaceTable* ft;

namespace PV_PerlinNoise {

PV_PerlinNoise::PV_PerlinNoise() {
    init();
    
    mCalcFunc = make_calc_function<PV_PerlinNoise, &PV_PerlinNoise::next>();
    next(1);
}

void PV_PerlinNoise::init() {
    for (int i = 0; i < 512; i++) {
        mPermutation[i] = mP[i % 256];
    }
}

int PV_PerlinNoise::fastfloor(double x) {
    return x > 0 ? (int)x : (int)(x - 1);
}

double PV_PerlinNoise::dot(const double* arr, double x, double y) {
    return arr[0] * x + arr[1] * y;
}

double PV_PerlinNoise::easing(double t) {
    // 6t5-15t4+10t3
    return t * t * t * (t * (t * 6 - 15) + 10);
}

double PV_PerlinNoise::lerp(double a, double b, double w) {
    return a + (b - a) * w;
}

double PV_PerlinNoise::CoherentNoiseGradient2D(double x, double y) {
    int intX = fastfloor(x);
    int intY = fastfloor(y);

    double fracX = x - intX;
    double fracY = y - intY;

    intX = intX & 255; // ==> % 256
    intY = intY & 255;

    // select mGradients with mPermutation
    int gradIdx00 = (mPermutation[intX + mPermutation[intY]] + (mSeed & 255)) & 3;
    int gradIdx10 = (mPermutation[intX + 1 + mPermutation[intY]] + (mSeed & 255)) & 3;
    int gradIdx01 = (mPermutation[intX + mPermutation[intY + 1]] + (mSeed & 255)) & 3;
    int gradIdx11 = (mPermutation[intX + 1 + mPermutation[intY + 1]] + (mSeed & 255)) & 3;

    // 영향력 구하기(gradient vector와 dist vector의 내적(dot))
    // A dot B ==> A.x * B.x + A.y + B.y
    // dist vector의 크기에 따라 각 지점의 gradient 벡터의 영향력을 결정
    double gradDotDist00 = dot(mGradients[gradIdx00], fracX - 0, fracY - 0);
    double gradDotDist10 = dot(mGradients[gradIdx10], fracX - 1, fracY - 0);
    double gradDotDist01 = dot(mGradients[gradIdx01], fracX - 0, fracY - 1);
    double gradDotDist11 = dot(mGradients[gradIdx11], fracX - 1, fracY - 1);

    double u = easing(fracX);
    double v = easing(fracY);

    double x1 = lerp(gradDotDist00, gradDotDist10, u);
    double x2 = lerp(gradDotDist01, gradDotDist11, u);
    double rslt = lerp(x1, x2, v);

    return rslt;
}

double PV_PerlinNoise::perlin2D(double x, double y, int octaves, double persistence) {
    double total = 0.0;
    double frequency = 1.0;
    double amplitude = 1.0;
    double maxValue = 0.0;
    
    for(int i = 0; i < octaves; i++) {
        total += CoherentNoiseGradient2D(x * frequency, y * frequency) * amplitude;
        frequency *= 2;
        maxValue += amplitude;
        amplitude *= persistence;
    }
    return total/maxValue;
}


void PV_PerlinNoise::next(int nSamples) {
    Unit* unit = (Unit*)this;
    PV_GET_BUF // get buf (chain)
    
    mMagScale = in0(1);
    mStepScale = in0(2);
    mOctave = in0(3);
    mPersistence = in0(4);
    mSeed = in0(5);
    
    SCPolarBuf * p = ToPolarApx(buf);

    double rslt[numbins];
    
    p->dc = 0.f;
    p->nyq = 0.f;

    for (int i = 0; i < numbins; i++) {
        rslt[i] = abs(perlin2D(mOffset * mStepScale, i * mStepScale, mOctave, mPersistence));
        
        // WARNING!! CPU Peak!!!
//        std::cout << "perlin2D result(" << i << "):" << rslt[i] << std::endl;
        
    }
    
    for (int i = 0; i < numbins; i++) {
        p->bin[i].mag = mMagScale * rslt[i];
        p->bin[i].phase = fmod(p->bin[i].phase * rslt[i], twopi);
    }
    
    mOffset++;
    mOffset = mOffset & ((1 << (sizeof(uint32) * 8 - 1)) - 1);

}

} // namespace PV_PerlinNoise

PluginLoad(PV_PerlinNoiseUGens) {
    // Plugin magic
    InterfaceTable* ft = inTable;
    registerUnit<PV_PerlinNoise::PV_PerlinNoise>(ft, "PV_PerlinNoise", false);
}
