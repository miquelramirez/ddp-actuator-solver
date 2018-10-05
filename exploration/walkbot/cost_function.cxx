#include "cost_function.hxx"

WalkbotCostFunction::WalkbotCostFunction()
{
    Q <<    1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    R << 0.0, 0.0,
        0.0, 0.0;

    lxx = Q;
    luu = R;
    lux <<  0.0,0.0,
            0.0,0.0,
            0.0,0.0,
            0.0,0.0;
    lxu <<  0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0;
    lx.setZero();
}

void WalkbotCostFunction::computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U)
{
    lx = Q * (X - Xdes);
    lu = R * U;
}

void WalkbotCostFunction::computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes)
{
    lx = 1.0 * Q * (X - Xdes);
    lxx = 1.0 * Q;
}