#pragma once

#include <ddp-actuator-solver/costfunction.hh>

class WalkbotCostFunction : public CostFunction<double, 4, 2>
{
public:
    WalkbotCostFunction();

    // Methods
    void computeAllCostDeriv(   const stateVec_t& X,
                                const stateVec_t& Xdes,
                                const commandVec_t& U);
    void computeFinalCostDeriv( const stateVec_t& X,
                                const stateVec_t& Xdes);

    // Properties
    stateMat_t      Q; // Q-component of cost function
    commandMat_t    R; // R-component of cost function
    double          dt; // time step
};
