#pragma once

#include <ddp-actuator-solver/dynamicmodel.hh>

class Walkbot : public DynamicModel<double, 4, 2>
{

public:

    Walkbot();
    virtual ~Walkbot();

    // attributes
    // State matrix: A \in R^{dxd}
    stateMat_t          A;
    // Input matrix: B \in R^{dxp}
    stateR_commandC_t   B;
private:
    stateMat_t          _QxxCont;
    commandMat_t        _QuuCont;
    commandR_stateC_t   _QuxCont;

    stateVec_t _dX; // derivatives
    stateVec_t _nextX, _k1, _k2, _k3, _k4;

public:
    stateVec_t          computeDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateVec_t          computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
    void                computeAllModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateMat_t          computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t        computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t   computeTensorContux(const stateVec_t& nextVx);
};
