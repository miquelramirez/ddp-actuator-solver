#include "dynamic_model.hxx"

Walkbot::Walkbot() {
    stateNb = 4;
    commandNb = 2;

    // State matrix coefficients
    A << 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0;

    B  << 0.0, 0.0,
        0.0, 0.0,
        1.0, 0.0,
        0.0, 1.0;

    // control bounds
    lowerCommandBounds << -1.0, -1.0;
    upperCommandBounds << 1.0, 1.0;

    fx = A;
    fu = B;


    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    // Cost tensors
    _QxxCont.setZero();
    _QuuCont.setZero();
    _QuxCont.setZero();
}

Walkbot::~Walkbot() {

}

Walkbot::stateVec_t
Walkbot::computeDeriv( double& dt,
                        const stateVec_t& X,
                        const commandVec_t &U) {

    _dX = A * X + B * U;
    return _dX;
}

Walkbot::stateVec_t
Walkbot::computeNextState(double& dt,
					    const stateVec_t& X,
					    const commandVec_t& U)
{
    // RK(4,5)
    _k1 = computeDeriv(dt, X, U);
    _k2 = computeDeriv(dt, X + (dt/2) * _k1, U);
    _k3 = computeDeriv(dt, X + (dt/2) * _k2, U);
    _k4 = computeDeriv(dt, X + dt*_k3, U);
    _nextX = X + (dt / 6)*(_k1 + 2*_k2 + 2*_k3 + _k4);
    return _nextX;
}

void Walkbot::computeAllModelDeriv(double& dt,
				  const stateVec_t& X,
				  const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(unsigned int i=0; i < stateNb; i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U))/dh;
        Xp = X;
        Xm = X;
    }
}

Walkbot::stateMat_t Walkbot::computeTensorContxx(const stateVec_t& )
{
  return _QxxCont;
}

Walkbot::commandMat_t Walkbot::computeTensorContuu(const stateVec_t& )
{
  return _QuuCont;
}

Walkbot::commandR_stateC_t Walkbot::computeTensorContux(const stateVec_t& )
{
  return _QuxCont;
}
