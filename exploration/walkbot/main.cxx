#include <iostream>
#include <fstream>

#include <ddp-actuator-solver/ddpsolver.hh>
#include "dynamic_model.hxx"
#include "cost_function.hxx"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double tinit = 0.0;
    double texec = 0.0;
    DDPSolver<double, 4, 2>::stateVec_t x0, xDes, x;
    DDPSolver<double, 4, 2>::commandVec_t u;

    x0 << 2.0, 8.0, -1.0140, 0.2664;
    xDes << 8.0, 2.0, 0.0, 0.0;

    unsigned int T = 100;
    double dt = 0.1;
    unsigned int iterMax = 3;//100;
    double stopCrit = 1e-5;
    DDPSolver<double, 4, 2>::stateVecTab_t xList;
    DDPSolver<double, 4, 2>::commandVecTab_t uList;
    DDPSolver<double, 4, 2>::traj lastTraj;

    gettimeofday(&tbegin,NULL);
    Walkbot model;
    WalkbotCostFunction cost;
    DDPSolver<double, 4, 2> solver(model, cost, ENABLE_FULLDDP, ENABLE_QPBOX);
    solver.FirstInitSolver(x0, xDes, T, dt, iterMax, stopCrit);
    gettimeofday(&tend,NULL);

    tinit = tend.tv_sec - tbegin.tv_sec;
    tinit += tend.tv_usec/1e6 - tbegin.tv_usec/1e6;

    gettimeofday(&tbegin,NULL);
    solver.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = solver.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;
    unsigned int iter = lastTraj.iter;


    texec =tend.tv_sec - tbegin.tv_sec;
    texec += tend.tv_usec/1e6 - tbegin.tv_usec/1e6;

    cout << endl;
    cout << "Total Initialization Time ";
    cout << tinit << endl;

    cout << "Total Execution Time ";
    cout << texec << endl;

    cout << "Execution Time per iteration ";
    cout << texec/iter << endl;

    cout << "Number of Iterations" << iter << endl;


    ofstream trajectory_log("trajectory.csv", ios::out | ios::trunc);
    if(trajectory_log)
    {
        trajectory_log << "x,y,vx,vy,ax,ay" << endl;
        x = x0;
        trajectory_log << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
           << x(3, 0) << "," << uList[0](0,0) << ", " << uList[0](1,0) << endl;
        for (unsigned i = 1; i < T; i++)
        {
            x = model.computeNextState(dt, x, uList[i - 1]);
            trajectory_log << x(0, 0) << ", " << x(1, 0) << ", " << x(2, 0) << ", "
               << x(3, 0) << ", " << uList[i - 1](0,0) << ", " << uList[i-1](1,0) << endl;
        }
        trajectory_log << xList[T](0, 0) << ", " << xList[T](1, 0) << ", "
                        << xList[T](2, 0) << ", " << xList[T](3, 0) << ", "
                        << uList[T - 1](0,0) << ", " << uList[T-1](1,0) << endl;
        trajectory_log.close();
    }
    else
        cerr << "Error opening file" << endl;

    return 0;

}
