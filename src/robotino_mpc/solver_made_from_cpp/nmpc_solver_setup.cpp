/*
 *    file:   nmpc_solver_setup.cpp
 *    author: Oskar Ljungqvist
 *    date:   2017-12-21
 *
 *    Comment: modified version of the nmpc_solver_setup.m works directly in ubutu.
 */

#include <acado_code_generation.hpp>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <matrix_vector/vector.hpp>

//#include <acado_optimal_control.hpp>
//#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main() {

    double Ts = 0.1;  // prediction sampling time
    double N = 20;   // Prediction horizon
    double PI = 3.1415926535897932;

    DifferentialState p_x; //x_w
    DifferentialState p_y; //y_w
    DifferentialState yaw_robot;
    DifferentialState yaw_camera; // yaw total
    DifferentialState dummy;

    Control velocity_x;
    Control velocity_y;
    Control velocity_yaw_robot;
    Control velocity_yaw_camera;

    int NUM_OBS = 10;
    OnlineData obstacle_xs[NUM_OBS];
    OnlineData obstacle_ys[NUM_OBS];

    // Model equations:
    DifferentialEquation f(0,N);

    f << dot(p_x) == (velocity_x) * cos(yaw_robot) - (velocity_y) * sin(yaw_robot);
    f << dot(p_y) == (velocity_x) * sin(yaw_robot) + (velocity_y) * cos(yaw_robot);
    f << dot(yaw_robot) == velocity_yaw_robot;
    f << dot(yaw_camera) == velocity_yaw_camera + velocity_yaw_robot;
    f << dot(dummy) == obstacle_xs[0] + obstacle_xs[1] + obstacle_xs[2] + obstacle_xs[3] + obstacle_xs[4] +
                       obstacle_xs[5] + obstacle_xs[6] + obstacle_xs[7] + obstacle_xs[8] + obstacle_xs[9] +
                       obstacle_ys[0] + obstacle_ys[1] + obstacle_ys[2] + obstacle_ys[3] + obstacle_ys[4] +
                       obstacle_ys[5] + obstacle_ys[6] + obstacle_ys[7] + obstacle_ys[8] + obstacle_ys[9];

    IntermediateState dist[NUM_OBS];
    for (int i = 0; i < NUM_OBS; i++) {
        dist[i] = sqrt((obstacle_xs[i] - p_x) * (obstacle_xs[i] - p_x) +
                       (obstacle_ys[i] - p_y) * (obstacle_ys[i] - p_y) + 0.00001);
    }

    IntermediateState v_tr;
    IntermediateState v_rot;
    v_tr = sqrt(0.00001 + (velocity_x * velocity_x) + (velocity_y * velocity_y))*100;
    v_rot = sqrt((0.00001 + velocity_yaw_robot * velocity_yaw_robot)) + sqrt((0.00001 + velocity_yaw_camera * velocity_yaw_camera));

    // Reference functions and weighting matrices:
    // minimize inputs to the robot
    Function h;
    h << velocity_x << velocity_y << velocity_yaw_robot << velocity_yaw_camera;
    h << p_x << p_y;
    h << yaw_camera;
    h << -(1 - exp(0.1 / (dist[0] * dist[0]))) // with dist w/o sqrt
      << -(1 - exp(0.1 / (dist[1] * dist[1])))
      << -(1 - exp(0.1 / (dist[2] * dist[2])))
      << -(1 - exp(0.1 / (dist[3] * dist[3])))
      << -(1 - exp(0.1 / (dist[4] * dist[4])))
      << -(1 - exp(0.1 / (dist[5] * dist[5])))
      << -(1 - exp(0.1 / (dist[6] * dist[6])))
      << -(1 - exp(0.1 / (dist[7] * dist[7])))
      << -(1 - exp(0.1 / (dist[8] * dist[8])))
      << -(1 - exp(0.1 / (dist[9] * dist[9])));

    // Ok -- yN verrà dato in cpp ed è la posizione finale. Questo mi da la penalizzazione sulla posizione finale
    Function hN;
    hN << p_x << p_y;
    hN << yaw_camera;

    // Provide defined weighting matrices:
    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    // Define OCP problem:
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    // Dummy constraints, real ones set online
    ocp.subjectTo(0 <= v_tr <= 0.5); // translational speed
    ocp.subjectTo(0 <= v_rot <= 1*100); // total rotation

    ocp.subjectTo(-0.1 <= velocity_x <= 0.1);
    ocp.subjectTo(-0.1 <= velocity_y <= 0.1);
    ocp.subjectTo(-45 * PI / 180 <= velocity_yaw_robot <= 45 * PI / 180);
    ocp.subjectTo(-45 * PI / 180 <= velocity_yaw_camera <= 45 * PI / 180);

    // todo evaluate if add translational + total rotational velocities

    ocp.subjectTo(dist[0] >= 0.5);
    ocp.subjectTo(dist[1] >= 0.5);
    ocp.subjectTo(dist[2] >= 0.5);
    ocp.subjectTo(dist[3] >= 0.5);
    ocp.subjectTo(dist[4] >= 0.5);
    ocp.subjectTo(dist[5] >= 0.5);
    ocp.subjectTo(dist[6] >= 0.5);
    ocp.subjectTo(dist[7] >= 0.5);
    ocp.subjectTo(dist[8] >= 0.5);
    ocp.subjectTo(dist[9] >= 0.5);

    // Export the code:
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); //FULL_CONDENsinG_N2
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL2);
//    mpc.set( NUM_INTEGRATOR_STEPS, N);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, NO);
    mpc.set(LEVENBERG_MARQUARDT, 1e-10);
    mpc.set(LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set(IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    mpc.set(CG_USE_OPENMP, YES);
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

    if (mpc.exportCode(".") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
