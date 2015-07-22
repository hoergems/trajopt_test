#include "traj.hpp"

using namespace trajopt;
using namespace std;
using namespace util;
using namespace OpenRAVE;

using std::cout;
using std::endl;

namespace traj_test {



TrajTest::TrajTest() {

}

}

int main(int argc, char** argv) {
    traj_test::TrajTest();
    bool plotting=true;
    bool verbose=true;
    string envfile;
    int decimation;
    int n_dof = 3;
    int n_steps = 50; // Length of the trajectory
    const double control_rate = 30.0;
    const double delta_t = 1.0 / control_rate;
    const double max_joint_velocity = 2.0;
    const double max_dist = delta_t * max_joint_velocity;

    {
      Config config;
      config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
      config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
      config.add(new Parameter<string>("envfile", &envfile, "env.xml"));
      config.add(new Parameter<int>("decimation", &decimation, "plot every n"));
      CommandParser parser(config);
      parser.read(argc, argv);
    }

    RaveInitialize(false, verbose ? Level_Debug : Level_Info);
    EnvironmentBasePtr env = RaveCreateEnvironment();
    env->StopSimulation();

    OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
    assert(viewer);
    env->Load("environment/env.xml");
    env->Load("model/model_test.xml");
    RobotBasePtr robot = GetRobot(*env);    
    vector<int> robotDofInds;
    for (int k=0; k < n_dof; k++) 
    {
        robotDofInds.push_back(k);
    }
     
    RobotAndDOFPtr rad(new RobotAndDOF(robot, robotDofInds));
    Vector3d start, goal;
    start = Vector3d(0, 0, 0);
    goal = Vector3d(-M_PI / 2.0, 0, 0);
    rad->SetDOFValues(toDblVec(start));

    OptProbPtr prob(new OptProb()); 
    VarArray trajvars;
    AddVarArray(*prob, n_steps, n_dof, "theta", trajvars);  

    // Velocity cost
    VectorXd vel_coeffs = VectorXd::Ones(3);
    prob->addCost(CostPtr(new JointVelCost(trajvars, vel_coeffs)));  

    // Collision cost and velocity constraints
    for (int i=0; i < n_steps-1; ++i) {
    	VarVector vars0 = trajvars.row(i); 
        VarVector vars1 = trajvars.row(i+1);
        VectorXd coeffs = VectorXd::Ones(3);
        VarVector vars = concat(vars0, vars1);
      
        // The velocity constraints
        AffExpr dist1 = trajvars(i+1,0) - trajvars(i,0);
        AffExpr dist2 = trajvars(i+1,1) - trajvars(i,1);
        AffExpr dist3 = trajvars(i+1,2) - trajvars(i,2);
        prob->addLinearConstraint(dist1 - max_dist, INEQ);
        prob->addLinearConstraint(-dist1 - max_dist, INEQ);
        prob->addLinearConstraint(dist2 - max_dist, INEQ);
        prob->addLinearConstraint(-dist2 - max_dist, INEQ);
        prob->addLinearConstraint(dist3 - max_dist, INEQ);
        prob->addLinearConstraint(-dist3 - max_dist, INEQ);
        if (i > 0) {
            prob->addCost(CostPtr(new CollisionCost(0.05, 20, rad, vars0, vars1)));
        }
    }

    // Linear constraints

    // Start and goal constraints
    for (int j=0; j < n_dof; ++j) {
        prob->addLinearConstraint(exprSub(AffExpr(trajvars(0,j)), start[j]), EQ);
    }
    for (int j=0; j < n_dof; ++j) {
        prob->addLinearConstraint(exprSub(AffExpr(trajvars(n_steps-1,j)), goal[j]), EQ);
    }

    // Optimization
    BasicTrustRegionSQP opt(prob);

    // Not entirely sure what this parameter is doing
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(0.25);

    // straight line initialization
    MatrixXd initTraj(n_steps, n_dof);  
    for (int idof = 0; idof < n_dof; ++idof) {
        initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
    }

    DblVec initVec = trajToDblVec(initTraj);
    opt.initialize(initVec);
  
    TrajPlotter plotter(env, rad, trajvars);
    plotter.Add(prob->getCosts());
    if (plotting) opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));   

    boost::timer timer;
    opt.optimize();
    cout << timer.elapsed() << " elapsed" << endl;
    
    RaveDestroy();
}
