#include "traj.hpp"



using std::cout;
using std::endl;


namespace traj_test {



TrajTest::TrajTest(bool plotting, 
                   bool verbose, 
                   int n_dof,
                   int num_steps,
                   double control_rate,
                   double delta_t,
                   double max_joint_velocity,
                   string env_path,
                   string robot_path) :
    plotting_(plotting),
    verbose_(verbose),
    n_dof_(n_dof),
    num_steps_(num_steps),
    control_rate_(control_rate),
    delta_t_(delta_t),    
    max_joint_velocity_(max_joint_velocity),
    max_dist_(delta_t_ * max_joint_velocity),
    env_path_(env_path),
    robot_path_(robot_path),
    env_(nullptr),
    robot_(nullptr),
    prob_(nullptr),    
    rad_(nullptr),
    start_(n_dof_),
    goal_(n_dof_),
    trajvars_()
    {
    
    
    
    

}

void TrajTest::setup(std::vector<double> &start_state,
                     std::vector<double> &goal_state) {
    /**Config config;
    config.add(new Parameter<bool>("plotting", &plotting_, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose_, "verbose"));
    config.add(new Parameter<string>("envfile", &envfile, "env.xml"));
    config.add(new Parameter<int>("decimation", &decimation, "plot every n"));
    CommandParser parser(config);
    parser.read(argc, argv); */ 

    RaveInitialize(false, verbose_ ? Level_Debug : Level_Info);
    env_ = EnvironmentBasePtr(RaveCreateEnvironment());
    env_->Load(env_path_);
    env_->Load(robot_path_);
    robot_ = RobotBasePtr(GetRobot(*env_));
    CollisionChecker::GetOrCreate(*env_)->SetContactDistance(0.25);

    vector<int> robotDofInds;
    for (int k=0; k < n_dof_; k++) 
    {
        robotDofInds.push_back(k);
    }
    rad_ = RobotAndDOFPtr(new RobotAndDOF(robot_, robotDofInds));
    //rad_ = boost::make_shared<RobotAndDOF>(robot_, robotDofInds)
    //rad_(RobotAndDOF(robot_, robotDofInds));    
    for (size_t k=0; k < start_state.size(); k++) {
        start_[k] = start_state[k];
        goal_[k] = goal_state[k];
    }    
    rad_->SetDOFValues(toDblVec(start_));

    prob_ = OptProbPtr(new OptProb());

    //VarArray trajvars;
    AddVarArray(*prob_, num_steps_, n_dof_, "theta", trajvars_);

    // Velocity cost
    VectorXd vel_coeffs = VectorXd::Ones(n_dof_);
    prob_->addCost(CostPtr(new JointVelCost(trajvars_, vel_coeffs)));  

    // Collision cost and velocity constraints
    for (int i=0; i < num_steps_-1; ++i) {
    	VarVector vars0 = trajvars_.row(i); 
        VarVector vars1 = trajvars_.row(i+1);
        VectorXd coeffs = VectorXd::Ones(3);
        VarVector vars = concat(vars0, vars1);
      
        // The velocity constraints
        AffExpr dist1 = trajvars_(i+1,0) - trajvars_(i,0);
        AffExpr dist2 = trajvars_(i+1,1) - trajvars_(i,1);
        AffExpr dist3 = trajvars_(i+1,2) - trajvars_(i,2);
        prob_->addLinearConstraint(dist1 - max_dist_, INEQ);
        prob_->addLinearConstraint(-dist1 - max_dist_, INEQ);
        prob_->addLinearConstraint(dist2 - max_dist_, INEQ);
        prob_->addLinearConstraint(-dist2 - max_dist_, INEQ);
        prob_->addLinearConstraint(dist3 - max_dist_, INEQ);
        prob_->addLinearConstraint(-dist3 - max_dist_, INEQ);
        if (i > 0) {
            prob_->addCost(CostPtr(new CollisionCost(0.05, 20, rad_, vars0, vars1)));
        }
    }

    // Start and goal constraints
    for (int j=0; j < n_dof_; ++j) {
        prob_->addLinearConstraint(exprSub(AffExpr(trajvars_(0,j)), start_[j]), EQ);
    }
    for (int j=0; j < n_dof_; ++j) {
        prob_->addLinearConstraint(exprSub(AffExpr(trajvars_(num_steps_-1,j)), goal_[j]), EQ);
    }    
}

void TrajTest::planPath() {   

    // Optimization
    BasicTrustRegionSQP opt(prob_);
    // straight line initialization
    MatrixXd initTraj(num_steps_, n_dof_);  
    for (int idof = 0; idof < n_dof_; ++idof) {
        initTraj.col(idof) = VectorXd::LinSpaced(num_steps_, start_[idof], goal_[idof]);
    }

    DblVec initVec = trajToDblVec(initTraj);
    opt.initialize(initVec);

    TrajPlotter plotter(env_, rad_, trajvars_);
    plotter.Add(prob_->getCosts());
    if (plotting_) {        
        opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));
    }

    boost::timer timer;
    opt.optimize();
    cout << timer.elapsed() << " elapsed" << endl;
    
    RaveDestroy();

}

}

int main(int argc, char** argv) {
    traj_test::TrajTest tt(true,
                           true,
                           3,
                           50,
                           30.0,
                           1.0 / 30.0,
                           2.0,
                           "environment/env.xml",
                           "model/model_test.xml");

    std::vector<double> start_state({0.0, 0.0, 0.0});
    std::vector<double> goal_state({-M_PI / 2.0, 0.0, 0.0});

    tt.setup(start_state, goal_state);
    tt.planPath();

    goal_state[0] = M_PI / 2.0;

    tt.setup(start_state, goal_state);

    tt.planPath();

    return 0;

    

    
    /**string envfile;
    int n_steps = 50; // Length of the trajectory
    const double control_rate = 30.0;
    const double delta_t = 1.0 / control_rate;
    const double max_joint_velocity = 2.0;
    const double max_dist = delta_t * max_joint_velocity;

    

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
    // According to the paper, this should be d_check. When the distance
    // between a pair of two convex sets is at most d_check, collision
    // is checked between them. Must be greater than the saftey margin.    
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
    
    RaveDestroy();*/

}
