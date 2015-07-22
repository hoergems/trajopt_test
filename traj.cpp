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
    RaveInitialize(false, verbose_ ? Level_Debug : Level_Info);
    env_ = EnvironmentBasePtr(RaveCreateEnvironment());
    env_->Load(env_path_);
    env_->Load(robot_path_);
    robot_ = RobotBasePtr(GetRobot(*env_));
    CollisionChecker::GetOrCreate(*env_)->SetContactDistance(0.25);
}

void TrajTest::setup(std::vector<double> &start_state,
                     std::vector<double> &goal_state) {
    vector<int> robotDofInds;
    for (int k=0; k < n_dof_; k++) 
    {
        robotDofInds.push_back(k);
    }
    rad_ = RobotAndDOFPtr(new RobotAndDOF(robot_, robotDofInds));        
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

    RaveDestroy();
    return 0;
}
