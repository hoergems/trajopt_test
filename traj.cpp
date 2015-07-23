#include "traj.hpp"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

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
    CollisionChecker::GetOrCreate(*env_)->SetContactDistance(0.15);
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
    prob_ = TrajOptProbPtr(new TrajOptProb());    
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
            prob_->addCost(CostPtr(new CollisionCost(0.0001, 20, rad_, vars0, vars1)));
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

double TrajTest::getPathLength(DblVec &trajVec) {
    double length = 0.0;
    for (int i = 0; i < (trajVec.size() / n_dof_) - 1; i++) {
        double lengthStep = 0.0;
        for (int j = 0; j < n_dof_; j++) {
            lengthStep += pow(trajVec[(i + 1) * n_dof_ + j] - trajVec[i * n_dof_ + j], 2);
        }
        length += sqrt(lengthStep);
    }

    return length;
}

double TrajTest::getPathLength(std::vector<std::vector<double> > &trajVec) {
    double length = 0.0;
    for (int i = 0; i < trajVec.size() - 1; i++) {
        double lengthStep = 0.0;
        for (size_t j = 0; j < trajVec[i].size(); j++) {
            lengthStep += pow(trajVec[i+1][j] - trajVec[i][j], 2);
        }
        length += sqrt(lengthStep);
    }
    return length;
}

std::vector<std::vector<double> > TrajTest::toVector(DblVec &trajVec) {
    vector<vector<double>> trajectory;
    for (int i=0; i < num_steps_; i++) {
        vector<double> col;
        for (int j=0; j < n_dof_; j++) {
            col.push_back(trajVec[i * n_dof_ + j]);   
        }
        trajectory.push_back(col);
    }
    return trajectory;
}

DblVec TrajTest::planPath() {   

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
    opt.optimize();
    return opt.x();    
}

void TrajTest::loadObstaclesXML(std::vector<std::shared_ptr<Obstacle> > *obst, 
                                std::string &obstacles_path) {    
    //read_xml(is, pt);

    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(obstacles_path.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << obstacles_path << endl;
        //return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string filename = std::string(dirp->d_name);
        if (!filename.find(".") == 0 && 
            filename.find("~") > filename.length()) {
                cout << "opening file " << filename << endl;
                ptree pt;
                std::ifstream fin;   
                fin.open(obstacles_path + "/" + filename);
                read_xml(fin, pt);
                BOOST_FOREACH( ptree::value_type &v, pt.get_child("Environment") ) {
                    if(v.first == "KinBody") { 
                        //get_child("<xmlattr>.type").data();
                        cout << v.second.get_child("<xmlattr>.name").data() << endl;                        
                        boost::property_tree::ptree subtree = (boost::property_tree::ptree)v.second;
                        BOOST_FOREACH(boost::property_tree::ptree::value_type &vs, subtree) {
                            if (vs.first == "Body") {
                                boost::property_tree::ptree subtree2 = (boost::property_tree::ptree)vs.second;
                                BOOST_FOREACH(boost::property_tree::ptree::value_type &vss, subtree2) {
                                    cout << vss.first << endl;
                                    if (vss.first == "Geom") {                                        
                                        std::string trans = vss.second.get<string>("Translation");                                        
                                    }
                                }   
                            }
                        }
                    }
                }
             }
         
    }
}

}

int main(int argc, char** argv) {    
    traj_test::TrajTest tt(false,
                           false,
                           3,
                           45,
                           30.0,
                           1.0 / 30.0,
                           2.0,
                           "environment/env.xml",
                           "model/model_test.xml");

    std::vector<std::shared_ptr<traj_test::Obstacle> > *obst;
    std::string obstaclesPath("/home/hoe01h/trajopt_test/environment");

    tt.loadObstaclesXML(obst, obstaclesPath);

    /**std::vector<double> start_state({0.0, 0.0, 0.0});
    std::vector<double> goal_state({-M_PI / 2.0, 0.0, 0.0});

    boost::timer timer;
    tt.setup(start_state, goal_state);
    tt.planPath();
    LOG_FATAL("elapsed: %s", CSTR(timer.elapsed())); 
    
    boost::timer timer2;
    tt.setup(start_state, goal_state);
    DblVec trajectory1 = tt.planPath();
    LOG_FATAL("elapsed: %s", CSTR(timer2.elapsed())); 

    start_state[0] = 0.01;
    boost::timer timer3;
    tt.setup(start_state, goal_state);
    DblVec trajectory2 = tt.planPath();
    LOG_FATAL("elapsed: %s", CSTR(timer3.elapsed()));

    boost::timer timer4;
    tt.setup(start_state, goal_state);
    tt.planPath();
    LOG_FATAL("elapsed: %s", CSTR(timer4.elapsed()));    
    LOG_INFO("length1: %s", CSTR(10000.0 / tt.getPathLength(trajectory1)));
    LOG_INFO("length2: %s", CSTR(10000.0 / tt.getPathLength(trajectory2)));
    
    */
    RaveDestroy();
    return 0;
}
