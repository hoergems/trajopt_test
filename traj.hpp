#include "osgviewer/osgviewer.hpp"
#include <iostream>
#include <string>
#include "sco/optimizers.hpp"
#include "sco/solver_interface.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/sco_common.hpp"
#include "sco/sco_fwd.hpp"
#include "sco/num_diff.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "trajopt/plot_callback.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/traj_plotter.hpp"
#include "utils/config.hpp"
#include "utils/eigen_conversions.hpp"
#include <openrave-core.h>
#include <openrave/openrave.h>
#include <cmath>
#include <boost/timer.hpp>

using namespace std;
using namespace trajopt;
using namespace util;
using namespace OpenRAVE;
using std::string;

namespace traj_test {

class TrajTest {
    public:
        TrajTest(bool plotting, 
                 bool verbose, 
                 int n_dof,
                 int num_step_,
                 double control_rate,
                 double delta_t,
                 double max_joint_velocity,
                 string env_path,
                 string robot_path);

        void setup(std::vector<double> &start_state,
                   std::vector<double> &goal_state);
  
        void planPath();
        ~TrajTest() = default;

    private:
        bool plotting_;

        bool verbose_;

        int n_dof_;   

        int num_steps_; 

        double control_rate_;

        double delta_t_;

        double max_joint_velocity_; 

        double max_dist_;

        string env_path_;

        string robot_path_;

        EnvironmentBasePtr env_;  

        RobotBasePtr robot_; 

        OptProbPtr prob_;

        RobotAndDOFPtr rad_; 

        Vector3d start_;
 
        Vector3d goal_; 

        VarArray trajvars_;        
    
};

}
