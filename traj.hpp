#include "osgviewer/osgviewer.hpp"
//#include "sco/expr_ops.hpp"
//#include "sco/modeling_utils.hpp"
//#include "sco/expr_op_overloads.hpp"

#include <iostream>

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

namespace traj_test {

class TrajTest {
    public:
        TrajTest();
        ~TrajTest() = default;
        
    
};

}
