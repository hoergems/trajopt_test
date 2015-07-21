#include "traj.hpp"

using namespace std;
using namespace trajopt;
using namespace util;
using namespace OpenRAVE;

namespace traj_test {

TrajTest::TrajTest() {

}

}

int main(int argc, char** argv) {
    traj_test::TrajTest();
    bool plotting=false;
    bool verbose=false;
    string envfile;
    int decimation;

    {
      Config config;
      config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
      config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
      config.add(new Parameter<string>("envfile", &envfile, "jagged_narrow.xml"));
      config.add(new Parameter<int>("decimation", &decimation, "plot every n"));
      CommandParser parser(config);
      parser.read(argc, argv);
    }

    RaveInitialize(false, verbose ? Level_Debug : Level_Info);
    EnvironmentBasePtr env = RaveCreateEnvironment();
    env->StopSimulation();

    OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
    assert(viewer);
    return 0;
}
