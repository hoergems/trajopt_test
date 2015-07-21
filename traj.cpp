#include "traj.hpp"

using namespace std;
using namespace trajopt;

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
    return 0;
}
