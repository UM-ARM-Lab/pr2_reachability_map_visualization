#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "reachability_map/reachability_map_problem.hpp"

using namespace std;

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if( type == OpenRAVE::PT_ProblemInstance && interfacename == "reachabilitymap" ) {
        return OpenRAVE::InterfaceBasePtr(new ReachabilityMapProblem(penv));
    }
    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_ProblemInstance].push_back("ReachabilityMap");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
