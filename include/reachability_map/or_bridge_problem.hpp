#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <list>
#include <map>
#include <string>
#include "sdf_tools/collision_map.hpp"
#include <openrave/openrave.h>

#ifndef BRIDGEPROBLEM_HPP
#define BRIDGEPROBLEM_HPP

class BridgeProblem : public OpenRAVE::ProblemInstance
{
public:
    BridgeProblem(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~BridgeProblem();
    virtual void Destroy();
    virtual int main(const std::string& args);
//    virtual void SetActiveRobots(const std::vector<OpenRAVE::RobotBasePtr>& robots);

    /// returns true when problem is solved
//    virtual bool SimulationStep(OpenRAVE::dReal fElapsedTime);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);
//    virtual void Query(const char* query, std::string& response);

    bool GetLinkSamples(std::ostream& sout, std::istream& sinput);
    bool InitGrid(std::ostream& sout, std::istream& sinput);
    bool InitGridOnTable(std::ostream& sout, std::istream& sinput);
    bool TestCostFunction(std::ostream& sout, std::istream& sinput);
    bool UpdateOccupancy(std::ostream& sout, std::istream& sinput);
    bool GetPenetrationCost(std::ostream& sout, std::istream& sinput);
    bool GetPenetrationGradient(std::ostream& sout, std::istream& sinput);
    bool GetSelfLaneCost(std::ostream& sout, std::istream& sinput);
    bool DrawOccupancy(std::ostream& sout, std::istream& sinput);
    bool DrawGrid(std::ostream& sout, std::istream& sinput);
    bool SetEvalSweptVolume(std::ostream& sout, std::istream& sinput);
//    bool SetActiveLinks(std::ostream& sout, std::istream& sinput);
    bool AddRobot(std::ostream& sout, std::istream& sinput);
    bool Reset(std::ostream& sout, std::istream& sinput);
    bool DrawPenSlice(std::ostream& sout, std::istream& sinput);
    bool DrawSelfSlice(std::ostream& sout, std::istream& sinput);
    std::vector<OpenRAVE::RaveVector<float> > ParseCSV(std::string filename);

    double ScaleValue(double val, double min, double max, double a, double b);
    float GetMaxCellValue();
    OpenRAVE::RaveVector<float> GetRGBColors(float min, float max, float value);
    OpenRAVE::RaveVector<float> GetRGBColors(float min, float max, float value, std::string color);
    OpenRAVE::RaveVector<float> GetCostColor(float value);

private:
    // Passive robot : robot the occupancy grid is created around
//    OpenRAVE::RobotBasePtr _passiveRobot;
////    std::vector<OpenRAVE::KinBody::LinkPtr> _passiveLinks; //Links to check for passive robot
//    std::map<std::string, std::pair<OpenRAVE::KinBody::LinkPtr, std::vector<OpenRAVE::RaveVector<float>>>> _passiveLinks;
//    std::vector<std::string> _passiveLinkNames;
//    std::string _strPassiveRobotName; ///< name of the active robot

//    // Active robot : robot whose penetration cost is checked
//    OpenRAVE::RobotBasePtr _activeRobot;
//    std::map<std::string, std::pair<OpenRAVE::KinBody::LinkPtr, std::vector<OpenRAVE::RaveVector<float>>>> _activeLinks;
//    std::vector<std::string> _activeLinkNames;
//    std::string _strActiveRobotName; ///< name of the active robot
    int _numPoints;
    double _maxCellVal;

    std::map<std::string, std::vector<std::pair<OpenRAVE::KinBody::LinkPtr, std::vector<OpenRAVE::RaveVector<float>>>>> _robots;

    sdf_tools::CollisionMapGrid _collisionGrid;
    OpenRAVE::KinBodyPtr _collisionBox;

    bool _evalSweptVolume;
    sdf_tools::CollisionMapGrid _sweptGrid;

    bool _sdfValid;
    sdf_tools::SignedDistanceField _sdf;
    std::pair<double, double> _sdfRange;

    std::vector<OpenRAVE::GraphHandlePtr> _graphPtrs;
    std::vector<OpenRAVE::RaveVector<float> > _vPoints;
    std::vector<float> _vColors;
//    std::vector<OpenRAVE::RaveVector<float> > _vColors;

    double _cellRes;

};


#endif
