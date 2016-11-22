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
#include <sdf_tools/reachability_map.hpp>
#include <openrave/openrave.h>

#ifndef REACHABILITYMAPPROBLEM_HPP
#define REACHABILITYMAPPROBLEM_HPP

class ReachabilityMapProblem : public OpenRAVE::ProblemInstance
{
public:
    ReachabilityMapProblem(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~ReachabilityMapProblem();
    virtual void Destroy();
    virtual int main(const std::string& args);
//    virtual void SetActiveRobots(const std::vector<OpenRAVE::RobotBasePtr>& robots);

    /// returns true when problem is solved
//    virtual bool SimulationStep(OpenRAVE::dReal fElapsedTime);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);
//    virtual void Query(const char* query, std::string& response);

    bool InitReachabilityMap(std::ostream& sout, std::istream& sinput);
    bool GetReachabilityMapCost(std::ostream& sout, std::istream& sinput);
    bool UpdateReachabilityMap(std::ostream& sout, std::istream& sinput);


    bool DrawGrid(std::ostream& sout, std::istream& sinput);
    std::vector<OpenRAVE::RaveVector<float> > ParseCSV(std::string filename);

//    bool DrawOccupancy(std::ostream& sout, std::istream& sinput);
//    bool SetActiveLinks(std::ostream& sout, std::istream& sinput);
//    bool AddRobot(std::ostream& sout, std::istream& sinput);
//    bool DrawPenSlice(std::ostream& sout, std::istream& sinput);

//    double ScaleValue(double val, double min, double max, double a, double b);
//    float GetMaxCellValue();
//    OpenRAVE::RaveVector<float> GetRGBColors(float min, float max, float value);
//    OpenRAVE::RaveVector<float> GetRGBColors(float min, float max, float value, std::string color);
//    OpenRAVE::RaveVector<float> GetCostColor(float value);

private:
    sdf_tools::ReachabilityMapGrid reachability_map;
    OpenRAVE::RobotBasePtr _robotPtr;
    OpenRAVE::RobotBase::ManipulatorPtr _manipPtr;
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
    double GetReachabilityCost(int64_t x, int64_t y, int64_t z);

    double _cellRes;

};


#endif
