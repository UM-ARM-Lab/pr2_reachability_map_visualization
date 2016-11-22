/////////////////////////////
// ReachabilityMapProblem //
/////////////////////////////
#include "reachability_map/reachability_map_problem.hpp"
#include <boost/tokenizer.hpp>
#include <math.h>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <Eigen/Geometry>

using namespace std;

ReachabilityMapProblem::ReachabilityMapProblem(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ProblemInstance(penv)
{
    RegisterCommand("InitReachabilityMap",boost::bind(&ReachabilityMapProblem::InitReachabilityMap,this,_1,_2),
                    "Load reachability map");

    RegisterCommand("GetReachabilityMapCost",boost::bind(&ReachabilityMapProblem::GetReachabilityMapCost,this,_1,_2),
                    "Get reachability map cost");

    RegisterCommand("DrawGrid",boost::bind(&ReachabilityMapProblem::DrawGrid,this,_1,_2),
                    "Draws a the full grid");

    RegisterCommand("UpdateMap", boost::bind(&ReachabilityMapProblem::UpdateReachabilityMap, this, _1, _2),
                    "Update reachability map");
//Rafi's collision map staff, might be useful for future visualization
//    RegisterCommand("DrawOccupancy",boost::bind(&ReachabilityMapProblem::DrawOccupancy,this,_1,_2),
//                    "Draws a color map of occupied cells based on their occupancy value");

//    RegisterCommand("SetActiveLinks",boost::bind(&BridgeProblem::SetActiveLinks,this,_1,_2),
//                    "Sets which links of the active robot will be collision checked in the grid");

//    RegisterCommand("AddRobot",boost::bind(&ReachabilityMapProblem::AddRobot,this,_1,_2),
//                    "Add a robot to be tracked.  Give the robot name, a path to a folder with its link samples, and each link name");

//    RegisterCommand("DrawPenSlice",boost::bind(&ReachabilityMapProblem::DrawPenSlice,this,_1,_2),
//                    "Draw a slice of the cost function");

    // Init variables
//    _sdfValid = false;
//    _evalSweptVolume = false;
}


void ReachabilityMapProblem::Destroy()
{

}

ReachabilityMapProblem::~ReachabilityMapProblem()
{
    Destroy();
}

std::vector<OpenRAVE::RaveVector<float> > ReachabilityMapProblem::ParseCSV(std::string filename)
{
    std::vector<OpenRAVE::RaveVector<float> > mat;
    std::ifstream  data(filename);

    std::string line;
    while(std::getline(data,line))
    {
        int idx = 0;
        std::stringstream lineStream(line);
        std::string   cell;
        OpenRAVE::RaveVector<float> coord;
        while(std::getline(lineStream,cell,','))
        {
            coord[idx] = (float)atof(cell.c_str());
            idx++;
        }
        mat.push_back(coord);
    }
    mat.pop_back(); //Hack to remove trailing new line
//    cout << mat[mat.size()-1] << endl;
//    cout << mat[mat.size()] << endl;
//    cout << mat[0] << endl;
    return mat;
}

int ReachabilityMapProblem::main(const std::string& cmd)
{

    RAVELOG_INFO("Starting Reachability Problem\n");

    return 0;
}

bool ReachabilityMapProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
//    SetActiveRobots(GetEnv()->GetRobots());
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    OpenRAVE::ProblemInstance::SendCommand(sout,sinput);
    return true;
}

bool ReachabilityMapProblem::InitReachabilityMap(std::ostream& sout, std::istream& sinput)
{
    std::string robot_name;
    std::string manip_name;
    std::string map_path;
    sinput >> robot_name;
    sinput >> manip_name;
    sinput >> map_path;
    if (this->reachability_map.LoadFromFile(map_path))
    {
        RAVELOG_INFO("Success Loading Reachability Map\n");
    }else
    {
        RAVELOG_INFO("Fail Loading Reachability Map\n");
    }
    this->_robotPtr = GetEnv()->GetRobot(robot_name);
    this->_manipPtr = this->_robotPtr->SetActiveManipulator(manip_name);
    this->_robotPtr->SetActiveDOFs(this->_manipPtr->GetArmIndices());

    return true;
}

bool ReachabilityMapProblem::GetReachabilityMapCost(std::ostream& sout, std::istream& sinput)
{
    string x_string;
    string y_string;
    string z_string;
    sinput >> x_string;
    sinput >> y_string;
    sinput >> z_string;
    double x = std::stod(x_string);
    double y = std::stod(y_string);
    double z = std::stod(z_string);
    std::vector<int64_t> index = this->reachability_map.LocationToGridIndex(x,y,z);
//    std::cout<<"input location: "<<x<<" "<<y<<" "<<z<<std::endl;
    double cost = 0.0;
    if (!index.empty())
    {
        cost = this->GetReachabilityCost(index[0],index[1],index[2]);
//        std::vector<double> locations=this->reachability_map.GridIndexToLocation(index[0],index[1],index[2]);
//        std::cout<<"location in map: "<<locations[0]<<" "<<locations[1]<<" "<<locations[2]<<std::endl;
    }else{
        cost = 0;
    }
//    cost = (64.0-cost)/64.0;
    sout<<std::to_string(cost);
    return true;
}

bool ReachabilityMapProblem::UpdateReachabilityMap(std::ostream& sout, std::istream& sinput)
{
    string x_string;
    string y_string;
    string z_string;
    sinput >> x_string;
    sinput >> y_string;
    sinput >> z_string;
    double x = std::stod(x_string);
    double y = std::stod(y_string);
    double z = std::stod(z_string);
//    std::vector<int64_t> index = this->reachability_map.LocationToGridIndex(x,y,z);

    string cost_string;
    sinput >> cost_string;
    double cost_d = std::stod(cost_string);
    u_int64_t cost = u_int64_t(cost_d);
    this->reachability_map.Set(x,y,z,cost);
    return true;
}

double ReachabilityMapProblem::GetReachabilityCost(int64_t x, int64_t y, int64_t z)
{
//    Get Reachability by the index of x, y, z
    u_int64_t reachability = this->reachability_map.GetImmutable(x, y, z).first;
    int cost = 0;
    for (u_int32_t bit_position = 0; bit_position < 64; bit_position++)
    {
        bool x = (reachability >> bit_position);
        cost = cost + x;
    }
    return double(cost);
}

bool ReachabilityMapProblem::DrawGrid(std::ostream& sout, std::istream& sinput)
{
//    Already update by Ruikun
    _graphPtrs.clear();

    for (int64_t x = 0; x < this->reachability_map.GetNumXCells(); x++)
    {
        for (int64_t y = 0; y < this->reachability_map.GetNumYCells(); y++)
        {
            for (int64_t z = 0; z < this->reachability_map.GetNumZCells(); z++)
            {
                const Eigen::Vector3d current_position = EigenHelpers::StdVectorDoubleToEigenVector3d(reachability_map.GridIndexToLocation(x, y, z));
                OpenRAVE::Transform Tbase = this->_manipPtr->GetBase()->GetTransform();
                OpenRAVE::Transform Tpoint;
                Tpoint.trans = OpenRAVE::Vector(current_position[0],current_position[1],current_position[2]);
                OpenRAVE::Transform T = Tbase*Tpoint;
                double cost = this->GetReachabilityCost(x,y,z);
                _vPoints.push_back(T.trans);

                _vColors.push_back(1-cost/64);
                _vColors.push_back(1-cost/64);
                _vColors.push_back(1-cost/64);
                _vColors.push_back(1);

            }
        }
    }

    _graphPtrs.push_back(GetEnv()->plot3(&_vPoints[0].x,_vPoints.size(),sizeof(_vPoints[0]),_cellRes,&_vColors[0],0, true));

    return true;
}



// Rafi's old collision map stuff.
/*

float ReachabilityMapProblem::GetMaxCellValue()
{
    float max = 0;

    for (int64_t x = 0; x < _collisionGrid.GetNumXCells(); x++)
    {
        for (int64_t y = 0; y < _collisionGrid.GetNumYCells(); y++)
        {
            for (int64_t z = 0; z < _collisionGrid.GetNumZCells(); z++)
            {
                float cellVal = _collisionGrid.Get(x,y,z).first.occupancy;
                if (cellVal > max)
                    max = cellVal;
            }
        }
    }

    return max;
}


//   Return a RGB colour value given a scalar v in the range [vmin,vmax]
//   In this case each colour component ranges from 0 (no contribution) to
//   1 (fully saturated), modifications for other ranges is trivial.
//   The colour is clipped at the end of the scales if v is outside
//   the range [vmin,vmax]


OpenRAVE::RaveVector<float> GetColour(double v,double vmin,double vmax)
{
   OpenRAVE::RaveVector<float> c = {1.0, 1.0, 1.0, 1.0};
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
       c[0] = 0;
       c[1] = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
       c[0] = 0;
       c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
       c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
       c[2] = 0;
   } else {
       c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
       c[2] = 0;
   }

   return(c);
}

OpenRAVE::RaveVector<float> ReachabilityMapProblem::GetCostColor(float value)
{

//    colors[0] = 255.0*value;
//    colors[1] = 0;
//    colors[2] = 0;
//    colors[3] = 1;
//    return colors;

    OpenRAVE::RaveVector<float> colors = GetColour(value,0,1);
    return colors;
}

OpenRAVE::RaveVector<float> ReachabilityMapProblem::GetRGBColors(float min, float max, float value)
{
    OpenRAVE::RaveVector<float> colors = GetColour(value,min,max);
    return colors;
}

OpenRAVE::RaveVector<float> ReachabilityMapProblem::GetRGBColors(float min, float max, float value, std::string color)
{
    OpenRAVE::RaveVector<float> colors;

    if (color == "red")
    {
        float ratio = (value-min) / (max-min);
        float r, g, b;
        b = 0;
        r = 1*ratio;
        g = 0;

        colors[0] = r; // Red
        colors[1] = g; // Green
        colors[2] = b; // Blue
//        if (ratio < 0.2)
//            ratio = 0;
        colors[3] = ratio;
    }
    if (color == "green")
    {
        float ratio = (value-min) / (max-min);
        float r, g, b;
        b = 0;
        r = 0;
        g = 1*ratio;;

        colors[0] = r; // Red
        colors[1] = g; // Green
        colors[2] = b; // Blue
//        if (ratio < 0.2)
//            ratio = 0;
        colors[3] = ratio;
    }
    else if (color == "blue")
    {
        float ratio = (value-min) / (max-min);
        float r, g, b;
        b = 1*ratio;
        r = 0;
        g = 0;

        colors[0] = r; // Red
        colors[1] = g; // Green
        colors[2] = b; // Blue
//        if (ratio < 0.2)
//            ratio = 0;
        colors[3] = ratio;
    }
    else
    {
        colors = GetRGBColors(min, max, value);
    }
    return colors;
}


bool ReachabilityMapProblem::DrawOccupancy(std::ostream& sout, std::istream& sinput)
{
    std::string color;
    sinput >> color;

    _graphPtrs.clear();
//    float max = GetMaxCellValue();
    float max = _maxCellVal;
    float min = 1;

    if (!_sdfValid)
    {
        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf = _collisionGrid.ExtractSignedDistanceField(INFINITY);
        _sdf = sdf.first;
        _sdfRange = std::pair<double, double>(sdf.second.second, sdf.second.first); //Wrote code for (min, max) library uses (max, min)
        _sdfValid = true;
    }

//    cout << "min " << min << " max " << max << endl;

    for (int64_t x = 0; x < _collisionGrid.GetNumXCells(); x++)
    {
        for (int64_t y = 0; y < _collisionGrid.GetNumYCells(); y++)
        {
            for (int64_t z = 0; z < _collisionGrid.GetNumZCells(); z++)
            {
                float cellVal = _collisionGrid.Get(x,y,z).first.occupancy;
                float sdf_cell = _sdf.GetSafe(x,y,z).first;

                if (cellVal > 0)
                {
                    double val = ScaleValue(log(cellVal+1), 0, log(_maxCellVal+1), 0, 1) * ScaleValue(sdf_cell, _sdfRange.first, _sdfRange.second, 1, 0);
                    std::vector<double> cell_pos = _collisionGrid.GridIndexToLocation(x,y,z);
    //                std::vector<float> pos(cell_pos.begin(), cell_pos.end());

                    _vPoints.push_back(OpenRAVE::RaveVector<float>(cell_pos[0], cell_pos[1], cell_pos[2]));

                    OpenRAVE::RaveVector<float> colors = GetRGBColors(0,1, val, color);
                    _vColors.push_back(colors[0]);
                    _vColors.push_back(colors[1]);
                    _vColors.push_back(colors[2]);
                    _vColors.push_back(colors[3]);
//                    _vColors.push_back(GetRGBColors(min, max, cellVal  ));
//                    _vColors.push_back(OpenRAVE::RaveVector<float>(1,0,0,0.2));
                }

            }
        }
    }
    _graphPtrs.push_back(GetEnv()->plot3(&_vPoints[0].x,_vPoints.size(),sizeof(_vPoints[0]),10.0,&_vColors[0],0, true));



    return true;
}

bool ReachabilityMapProblem::DrawPenSlice(std::ostream& sout, std::istream& sinput)
{
    std::string temp;
    sinput >> temp;
    int64_t z = std::stoi(temp);

    _graphPtrs.clear();
    _vPoints.clear();
    _vColors.clear();

    cout << "Trying to draw z of : " << z << endl;
    cout << "Num z " << _collisionGrid.GetNumZCells() << endl;

    std::vector<OpenRAVE::RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    for(OpenRAVE::RobotBasePtr& robot : robots)
        robot->SetVisible(false);

    if (!_sdfValid)
    {
        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf = _collisionGrid.ExtractSignedDistanceField(INFINITY);
        _sdf = sdf.first;
        _sdfRange = std::pair<double, double>(sdf.second.second, sdf.second.first); //Wrote code for (min, max) library uses (max, min)
        _sdfValid = true;

    }
    for (int64_t x = 0; x < _collisionGrid.GetNumXCells(); x++)
    {
        for (int64_t y = 0; y < _collisionGrid.GetNumYCells(); y++)
        {
            std::pair<sdf_tools::COLLISION_CELL, bool> cell = _collisionGrid.Get(x,y,z);
            std::pair<float, bool> sdf_cell = _sdf.GetSafe(x,y,z);


            if (cell.second == true && sdf_cell.second == true)
            {

                // Normalize cell val
                double val = cell.first.occupancy;
                double max = _maxCellVal;
                double min = 0;

                if (val > 0)
                    val = ScaleValue(log(val+1), 0, log(_maxCellVal+1), 0, 1);
                else if (val == 0)
                    val = ScaleValue(log(0.9+1), 0, log(_maxCellVal+1), 0, 1);
                double sdf_val = ScaleValue(atan(sdf_cell.first), atan(_sdfRange.first), atan(_sdfRange.second), 1, 0);

                std::vector<double> cell_pos = _collisionGrid.GridIndexToLocation(x,y,z);
                cell_pos[2] -= 5;
                _vPoints.push_back(OpenRAVE::RaveVector<float>((float)cell_pos[0], (float)cell_pos[1], (float)cell_pos[2]));
//                OpenRAVE::RaveVector<float> colors = GetCostColor(float(val));
//                OpenRAVE::RaveVector<float> colors = GetCostColor(float(sdf_val));
                OpenRAVE::RaveVector<float> colors = GetCostColor(float(val*sdf_val));


//                cout << "scaled value : " << (val*sdf_val) << endl;
//                cout << "recovered colors: " << PrettyPrint::PrettyPrint(colors) << endl;

                if (y == 5)
                {
//                    cout << sdf_cell.first << " " << sdf_val <<" "<< colors[0]<< endl;

                    _vColors.push_back(colors[0]);
                    _vColors.push_back(colors[1]);
                    _vColors.push_back(colors[2]);
//                    _vColors.push_back(colors[3]);
                }
                else
                {
                    _vColors.push_back(colors[0]);
                    _vColors.push_back(colors[1]);
                    _vColors.push_back(colors[2]);
//                    _vColors.push_back(colors[3]);
                }
            }
        }
    }
//    cout << "max value : " << max_val << endl;
    _graphPtrs.push_back(GetEnv()->plot3(&_vPoints[0].x,_vPoints.size(),sizeof(_vPoints[0]),_cellRes/2.0,&_vColors[0],1, false));

    return true;
}



// Scales a value between min max to range a-b
double ReachabilityMapProblem::ScaleValue(double val, double min, double max, double a, double b)
{
    return (((b-a)*(val-min))/(max-min))+a;
}
*/








