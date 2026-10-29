#include "waypoint_planner_AreaMap.h"

void AreaMapAccessor::setMap(
    const nav_msgs::msg::OccupancyGrid& map)
{
    if(map.info.width <= 0 || map.info.height <= 0){
        initialized_ = false;
        return;
    } 

    map_ = map;
    initialized_ = true;
/*
    std::cout
        << "AreaMap loaded : "
        << map_.info.width
        << " x "
        << map_.info.height
        << std::endl;

    std::cout
        << "Resolution : "
        << map_.info.resolution
        << std::endl;

    std::cout
        << "Origin : "
        << map_.info.origin.position.x
        << ", "
        << map_.info.origin.position.y
        << std::endl;
*/        
}

bool AreaMapAccessor::worldToGrid(
    double wx,
    double wy,
    int& gx,
    int& gy) const
{
    if(!initialized_)
        return false;

    gx =
        static_cast<int>(
            (wx-map_.info.origin.position.x)
            /map_.info.resolution);

    gy =
        static_cast<int>(
            (wy-map_.info.origin.position.y)
            /map_.info.resolution);

    return
        gx>=0 &&
        gy>=0 &&
        gx<(int)map_.info.width &&
        gy<(int)map_.info.height;
}

int AreaMapAccessor::value(
    int gx,
    int gy) const
{
    if(!initialized_) return 0; //どこでも走行可能

    return
        map_.data[
            gy*
            map_.info.width+
            gx];
}

int AreaMapAccessor::valueWorld(
    double wx,
    double wy) const
{
    if(!initialized_) return 0;

    int gx,gy;

    if(!worldToGrid(
        wx,
        wy,
        gx,
        gy))
    {
        return -1;
    }

    return value(
        gx,
        gy);
}

void AreaMapAccessor::paintCircle(
    int gx,
    int gy,
    int radius)
{
    for(int dy=-radius;dy<=radius;dy++)
    {
        for(int dx=-radius;dx<=radius;dx++)
        {
            if(dx*dx+dy*dy > radius*radius)
                continue;

            int x=gx+dx;
            int y=gy+dy;

            if(x<0 || y<0 ||
               x>=(int)map_.info.width ||
               y>=(int)map_.info.height)
                continue;

            int index =
                y*map_.info.width+x;

            //map_.data[index]=0;      // Traversable
            if(map_.data[index] == -1)
            {
                map_.data[index] = 0;
            }

        
        }
    }
}

void AreaMapAccessor::paintTrajectory(
    double wx0,
    double wy0,
    double wx1,
    double wy1,
    double robot_width)
{
    int gx0,gy0;
    int gx1,gy1;

    if(!worldToGrid(wx0,wy0,gx0,gy0))
        return;

    if(!worldToGrid(wx1,wy1,gx1,gy1))
        return;

    int radius =
        robot_width*0.5/
        map_.info.resolution;

    //------------------------------------
    // Bresenham
    //------------------------------------

    int dx=std::abs(gx1-gx0);
    int dy=std::abs(gy1-gy0);

    int sx=(gx0<gx1)?1:-1;
    int sy=(gy0<gy1)?1:-1;

    int err=dx-dy;

    while(true)
    {
        paintCircle(
            gx0,
            gy0,
            radius);

        if(gx0==gx1 &&
           gy0==gy1)
            break;

        int e2=2*err;

        if(e2>-dy)
        {
            err-=dy;
            gx0+=sx;
        }

        if(e2<dx)
        {
            err+=dx;
            gy0+=sy;
        }
    }
}

bool AreaMapAccessor::savePGM(
    const std::string& pgm_file,
    const std::string& yaml_file) const
{
    //----------------------------------
    // Open PGM
    //----------------------------------

    std::ofstream ofs(
        pgm_file,
        std::ios::binary);

    if(!ofs.is_open())
        return false;

    //----------------------------------
    // Header
    //----------------------------------

    ofs
        << "P5\n"
        << map_.info.width
        << " "
        << map_.info.height
        << "\n255\n";

    //----------------------------------
    // Image
    //----------------------------------

    for(int y=map_.info.height-1;
        y>=0;
        y--)
    {
        for(int x=0;
            x<(int)map_.info.width;
            x++)
        {
            int index =
                y*
                map_.info.width+x;

            uint8_t value;

            switch(map_.data[index])
            {
            case 0:

                value=254;
                break;

            case 100:

                value=0;
                break;

            default:

                value=205;
            }

            ofs.write(
                reinterpret_cast<char*>(&value),
                1);
        }
    }

    ofs.close();

    
    ///// YAML ////

    std::ofstream yaml(yaml_file);

    if(!yaml.is_open())
        return false;

    std::string image_name =
        pgm_file.substr(
            pgm_file.find_last_of("/\\")+1);

    yaml
        << "image: "
        << image_name
        << "\n";

    yaml
        << "resolution: "
        << map_.info.resolution
        << "\n";

    yaml
        << "origin: ["
        << map_.info.origin.position.x
        << ", "
        << map_.info.origin.position.y
        << ", 0.0]\n";

    yaml
        << "negate: 0\n";

    yaml
        << "occupied_thresh: 0.65\n";

    yaml
        << "free_thresh: 0.20\n";

    yaml.close();

    return true;
}

bool AreaMapAccessor::isTraversableCell(
    int gx,
    int gy,
    bool allow_unknown) const
{
    const int v = value(gx,gy);

    if(v == 0)
        return true;

    if(allow_unknown && v == -1)
        return true;

    return false;   // 100(障害物) など
}

// (wx,wy) から (dir_x,dir_y) 方向へ進み、走行可能なセルが続く距離を返す
double AreaMapAccessor::rayDistance(
    double wx,
    double wy,
    double dir_x,
    double dir_y,
    double max_dist,
    bool allow_unknown) const
{
    const double step = map_.info.resolution * 0.5;  // セルを飛ばさないよう半分刻み
    double dist = 0.0;

    for(double d = step; d <= max_dist; d += step)
    {
        int gx,gy;

        // マップ外
        if(!worldToGrid(wx + dir_x*d, wy + dir_y*d, gx, gy))
            break;

        // 障害物（または未知）
        if(!isTraversableCell(gx,gy,allow_unknown))
            break;

        dist = d;
    }

    return dist;
}

bool AreaMapAccessor::traversableWidth(
    double wx,
    double wy,
    double yaw,
    double max_half_width,
    double& left,
    double& right,
    double& width,
    bool allow_unknown) const
{
    left = right = width = 0.0;

    if(!initialized_)
        return false;

    // 開始位置が走行可能か
    int gx,gy;
    if(!worldToGrid(wx,wy,gx,gy))
        return false;

    if(!isTraversableCell(gx,gy,allow_unknown))
        return false;

    // 進行方向に直交する単位ベクトル
    const double lx = -std::sin(yaw);   // 左 (yaw + 90deg)
    const double ly =  std::cos(yaw);

    left  = rayDistance(wx,wy, lx, ly, max_half_width, allow_unknown);
    right = rayDistance(wx,wy,-lx,-ly, max_half_width, allow_unknown);

    width = left + right;

    return true;
}