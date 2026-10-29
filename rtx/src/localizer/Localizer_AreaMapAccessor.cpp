#include "Localizer_AreaMapAccessor.h"

bool AreaMapAccessor::load(
    const std::string& pgm_file,
    const std::string& yaml_file)
{
    //----------------------------------
    // Load PGM
    //----------------------------------

    image_ =
        cv::imread(
            pgm_file,
            cv::IMREAD_GRAYSCALE);

    if(image_.empty())
    {
        std::cerr
            << "Failed to load PGM : "
            << pgm_file
            << std::endl;

        return false;
    }

    //----------------------------------
    // Default value
    //----------------------------------

    resolution_ = 0.1;

    origin_x_ = 0.0;
    origin_y_ = 0.0;

    //----------------------------------
    // Load YAML
    //----------------------------------

    std::ifstream ifs(yaml_file);

    if(!ifs.is_open())
    {
        std::cerr
            << "Failed to open YAML : "
            << yaml_file
            << std::endl;

        return false;
    }

    std::string line;

    while(std::getline(ifs,line))
    {
        //----------------------------------
        // resolution
        //----------------------------------

        if(line.find("resolution:") !=
           std::string::npos)
        {
            sscanf(
                line.c_str(),
                "resolution: %lf",
                &resolution_);
        }

        //----------------------------------
        // origin
        //----------------------------------

        if(line.find("origin:") !=
           std::string::npos)
        {
            double yaw;

            sscanf(
                line.c_str(),
                "origin: [%lf, %lf, %lf]",
                &origin_x_,
                &origin_y_,
                &yaw);
        }
    }

    ifs.close();

    occupancy_grid_ = occupancyGrid();
    
    initialized_ = true;

    //----------------------------------
    // Debug
    //----------------------------------

    std::cout
        << "AreaMap Loaded"
        << std::endl;

    std::cout
        << "Image Size : "
        << image_.cols
        << " x "
        << image_.rows
        << std::endl;

    std::cout
        << "Resolution : "
        << resolution_
        << std::endl;

    std::cout
        << "Origin : "
        << origin_x_
        << ", "
        << origin_y_
        << std::endl;

    return true;
}

bool AreaMapAccessor::load(
    const std::string& pgm_file)
{
    
    std::string yaml_file = pgm_file;
    yaml_file.replace(yaml_file.find_last_of('.'),4,".yaml");

    //----------------------------------
    // Load PGM
    //----------------------------------

    image_ =
        cv::imread(
            pgm_file,
            cv::IMREAD_GRAYSCALE);

    if(image_.empty())
    {
        std::cerr
            << "Failed to load PGM : "
            << pgm_file
            << std::endl;

        return false;
    }

    //----------------------------------
    // Default value
    //----------------------------------

    resolution_ = 0.1;

    origin_x_ = 0.0;
    origin_y_ = 0.0;

    //----------------------------------
    // Load YAML
    //----------------------------------

    std::ifstream ifs(yaml_file);

    if(!ifs.is_open())
    {
        std::cerr
            << "Failed to open YAML : "
            << yaml_file
            << std::endl;

        return false;
    }

    std::string line;

    while(std::getline(ifs,line))
    {
        //----------------------------------
        // resolution
        //----------------------------------

        if(line.find("resolution:") !=
           std::string::npos)
        {
            sscanf(
                line.c_str(),
                "resolution: %lf",
                &resolution_);
        }

        //----------------------------------
        // origin
        //----------------------------------

        if(line.find("origin:") !=
           std::string::npos)
        {
            double yaw;

            sscanf(
                line.c_str(),
                "origin: [%lf, %lf, %lf]",
                &origin_x_,
                &origin_y_,
                &yaw);
        }
    }

    ifs.close();

    //----------------------------------
    // Debug
    //----------------------------------

    std::cout
        << "AreaMap Loaded"
        << std::endl;

    std::cout
        << "Image Size : "
        << image_.cols
        << " x "
        << image_.rows
        << std::endl;

    std::cout
        << "Resolution : "
        << resolution_
        << std::endl;

    std::cout
        << "Origin : "
        << origin_x_
        << ", "
        << origin_y_
        << std::endl;

    return true;
}

nav_msgs::msg::OccupancyGrid
AreaMapAccessor::occupancyGrid() const
{
    nav_msgs::msg::OccupancyGrid grid;

    grid.header.frame_id = "map";

    grid.info.resolution = resolution_;

    grid.info.width  = image_.cols;
    grid.info.height = image_.rows;

    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(image_.cols * image_.rows);

    for(int y=0;y<image_.rows;y++)
    {
        for(int x=0;x<image_.cols;x++)
        {
            uint8_t value = image_.at<uint8_t>(y,x);

            int index =
                (image_.rows-1-y)*image_.cols+x;

            if (value >= 250){
                grid.data[index]=0;
                //break;
            }
            else if(value == 0){
                grid.data[index]=100;
                //break;
            }
            else{
                grid.data[index]=-1;
                //break;
            }     
            // switch(value)
            // {
            // case 254:

            //     grid.data[index]=0;
            //     break;

            // case 0:

            //     grid.data[index]=100;
            //     break;

            // default:

            //     grid.data[index]=-1;
            //     break;
            // }
        }
    }

    return grid;
}

nav_msgs::msg::OccupancyGrid
AreaMapAccessor::crop(
    double center_x,
    double center_y,
    double width,
    double height) const
{
    nav_msgs::msg::OccupancyGrid local;

    //----------------------------------
    // information
    //----------------------------------

    local.header.frame_id = "map";

    local.info.resolution = occupancy_grid_.info.resolution;

    local.info.width =
        static_cast<int>(
            width/local.info.resolution);

    local.info.height =
        static_cast<int>(
            height/local.info.resolution);

    //----------------------------------
    // Local origin
    //----------------------------------

    local.info.origin.position.x =
        center_x-width*0.5;

    local.info.origin.position.y =
        center_y-height*0.5;

    local.info.origin.orientation.w = 1.0;

    //----------------------------------

    local.data.assign(
        local.info.width*
        local.info.height,
        -1);

    //----------------------------------
    // global pixel
    //----------------------------------

    int gx0 =
        static_cast<int>(
        (local.info.origin.position.x-
         occupancy_grid_.info.origin.position.x)
        /local.info.resolution);

    int gy0 =
        static_cast<int>(
        (local.info.origin.position.y-
         occupancy_grid_.info.origin.position.y)
        /local.info.resolution);

    //----------------------------------
    // copy
    //----------------------------------

    for(int y=0;y<(int)local.info.height;y++)
    {
        int gy=gy0+y;

        if(gy<0 ||
           gy>=
           (int)occupancy_grid_.info.height)
            continue;

        for(int x=0;x<(int)local.info.width;x++)
        {
            int gx=gx0+x;

            if(gx<0 ||
               gx>=
               (int)occupancy_grid_.info.width)
                continue;

            int src=
                gy*
                occupancy_grid_.info.width+
                gx;

            int dst=
                y*
                local.info.width+
                x;

            local.data[dst]=
                occupancy_grid_.data[src];
        }
    }

    return local;
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
        occupancy_grid_.info.resolution;

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
            radius,true);

        // inflateCircle(
        //     gx0,
        //     gy0,
        //     radius);

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
        << occupancy_grid_.info.width
        << " "
        << occupancy_grid_.info.height
        << "\n255\n";

    //----------------------------------
    // Image
    //----------------------------------

    for(int y=occupancy_grid_.info.height-1;
        y>=0;
        y--)
    {
        for(int x=0;
            x<(int)occupancy_grid_.info.width;
            x++)
        {
            int index =
                y*
                occupancy_grid_.info.width+x;

            uint8_t value;

            switch(occupancy_grid_.data[index])
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
        << occupancy_grid_.info.resolution
        << "\n";

    yaml
        << "origin: ["
        << occupancy_grid_.info.origin.position.x
        << ", "
        << occupancy_grid_.info.origin.position.y
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

void AreaMapAccessor::paintCircle(
    int gx,
    int gy,
    int radius,bool flag_force_paint)
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
               x>=(int)occupancy_grid_.info.width ||
               y>=(int)occupancy_grid_.info.height)
                continue;

            int index =
                y*occupancy_grid_.info.width+x;

            //map_.data[index]=0;      // Traversable
            if(flag_force_paint){
                occupancy_grid_.data[index] = 0;
            }
            else if(occupancy_grid_.data[index] == -1)
            {
                occupancy_grid_.data[index] = 0;
            }

        
        }
    }
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
            (wx-occupancy_grid_.info.origin.position.x)
            /occupancy_grid_.info.resolution);

    gy =
        static_cast<int>(
            (wy-occupancy_grid_.info.origin.position.y)
            /occupancy_grid_.info.resolution);

    return
        gx>=0 &&
        gy>=0 &&
        gx<(int)occupancy_grid_.info.width &&
        gy<(int)occupancy_grid_.info.height;
}

void AreaMapAccessor::inflateCircle(
    int cx,
    int cy,
    double radius)
{
    int r =
        static_cast<int>(
            std::ceil(
                radius / resolution_));

    double r2 = radius * radius;

    for(int dy=-r; dy<=r; dy++)
    {
        int yy = cy + dy;

        if(yy < 0 || yy >= (int)occupancy_grid_.info.height)
            continue;

        for(int dx=-r; dx<=r; dx++)
        {
            int xx = cx + dx;

            if(xx < 0 || xx >= (int)occupancy_grid_.info.width)
                continue;

            double dist2 =
                (dx * resolution_) *
                (dx * resolution_) +
                (dy * resolution_) *
                (dy * resolution_);

            if(dist2 > r2)
                continue;
            
            int index =
                yy*occupancy_grid_.info.width+xx;

            occupancy_grid_.data[index] = 0;

            //map[yy * width_ + xx] = value;
        }
    }
}