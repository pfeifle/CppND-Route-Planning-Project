#include <optional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    float start_x=10.0, start_y=10.0, end_x=90.0, end_y=90.0;
    std::string s_start_x="",s_start_y="", s_end_x="", s_end_y="";
    char c;
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    std::cout <<"\nPlease enter the x coordinate of the start point as value between 0 and 100: ";
    std::cin >> s_start_x;  
    start_x=stof(s_start_x);  
    std::cout <<"\nPlease enter the y coordinate of the start point as value between 0 and 100: ";
    std::cin >> s_start_y;  
    start_y=stof(s_start_y);      
    std::cout <<"\nPlease enter the x coordinate of the end point as value between 0 and 100: ";
    std::cin >> s_end_x;  
    end_x=stof(s_end_x);      
    std::cout <<"\nPlease enter the y coordinate of the end point as value between 0 and 100: ";
    std::cin >> s_end_y;
    end_y=stof(s_end_y);  
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    //RoutePlanner route_planner{model, 10, 10, 90, 90};
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

