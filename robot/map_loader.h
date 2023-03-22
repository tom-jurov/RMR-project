#ifndef MAP_LOADER_H
#define MAP_LOADER_H
#include <vector>
#include "robot.h"

typedef struct
{

    double x;
    double y;

}Point;

typedef union{
    Point point;
    double suradnice[2];
}TMapPoint;

typedef struct
{
    int numofpoints;
    std::vector<TMapPoint> points;
}TMapObject;

typedef struct
{
   TMapObject wall;
   int numofObjects;
   std::vector<TMapObject> obstacle;
}TMapArea;

struct ObstacleCells
{
    int x;
    int y;
};

class ROBOT_EXPORT map_loader
{
static constexpr int GRID_SIZE = 5;
public:
    map_loader();
    void load_map(const char *filename,TMapArea &mapss);
    std::vector<std::vector<int>> createMap(const TMapArea& map);
    std::vector<std::vector<int>> createBloatedMap();
private:
    std::vector<std::vector<int>> occupancy_map_;
    std::vector<ObstacleCells> obstacle_cells_;
};

#endif // MAP_LOADER_H
