#include "map_loader.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "iostream"

map_loader::map_loader()
    : occupancy_map_(std::vector<std::vector<int>>(115, std::vector<int>(115,0)))
    , obstacle_cells_({})
{
}

 void map_loader::load_map(const char *filename,TMapArea &mapss)
 {


     FILE *fp=fopen(filename,"r");
     if (fp==NULL)
     {

         printf("zly file\n");
         return ;
     }

     //tu nacitame obvodovu stenu
     char myLine[550];
     fgets(myLine,550,fp);
     printf("%s\n",myLine);
     char *myCopy=(char*)calloc(strlen(myLine)+2,sizeof(char));
     memcpy(myCopy,myLine,sizeof(char)*strlen(myLine));
     char *freeMyCopy;
     freeMyCopy=myCopy;
     myCopy=strtok(myCopy,"[]");
     mapss.wall.numofpoints=(atoi(myCopy));
      printf("num of points %i\n",mapss.wall.numofpoints);
      mapss.wall.points.reserve(mapss.wall.numofpoints);
      for(int i=0;i<mapss.wall.numofpoints;i++)
      {
          TMapPoint temp;
          myCopy=strtok(NULL,"[,");
          temp.point.x=atof(myCopy);
          myCopy=strtok(NULL,"[,");
          temp.point.y=atof(myCopy);
          mapss.wall.points.push_back(temp);
       //   mapss.wall.points[i/2].suradnice[i%2]=atof(myCopy);

      }
      free(freeMyCopy);

      //tu nacitame jednotlive prekazky
       mapss.numofObjects=0;
        mapss.obstacle.clear();
      while( fgets(myLine,550,fp))
      {
          printf("%s\n",myLine);
          myCopy=(char*)calloc(strlen(myLine)+2,sizeof(char));
          memcpy(myCopy,myLine,sizeof(char)*strlen(myLine));

          freeMyCopy=myCopy;
          myCopy=strtok(myCopy,"[]");
          if((atoi(myCopy))==0)
              break;
          TMapObject tempObstacle;
          mapss.numofObjects++;

          tempObstacle.numofpoints=(atoi(myCopy));
          for(int i=0;i< tempObstacle.numofpoints;i++)
           {
              TMapPoint temp;
              myCopy=strtok(NULL,"[,");
              temp.point.x=atof(myCopy);
              myCopy=strtok(NULL,"[,");
              temp.point.y=atof(myCopy);
              tempObstacle.points.push_back(temp);

           }
           free(freeMyCopy);
           mapss.obstacle.push_back(tempObstacle);



      }


 }

 std::vector<std::vector<int>> map_loader::createMap(const TMapArea& map)
 {
     int i = 0;
     for (const auto& obstacle : map.obstacle)
     {
         i=0;
         for (const auto& point : obstacle.points){
             auto temp_x = point.point.x;
             auto temp_y = point.point.y;
             if (i==0)
             {
                while(temp_y <= (&point+1)->point.y)
                {
                    occupancy_map_[static_cast<int>(point.point.x/GRID_SIZE)][static_cast<int>(temp_y/GRID_SIZE)] = 1;
                    obstacle_cells_.push_back({static_cast<int>(point.point.x/GRID_SIZE), static_cast<int>(temp_y/GRID_SIZE)});
                    temp_y += GRID_SIZE;
                }
             }
             else if(i==1)
             {
                 while(temp_x <= (&point+1)->point.x)
                 {
                     occupancy_map_[static_cast<int>(temp_x/GRID_SIZE)][static_cast<int>(point.point.y/GRID_SIZE)] = 1;
                     obstacle_cells_.push_back({static_cast<int>(temp_x/GRID_SIZE), static_cast<int>(point.point.y/GRID_SIZE)});
                     temp_x += GRID_SIZE;
                 }
             }
             else if(i==2)
             {
                 while(temp_y >= (&point+1)->point.y)
                 {
                     occupancy_map_[static_cast<int>(point.point.x/GRID_SIZE)][static_cast<int>(temp_y/GRID_SIZE)] = 1;
                     obstacle_cells_.push_back({static_cast<int>(point.point.x/GRID_SIZE), static_cast<int>(temp_y/GRID_SIZE)});
                     temp_y -= GRID_SIZE;
                 }
             }
             else if(i==3)
             {
                 while(temp_x >= (&point-3)->point.x)
                 {
                     occupancy_map_[static_cast<int>(temp_x/GRID_SIZE)][static_cast<int>(point.point.y/GRID_SIZE)] = 1;
                     obstacle_cells_.push_back({static_cast<int>(temp_x/GRID_SIZE), static_cast<int>(point.point.y/GRID_SIZE)});
                     temp_x -= GRID_SIZE;
                 }
             }
             i++;
         }
     }
     i = 0;
     for (const auto& point : map.wall.points)
     {
        auto temp_x = point.point.x;
        auto temp_y = point.point.y;
        if (i==0)
        {
            while(temp_x <= (&point+1)->point.x)
            {
                occupancy_map_[static_cast<int>(temp_x/GRID_SIZE)][static_cast<int>(point.point.y/GRID_SIZE)] = 1;
                obstacle_cells_.push_back({static_cast<int>(temp_x/GRID_SIZE), static_cast<int>(point.point.y/GRID_SIZE)});
                temp_x += GRID_SIZE;
            }
        }
        else if (i==1 || i==3)
        {
            while(temp_y <= (&point+1)->point.y)
            {
                occupancy_map_[static_cast<int>(point.point.x/GRID_SIZE)][static_cast<int>(temp_y/GRID_SIZE)] = 1;
                obstacle_cells_.push_back({static_cast<int>(point.point.x/GRID_SIZE), static_cast<int>(temp_y/GRID_SIZE)});
                temp_y += GRID_SIZE;
            }
        }
        else if (i==2 || i==4 || i==6)
        {
            while(temp_x >= (&point+1)->point.x)
            {
                occupancy_map_[static_cast<int>(temp_x/GRID_SIZE)][static_cast<int>(point.point.y/GRID_SIZE)] = 1;
                obstacle_cells_.push_back({static_cast<int>(temp_x/GRID_SIZE), static_cast<int>(point.point.y/GRID_SIZE)});
                temp_x -= GRID_SIZE;
            }
        }
        else if (i==5)
        {
            while(temp_y >= (&point+1)->point.y)
            {
                occupancy_map_[static_cast<int>(point.point.x/GRID_SIZE)][static_cast<int>(temp_y/GRID_SIZE)] = 1;
                obstacle_cells_.push_back({static_cast<int>(point.point.x/GRID_SIZE), static_cast<int>(temp_y/GRID_SIZE)});
                temp_y -= GRID_SIZE;
            }
        }
        else if (i==7)
        {
            while(temp_y >= (&point-7)->point.y)
            {
                occupancy_map_[static_cast<int>(point.point.x/GRID_SIZE)][static_cast<int>(temp_y/GRID_SIZE)] = 1;
                obstacle_cells_.push_back({static_cast<int>(point.point.x/GRID_SIZE), static_cast<int>(temp_y/GRID_SIZE)});
                temp_y -= GRID_SIZE;
            }
        }
        i++;
     }
     return occupancy_map_;
 }

 std::vector<std::vector<int>> map_loader::createBloatedMap()
 {
    std::vector<std::vector<int>> bloated_map(115, std::vector<int>(115,0));
    for (const auto& cell : obstacle_cells_)
    {
        bloated_map[cell.x][cell.y] = 1;
        for (int j=1; j<6; j++)
        {
            if (!(cell.x + j > 114))
            {
                bloated_map[cell.x+j][cell.y] = 1;
            }
            if (!(cell.x - j < 0))
            {
                bloated_map[cell.x-j][cell.y] = 1;
            }
            if (!(cell.y + j >114))
            {
                bloated_map[cell.x][cell.y+j] = 1;
            }
            if (!(cell.y - j < 0))
            {
                bloated_map[cell.x][cell.y-j] = 1;
            }
        }
    } // 66 47.7707
    return bloated_map;
 }
