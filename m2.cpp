/* 
 * Copyright 2024 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/callback.hpp"
#include "ezgl/camera.hpp"
#include "ezgl/canvas.hpp"
#include "ezgl/color.hpp"
#include "ezgl/control.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/point.hpp"
#include "ezgl/rectangle.hpp"
#include "StreetsDatabaseAPI.h"
#include <math.h>
#include <sstream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <iostream>
#include <cstddef>
#include <iomanip>
#include <numeric>
#include <vector>

//Global Variables
struct Intersection_data{
   LatLon position;
   std::string name;
   bool highlight = false;
};

struct NaturalFeature_data{
   std::vector<LatLon> points;
   std::string name;
   bool closed;
   int featureType;
};

struct POI_data{
   LatLon position;
   std::string name;
   bool high_light = false;
};

std::vector<Intersection_data> intersections_vec;
std::vector<std::vector<LatLon>> LatLon_of_a_street;
std::vector<NaturalFeature_data> naturalFeatures;
std::vector<POI_data> POI_vec;
std::vector<int> StreetSegment_Major;
std::vector<std::string> OneWayDir;

double max_lat, min_lat, max_lon, min_lon, avg_lat;
bool DarkMode = false;
bool OnlyRoad = false;
bool findClosestStarbucks = false;

//Helper Functions declaration
double x_from_lon(double lon);
double y_from_lat(double lat);
double lon_from_x(double x);
double lat_from_y(double y);
void draw_main_canvas (ezgl::renderer *g);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void toggle_pink(GtkWidget* /*widget*/, ezgl::application* application);
void toggle_road(GtkWidget* /*widget*/, ezgl::application* application);
void toggle_POI(GtkWidget* /*widget*/, ezgl::application* application);
void initial_setup(ezgl::application* application, bool /*new_window*/);

void drawMap() {
   // Set up the ezgl graphics window and hand control to it, as shown in the 
   // ezgl example program. 
   // This function will be called by both the unit tests (ece297exercise) 
   // and your main() function in main/src/main.cpp.
   // The unit tests always call loadMap() before calling this function
   // and call closeMap() after this function returns.

   const auto start = std::chrono::high_resolution_clock::now();

   //loading intersection and calculating latlon
   max_lat = getIntersectionPosition(0).latitude();
   min_lat = max_lat;
   max_lon = getIntersectionPosition(0).longitude();
   min_lon = max_lon;
   intersections_vec.resize(getNumIntersections());

   for(int id = 0; id < getNumIntersections(); id++){
      intersections_vec[id].position = getIntersectionPosition(id);
      intersections_vec[id].name = getIntersectionName(id);

      max_lat = std::max(max_lat, intersections_vec[id].position.latitude());
      min_lat = std::min(min_lat, intersections_vec[id].position.latitude());
      max_lon = std::max(max_lon, intersections_vec[id].position.longitude());
      min_lon = std::min(min_lon, intersections_vec[id].position.longitude());
   }
   avg_lat = (max_lat + min_lat) / 2;
   
   POI_vec.resize(getNumPointsOfInterest());

   for(int id = 0; id < getNumPointsOfInterest(); id++){
      POI_vec[id].position = getPOIPosition(id);
      POI_vec[id].name = getPOIName(id);
   }


   //load vector LatLon_of_a_street
   LatLon_of_a_street.resize(getNumStreetSegments()); 

   for(int streetSegmentId = 0; streetSegmentId < getNumStreetSegments(); streetSegmentId++){
      LatLon currentPositition = getIntersectionPosition(getStreetSegmentInfo(streetSegmentId).from);
      LatLon_of_a_street[streetSegmentId].push_back(currentPositition);
      for(int NumCurvePoint = 0; NumCurvePoint < getStreetSegmentInfo(streetSegmentId).numCurvePoints; NumCurvePoint++){
         currentPositition = getStreetSegmentCurvePoint(NumCurvePoint, streetSegmentId);
         LatLon_of_a_street[streetSegmentId].push_back(currentPositition);
      }
      currentPositition = getIntersectionPosition(getStreetSegmentInfo(streetSegmentId).to);
      LatLon_of_a_street[streetSegmentId].push_back(currentPositition);
   }

   //loading vector naturalFeatures
   naturalFeatures.resize(getNumFeatures());

   for(int id = 0; id < getNumFeatures(); id++){
      naturalFeatures[id].name = getFeatureName(id);
      naturalFeatures[id].featureType = getFeatureType(id);
      int numPoints = getNumFeaturePoints(id);
      if(getFeaturePoint(0,id) == getFeaturePoint(numPoints-1, id)){
         naturalFeatures[id].closed = true;
      } else{
         naturalFeatures[id].closed = false;
      }
      for(int point = 0; point < numPoints; point++){
         naturalFeatures[id].points.push_back(getFeaturePoint(point, id));
      }
   }

   // load vector StreetSegment_Major
   StreetSegment_Major.resize(getNumStreetSegments());

   for(int id = 0; id < getNumStreetSegments(); id++){
      int size = findIntersectionsOfStreet(getStreetSegmentInfo(id).streetID).size();
      int speed = getStreetSegmentInfo(id).speedLimit;
      if((size >= 100 && size < 5000)|| speed >= 30){
         StreetSegment_Major[id] = 1;
      } else{
         StreetSegment_Major[id] = 0;
      }
   }

   //load vector OneWayDir
   OneWayDir.resize(getNumStreetSegments());

   for (int streetSegmentId = 0; streetSegmentId < getNumStreetSegments(); streetSegmentId++){
      if(getStreetSegmentInfo(streetSegmentId).oneWay){
         int id_from = getStreetSegmentInfo(streetSegmentId).from;
         int id_to = getStreetSegmentInfo(streetSegmentId).to;
         double lat = getIntersectionPosition(id_to).latitude() - getIntersectionPosition(id_from).latitude();
         double lat_abs = abs(lat);
         double lon = getIntersectionPosition(id_to).longitude() - getIntersectionPosition(id_from).longitude();
         double lon_abs = abs(lon);
         if(lat_abs >= lon_abs){
            if(lat >= 0){
               OneWayDir[streetSegmentId] = "^";
            } else{
               OneWayDir[streetSegmentId] = "v";
            }
         } else{
            if(lon >= 0){
               OneWayDir[streetSegmentId] = ">";
            } else{
               OneWayDir[streetSegmentId] = "<";
            }
         }
      }
   }


   ezgl::application::settings settings;
   settings.main_ui_resource = "libstreetmap/resources/main.ui";
   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";

   ezgl::application application(settings);

   ezgl::rectangle initial_world({x_from_lon(min_lon),y_from_lat(min_lat)},{x_from_lon(max_lon),y_from_lat(max_lat)});

   application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

   auto end = std::chrono::high_resolution_clock::now();
 
      std::chrono::duration<double> diff = end - start;
 
        std::cout << "Time: " << diff.count() << '\n';

   application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}


//helper function implementations

double x_from_lon(double lon) {
  return lon * kDegreeToRadian * kEarthRadiusInMeters * cos(avg_lat * kDegreeToRadian);
}

double y_from_lat(double lat) {
  return kEarthRadiusInMeters * kDegreeToRadian * lat;
}

double lon_from_x(double x){
   return x / kDegreeToRadian / kEarthRadiusInMeters / cos(avg_lat * kDegreeToRadian);
}
double lat_from_y(double y){
   return y / kEarthRadiusInMeters / kDegreeToRadian;
}

void draw_main_canvas (ezgl::renderer *g){
   //check if DarkMode is on
   if(DarkMode){
      ezgl::rectangle visible_world = g->get_visible_world();
      g->set_color(0,0,0);
      g->fill_rectangle(visible_world);
   }

   //drawing
   if( g->get_visible_world().width() < 30000){
      for(int i = 0; i < intersections_vec.size(); i++){	
         if(intersections_vec[i].highlight){
            g->set_color(ezgl::ORANGE);
         } else{
            g->set_color(ezgl::BLACK);
         }

         double x = x_from_lon(intersections_vec[i].position.longitude());
         double y = y_from_lat(intersections_vec[i].position.latitude());

         double width = 5;
         double height = width;

         g->fill_rectangle({x - (width / 2), y - (height / 2)}, width, height);
      
      }
   }

   if( g->get_visible_world().width() >= 30000){
      g->set_line_width(0);
   } 
   

   for (int streetSegmentId = 0; streetSegmentId < getNumStreetSegments(); streetSegmentId++){
      double start_x = 0;
      double start_y = 0;
      for (int points = 0; points < getStreetSegmentInfo(streetSegmentId).numCurvePoints + 1; points++){
         if(StreetSegment_Major[streetSegmentId] == 1){
            g->set_color(ezgl::YELLOW);
         } else if(getStreetSegmentInfo(streetSegmentId).oneWay){
            g->set_color(ezgl::RED);
         } else{
            g->set_color(ezgl::BLACK);
         }
         if(g->get_visible_world().width() >= 30000 && StreetSegment_Major[streetSegmentId] == 0 && !getStreetSegmentInfo(streetSegmentId).oneWay){
            continue;
         } else{
            start_x = x_from_lon(LatLon_of_a_street[streetSegmentId][points].longitude());
            start_y = y_from_lat(LatLon_of_a_street[streetSegmentId][points].latitude());
            double end_x = x_from_lon(LatLon_of_a_street[streetSegmentId][points+1].longitude());
            double end_y = y_from_lat(LatLon_of_a_street[streetSegmentId][points+1].latitude());
            ezgl::point2d startPoint(start_x,start_y);
            ezgl::point2d endPoint(end_x,end_y);
            g->draw_line(startPoint,endPoint);
            if(points <= 2){
               g->draw_text(startPoint, OneWayDir[streetSegmentId], 20, 20);
            }
         }
      }
      g->set_color(ezgl::PINK);
      ezgl::point2d Point(start_x,start_y);
      g->draw_text(Point, getStreetName(getStreetSegmentInfo(streetSegmentId).streetID), 50, 50);
   }

   if(!OnlyRoad){
      for(int id = 0; id < getNumFeatures(); id ++){
         int i = naturalFeatures[id].featureType;
         if (i == 1 || i == 5 || i == 7 || i == 8){
            g->set_color(ezgl::GREEN);
         } else if(i == 3 || i == 4 || i == 9){
            g->set_color(ezgl::BLUE);
         } else if(i == 2){
            g->set_color(ezgl::SADDLE_BROWN);
         } else if(i == 6){
            g->set_color(ezgl::GREY_55);
         } else if(i == 10){
            g->set_color(ezgl::WHITE);
         } else{
            g->set_color(ezgl::RED);
         }

         if(g->get_visible_world().width() >= 30000 && i == 6){
            continue;
         }else if(naturalFeatures[id].closed && naturalFeatures[id].points.size() > 1){
            std::vector<ezgl::point2d> points2d_vec;
            for(int b = 0; b < naturalFeatures[id].points.size(); b++){
               double point_x = x_from_lon(naturalFeatures[id].points[b].longitude());
               double point_y = y_from_lat(naturalFeatures[id].points[b].latitude());
               ezgl::point2d point_2d(point_x,point_y);
               points2d_vec.push_back(point_2d);
            }
            g->fill_poly(points2d_vec);
         } else{
            g->set_line_width(5);
            if(g->get_visible_world().width () >= 10 || g->get_visible_screen().width() >= 10){
               g->set_line_width(2);
            }
            for(int a = 0; a < naturalFeatures[id].points.size() - 1; a++){
               double start_x = x_from_lon(naturalFeatures[id].points[a].longitude());
               double start_y = y_from_lat(naturalFeatures[id].points[a].latitude());
               double end_x = x_from_lon(naturalFeatures[id].points[a+1].longitude());
               double end_y = y_from_lat(naturalFeatures[id].points[a+1].latitude());
               ezgl::point2d startPoint(start_x,start_y);
               ezgl::point2d endPoint(end_x,end_y);
               g->draw_line(startPoint,endPoint);
            }
         }
      }
      if(g->get_visible_world().width() < 30000 ){
         for(int id = 0; id < getNumPointsOfInterest(); id++){
            if(POI_vec[id].high_light){
               g->set_color(ezgl::FIRE_BRICK);
            } else{
               g->set_color(ezgl::DARK_KHAKI);
            }

            double x = x_from_lon(POI_vec[id].position.longitude());
            double y = y_from_lat(POI_vec[id].position.latitude());

            double width = 5;
            double height = width;

            g->fill_rectangle({x - (width / 2), y - (height / 2)}, width, height);
            g->set_color(ezgl::LIME_GREEN);
            ezgl::point2d PointOI(x,y);
            g->draw_text(PointOI, getPOIName(id), 50, 50);
         }
      }
   }
}

void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y){
   std::cout << "Mouse clicked at (" << x << "," << y << ")\n";

   LatLon pos = LatLon(lat_from_y(y) , lon_from_x(x));
   if(findClosestStarbucks){
      int inter_id = findClosestPOI(pos,"Starbucks");
      std::cout << "Closest Starbucks: " << POI_vec[inter_id].name << "\n";
      POI_vec[inter_id].high_light = !POI_vec[inter_id].high_light;
      std::stringstream ss;
      ss<< "POI: " << POI_vec[inter_id].name;
      app->update_message(ss.str()); // message at bottom
      app->refresh_drawing();// force a redraw
   } else{
      int inter_id = findClosestIntersection(pos);
      std::cout << "Closest Intersection: " << intersections_vec[inter_id].name << "\n";
      intersections_vec[inter_id].highlight = !intersections_vec[inter_id].highlight;
      std::stringstream ss;
      ss<< "Intersection: " << intersections_vec[inter_id].name;
      app->update_message(ss.str()); // message at bottom
      app->refresh_drawing();// force a redraw
   }
}

void toggle_pink(GtkWidget* /*widget*/, ezgl::application* application){
   DarkMode = !DarkMode;
   application->refresh_drawing();
}

void toggle_road(GtkWidget* /*widget*/, ezgl::application* application){
   OnlyRoad = !OnlyRoad;
   application->refresh_drawing();
}

void toggle_POI(GtkWidget* /*widget*/, ezgl::application* application){
   findClosestStarbucks = !findClosestStarbucks;
   application->refresh_drawing();
}

void initial_setup(ezgl::application* application, bool /*new_window*/){
   //Create a Pinkify button in the sixth row (starting from 0)
   application->create_button("DarkMode", 6, toggle_pink);
   application->create_button("OnlyRoads", 7, toggle_road);
   application->create_button("findClosestStarbucks", 8, toggle_POI);
}
