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

#include <math.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
using namespace std;

// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection
// data that is higher-level than the raw OSM data).
// This file name will always end in ".streets.bin" and you
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the
// name of the ".osm.bin" file that matches your map -- just change
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments;
std::multimap<std::string, StreetIdx> street_name_map;
std::unordered_map<OSMID, int> unmapWay;
std::unordered_map<OSMID, int> unmapNode;
std::multimap<StreetIdx, IntersectionIdx> mapStInt;
std::vector<std::vector<StreetSegmentIdx>> street_streetSegments;
std::vector<double> vecTime;
std::unordered_map<int, vector<IntersectionIdx>> unmapIntSetOfSt;

bool loadMap(std::string map_streets_database_filename) { // this funciton will load all lower-level APIs and checks if they are loaded successfully.
//This funciton also preload some data structures(global variable) in order to pass the performance-speed tests  
  bool load_successful = false;  // Indicates whether the map has loaded successfully
  bool load_streets = false;
  bool load_osm = false;
  std::cout << "loadMap: " << map_streets_database_filename << std::endl;

  // loading lower-level APIs
  load_streets = loadStreetsDatabaseBIN(map_streets_database_filename);

  std::string map_osm_database_filename =
      map_streets_database_filename;  // modified to a proper name
  size_t pos = map_osm_database_filename.rfind(".streets.bin");
  if (pos != std::string::npos) {
    map_osm_database_filename.replace(pos, 12, ".osm.bin");
  }
  load_osm = loadOSMDatabaseBIN(map_osm_database_filename);

  // check if load is successful, if not, return false
  if (!(load_streets && load_osm)) {
    return false;
  }
  //
  // Load your map related data structures here.
  //
  intersection_street_segments.resize(getNumIntersections());  // create empty vector to store every intersection

  for (int intersections = 0; intersections < getNumIntersections();++intersections) {  // iterate through all intersection
    for (int i = 0; i < getNumIntersectionStreetSegment(intersections);++i) {  // iterate through all segments in the intersection
      int streetSegmentID = getIntersectionStreetSegment(i, intersections);
      intersection_street_segments[intersections].push_back(streetSegmentID);
    }
  }//intersection_street_segments is now a double vector. In the first vector, the key is the street id. Each space in the
  //first vector stores a vector that contains all street segments corresponding to the streets they belong to. 

  for (StreetIdx street_id = 0; street_id < getNumStreets(); street_id++) {
    std::string result = getStreetName(street_id);
    std::transform(result.begin(), result.end(), result.begin(),::tolower);  // remove spaces and make the string lowercase
    for (int i = 0; i < result.length(); i++) {
      if (result[i] == ' ') {
        result.erase(result.begin() + i);
        i--;
      }
    }
    street_name_map.insert(std::make_pair(result, street_id));// street_name_map is now a multimap using the string (its street name) as key.
    // In each space of the street_name_map, there is a street id corresponding to the street name.
  }
  //pair OSMway_id and OSMway_index 
  for (int i = 0; i < getNumberOfWays(); i++) {
    unmapWay.insert(pair<OSMID, int>(getWayByIndex(i)->id(), i));
  }
  //pair OSMnode_id and OSMnode_index 
  for (int i = 0; i < getNumberOfNodes(); i++) {
    unmapNode.insert(pair<OSMID, int>(getNodeByIndex(i)->id(), i));
  }
  //pair the street_id and the two intersections of one street segment
  for (int i = 0; i < getNumStreetSegments(); i++) {
    mapStInt.insert(make_pair(getStreetSegmentInfo(i).streetID, getStreetSegmentInfo(i).from));
    mapStInt.insert(make_pair(getStreetSegmentInfo(i).streetID, getStreetSegmentInfo(i).to));
  }

  //pair street_id and the set of all the intersections of this street
  for (int i = 0; i < getNumStreets(); i++){
    std::vector<IntersectionIdx> intSecIdx;
    auto range = mapStInt.equal_range(i);
    for (auto it = range.first; it != range.second; it++) {
    if (find(intSecIdx.begin(), intSecIdx.end(), it->second) ==
      intSecIdx.end()) {
        intSecIdx.push_back(it->second);
      }
    }
      unmapIntSetOfSt.insert(make_pair(i, intSecIdx));
  }
  
  // all street_ids and their streetSegments
  if(street_streetSegments.empty()){
    street_streetSegments.resize(getNumStreets());
    for (StreetSegmentIdx i = 0; i< getNumStreetSegments(); i++){
       StreetIdx streetid = getStreetSegmentInfo(i).streetID;
        street_streetSegments[streetid].push_back(i);
    }
  }

    //store the corresponding travel time of each street segment in one vector
    for (int i = 0; i < getNumStreetSegments(); i++){
      vecTime.resize(getNumStreetSegments());
      vecTime[i] = findStreetSegmentLength(i)/(getStreetSegmentInfo(i).speedLimit);
    }




  load_successful = true;  // Make sure this is updated to reflect whether loading the map succeeded or failed

  return load_successful;
}

void closeMap() { //In this function, it closes all data bases and clear all global variables
  // Clean-up your map related data structures here
  closeStreetDatabase();  // close street database
  closeOSMDatabase();     // close OSM database
  intersection_street_segments.clear();// close intersection_street_segments
  street_name_map.clear();// close street_name_map
  unmapWay.clear();// close unmapWay
  unmapNode.clear();// close unmapNode
  mapStInt.clear();// close mapStInt
  street_streetSegments.clear();// close street_streetSegments
  vecTime.clear();// close vecTime
  unmapIntSetOfSt.clear();;// close unmapIntSetOfSt
}

//filling up implement below

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {//this function take intersection id as an input, will output a vector containing all the
  //street segments belongs to this street
  return intersection_street_segments[intersection_id];//using global variable intersection_street_segments which is preloaded in loadmap() here
}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {// this function will take a street segment id as input to output its length
  StreetSegmentInfo segmentInfo = getStreetSegmentInfo(street_segment_id);
  double street_segment_length_double = 0;
  IntersectionIdx pointFrom = segmentInfo.from;
  IntersectionIdx pointTo = segmentInfo.to;
  int numCurpts = segmentInfo.numCurvePoints;
  LatLon posFrom = getIntersectionPosition(pointFrom);
  LatLon posTo = getIntersectionPosition(pointTo);
  if (numCurpts ==0) {  // no curve points, only add the distance between from and to into street_segment_length_double
    street_segment_length_double =findDistanceBetweenTwoPoints(posFrom,posTo);
  } else if (numCurpts >=1) {  // have curve points to be add into street_segment_length_double
    street_segment_length_double += findDistanceBetweenTwoPoints(posFrom,getStreetSegmentCurvePoint(0, street_segment_id));
    street_segment_length_double += findDistanceBetweenTwoPoints(getStreetSegmentCurvePoint(numCurpts - 1,street_segment_id),posTo);
    for (int i = 0; i < numCurpts - 1;++i) {  // add the distances between curve points into// street_segment_length_double
      street_segment_length_double += findDistanceBetweenTwoPoints(getStreetSegmentCurvePoint(i, street_segment_id),getStreetSegmentCurvePoint(i + 1, street_segment_id));
    }
  }
  return street_segment_length_double;
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {//this function will take a prefix of a street as input 
// to output a vector containing all the street_ids. These ids are the ids from the streets which name contain the prefix
  std::vector<StreetIdx> match_street_id;
  std::string street_prefix_converted = street_prefix;
  if (street_prefix.length() == 0) {
    return match_street_id;
  }
  std::transform(street_prefix_converted.begin(), street_prefix_converted.end(), street_prefix_converted.begin(), ::tolower);  // remove spaces and make the string lowercase
  for (int i = 0; i < street_prefix_converted.length(); i++) {
    if (street_prefix_converted[i] == ' ') {
      street_prefix_converted.erase(street_prefix_converted.begin() + i);
      i--;
    }
  }
  auto it = street_name_map.lower_bound(street_prefix_converted);//using global variable street_name_map which is preloaded in loadmap() here
  while ((it != street_name_map.end() && (it->first).find(street_prefix_converted) == 0)) {
    match_street_id.push_back(it->second);
    it++;
  }
  return match_street_id;
}

double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){
    double dToR = kDegreeToRadian;
    double kERM = kEarthRadiusInMeters;
    double lat1 = point_1.latitude()*dToR;
    double lon1 = point_1.longitude()*dToR;
    double lat2 = point_2.latitude()*dToR;
    double lon2 = point_2.longitude()*dToR;
    double latavg = (lat1+lat2)/2.000;
    double cosLatavg = cos(latavg);
    double x1 = kERM*lon1*cosLatavg;
    double y1 = kERM*lat1;
    double x2 = kERM*lon2*cosLatavg;
    double y2 = kERM*lat2;
    double dy = y2-y1;
    double dx = x2-x1;
    return sqrt(dy*dy+dx*dx);
}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
  return vecTime[street_segment_id];
}

double findAngleBetweenStreetSegments(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id){
    StreetSegmentInfo segmentinfo1 = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo segmentinfo2 = getStreetSegmentInfo(dst_street_segment_id);
    
    if(src_street_segment_id == dst_street_segment_id) return 0;
    
    if((src_street_segment_id!=dst_street_segment_id)&&((segmentinfo1.to == segmentinfo2.from) || (segmentinfo1.from==segmentinfo2.to) ||(segmentinfo1.from==segmentinfo2.from) ||(segmentinfo1.to==segmentinfo2.to))){
        
        LatLon pointA = getIntersectionPosition(segmentinfo1.from);
        if(segmentinfo1.from==segmentinfo2.to || segmentinfo1.from==segmentinfo2.from){
          pointA = getIntersectionPosition(segmentinfo1.to);
        }
        if(segmentinfo1.numCurvePoints>0){
            if(segmentinfo1.to==segmentinfo2.from || segmentinfo1.to==segmentinfo2.to){pointA = getStreetSegmentCurvePoint(segmentinfo1.numCurvePoints-1, src_street_segment_id);}
            else{pointA = getStreetSegmentCurvePoint(0, src_street_segment_id);}
        }
        
        LatLon pointB = getIntersectionPosition(segmentinfo1.to);
        if(segmentinfo1.from==segmentinfo2.to || segmentinfo1.from==segmentinfo2.from){pointB = getIntersectionPosition(segmentinfo1.from);}
        
        LatLon pointC = getIntersectionPosition(segmentinfo2.to);
        if(segmentinfo2.to==segmentinfo1.from || segmentinfo2.to==segmentinfo1.to){pointC = getIntersectionPosition(segmentinfo2.from);}
        if(segmentinfo2.numCurvePoints>0){
            if(segmentinfo1.to==segmentinfo2.from || segmentinfo1.from==segmentinfo2.from){pointC = getStreetSegmentCurvePoint(0, dst_street_segment_id);}
            else{pointC = getStreetSegmentCurvePoint(segmentinfo2.numCurvePoints-1, dst_street_segment_id);}
        }
        
        double lenAB = findDistanceBetweenTwoPoints(pointA, pointB);
        double lenBC = findDistanceBetweenTwoPoints(pointB, pointC);
        double lenAC = findDistanceBetweenTwoPoints(pointA, pointC);
        if(lenAB==0 || lenBC==0 || lenAC==0){return 0;}
        double cosB = (lenAB*lenAB+lenBC*lenBC-lenAC*lenAC)/(2*lenAB*lenBC);
        if(cosB < -1.0 || cosB >1.0) return 0;
        double angle = acos(cosB);
        angle = M_PI - angle;
        return angle;
    }else{
        return NO_ANGLE;
    }
}


bool intersectionsAreDirectlyConnected(
    std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
  IntersectionIdx intersectid1 = intersection_ids.first;
  IntersectionIdx intersectid2 = intersection_ids.second;
  
  if (intersectid1 == intersectid2) {
    return true;
  }
  std::vector<StreetSegmentIdx> totalSegmentsIdx =
      findStreetSegmentsOfIntersection(intersectid1);
  for (StreetSegmentIdx segmentIdx :
       totalSegmentsIdx) {  // check all segments from first intersection
    StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentIdx);
    if ((segmentInfo.from == intersectid1 && segmentInfo.to == intersectid2) ||
        (segmentInfo.from == intersectid2 && segmentInfo.to == intersectid1)) {
      return true;
    }
  }
  return false;
}

IntersectionIdx findClosestIntersection(LatLon my_position) {
  IntersectionIdx closest = 0;
  for (int i = 1; i < getNumIntersections(); i++) {
    if (findDistanceBetweenTwoPoints(my_position, getIntersectionPosition(i)) <
        findDistanceBetweenTwoPoints(my_position,
                                     getIntersectionPosition(closest))) {
      closest = i;
    }
  }
  return closest;
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
  return unmapIntSetOfSt.find(street_id)->second;
}

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(
    std::pair<StreetIdx, StreetIdx> street_ids) {
  std::vector<IntersectionIdx> intersectionStreet1 =
      findIntersectionsOfStreet(street_ids.first);
  std::vector<IntersectionIdx> intersectionStreet2 =
      findIntersectionsOfStreet(street_ids.second);
  std::vector<IntersectionIdx> intersectionsOfTwoStreet;
  // find common intersections of two streets
  for (const auto& intersections : intersectionStreet1) {
    if (find(intersectionStreet2.begin(), intersectionStreet2.end(), intersections)!= intersectionStreet2.end()) {
      intersectionsOfTwoStreet.push_back(intersections);
    }
  }
  return intersectionsOfTwoStreet;
}


double findStreetLength(StreetIdx street_id){
    // get street segments vector from street_id
    std::vector <StreetSegmentIdx> streetSegments = street_streetSegments[street_id];
    double length = 0.0;
    for(int i=0; i<streetSegments.size(); i++){
        length += findStreetSegmentLength(streetSegments[i]);
    }
    return length;
}


POIIdx findClosestPOI(LatLon my_position, std::string poi_name) {
  POIIdx closest = -1;
  for (int i = 0; i < getNumPointsOfInterest(); i++) {
    if (getPOIName(i) == poi_name && closest < 0) {
      closest = i;
    } else if (getPOIName(i) == poi_name &&
               findDistanceBetweenTwoPoints(my_position, getPOIPosition(i)) <
                   findDistanceBetweenTwoPoints(my_position,
                                                getPOIPosition(closest))) {
      closest = i;
    }
  }
  return closest;
}

double findFeatureArea(FeatureIdx feature_id) {
  double area = 0;

  //check whether the feature is a closed polygon
  if (getFeaturePoint(0, feature_id).latitude() !=
          getFeaturePoint(getNumFeaturePoints(feature_id) - 1, feature_id)
              .latitude() ||
      getFeaturePoint(0, feature_id).longitude() !=
          getFeaturePoint(getNumFeaturePoints(feature_id) - 1, feature_id)
              .longitude()) {
    return area;
  }
  
  double latsum = 0;
  for (int i = 0; i < getNumFeaturePoints(feature_id) - 1; i++) {
    latsum += getFeaturePoint(i, feature_id).latitude();
  }
  double latavg = kDegreeToRadian * (latsum / (getNumFeaturePoints(feature_id) - 1));

  for (int i = 0; i < getNumFeaturePoints(feature_id) - 1; i++) {
    double lat1 = getFeaturePoint(i, feature_id).latitude() * kDegreeToRadian;
    double lon1 = getFeaturePoint(i, feature_id).longitude() * kDegreeToRadian;
    double lat2 =
        getFeaturePoint(i + 1, feature_id).latitude() * kDegreeToRadian;
    double lon2 =
        getFeaturePoint(i + 1, feature_id).longitude() * kDegreeToRadian;
    double x1 = kEarthRadiusInMeters * lon1 * cos(latavg);
    double y1 = kEarthRadiusInMeters * lat1;
    double x2 = kEarthRadiusInMeters * lon2 * cos(latavg);
    double y2 = kEarthRadiusInMeters * lat2;
    //trapezoidal integration
      area += 0.5 * ((x1 - x2) * (y1 + y2));
  }
  area = fabs(area);
  return area;
}

double findWayLength(OSMID way_id) {
  int index = -1;
  //find index of the way
  unordered_map<OSMID, int>::iterator it_way = unmapWay.find(way_id);
  if (it_way != unmapWay.end()) {
    index = it_way->second;
  } else {
    return -1;
  }

  const OSMWay* w = getWayByIndex(index);
  std::vector<OSMID> wNodeId = getWayMembers(w);
  double sum = 0;
  std::vector<OSMNode> wNode;
  if (wNodeId.size() > 0) {
    //store the position of each node on this way into vector wNode
    for (int i = 0; i < wNodeId.size(); i++) {
      unordered_map<OSMID, int>::iterator it = unmapNode.find(wNodeId[i]);
      if (it != unmapNode.end()) {
        wNode.push_back(*(getNodeByIndex(it->second)));
      }
    }

  //calculate the distance between each two nodes
    for (int i = 1; i < wNode.size(); i++) {
      sum += findDistanceBetweenTwoPoints(getNodeCoords(&wNode[i - 1]),
                                          getNodeCoords(&wNode[i]));
    }
    if (isClosedWay(w)) {
      sum += findDistanceBetweenTwoPoints(
          getNodeCoords(&wNode[0]), getNodeCoords(&wNode[wNode.size() - 1]));
    }
  }
  return sum;
}


std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    auto it = unmapNode.find(osm_id);
    string value = "";
    // check if the OSM_id is found
    if(it != unmapNode.end()){
        int index = it->second;
        const OSMNode* node = getNodeByIndex(index);
        for(int i=0; i<getTagCount(node); i++){
            if(getTagPair(node, i).first == key){
                value = getTagPair(node, i).second;
                break;
            }
        }
    }
    return value;
}
