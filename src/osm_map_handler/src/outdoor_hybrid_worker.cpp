// One bounded, forward-only request. Map/admission adapter; the search itself
// is HybridAStar::Algorithm from the existing shared HYAS library.
#include <algorithm.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

using boost::property_tree::ptree;
using namespace HybridAStar;
using Pose = std::array<double, 3>;
using Point = std::array<double, 2>;

namespace {
struct QuietOutput {
  std::ostringstream buffer;
  std::streambuf* previous;
  QuietOutput() : previous(std::cout.rdbuf(buffer.rdbuf())) {}
  ~QuietOutput() { std::cout.rdbuf(previous); }
};
double wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
double normalized(double x) { double t=std::fmod(x, 2*M_PI); return t < 0 ? t+2*M_PI : t; }
std::vector<double> values(const ptree& list, bool nullable=false) {
  std::vector<double> result;
  for (const auto& item : list) {
    if (nullable && item.second.data()=="null") result.push_back(std::numeric_limits<double>::quiet_NaN());
    else {
      double value=item.second.get_value<double>();
      if (!std::isfinite(value)) throw std::runtime_error("nonfinite input");
      result.push_back(value);
    }
  }
  return result;
}
Pose pose(const ptree& input) {
  const auto a=values(input);
  if (a.size()!=3) throw std::runtime_error("invalid pose");
  return {{a[0],a[1],a[2]}};
}

struct Space {
  Pose start, capture;
  std::vector<std::array<double, 6>> roads;
  std::vector<double> ranges;
  std::vector<Point> body, buffer;
  double xmin, xmax, ymin, ymax, angle_min, increment, range_min, range_max;
  double resolution, lower_x, lower_y;
  int size, road_rejected=0, boundary_rejected=0, free_cells=0;
  std::map<std::string,int> scan_rejected;
  nav_msgs::OccupancyGrid::Ptr grid;

  Space(const ptree& request) : start(pose(request.get_child("start"))),
      capture(pose(request.get_child("scan.pose"))) {
    resolution=request.get<double>("resolution", .1);
    const double extent=request.get<double>("extent", 8.);
    if (!(resolution>=.08 && resolution<=.25 && extent>=6 && extent<=10)) throw std::runtime_error("invalid window");
    size=int(std::ceil(2*extent/resolution));
    lower_x=start[0]-extent; lower_y=start[1]-extent;
    auto vertices=request.get_child("vertices");
    if (vertices.size()!=2) throw std::runtime_error("invalid vertices");
    const auto vx=values(vertices.front().second), vy=values(vertices.back().second);
    if (vx.size()!=4 || vy.size()!=4) throw std::runtime_error("invalid footprint");
    xmin=*std::min_element(vx.begin(),vx.end()); xmax=*std::max_element(vx.begin(),vx.end());
    ymin=*std::min_element(vy.begin(),vy.end()); ymax=*std::max_element(vy.begin(),vy.end());
    if (!(xmax>xmin && ymax>ymin && xmax-xmin<2 && ymax-ymin<2)) throw std::runtime_error("invalid footprint size");
    for (int extra=0; extra<2; ++extra) {
      const double m=extra ? .12 : 0.;
      const int nx=std::max(1,int(std::ceil((xmax-xmin+2*m)/.12)));
      const int ny=std::max(1,int(std::ceil((ymax-ymin+2*m)/.12)));
      for(int y=0;y<=ny;++y) for(int x=0;x<=nx;++x)
        (extra ? buffer : body).push_back({{xmin-m+(xmax-xmin+2*m)*x/nx, ymin-m+(ymax-ymin+2*m)*y/ny}});
    }
    ranges=values(request.get_child("scan.ranges"),true);
    angle_min=request.get<double>("scan.angle_min"); increment=request.get<double>("scan.increment");
    range_min=request.get<double>("scan.range_min"); range_max=request.get<double>("scan.range_max");
    if (!(increment>0 && increment<.2 && range_min>=0 && range_min<range_max && ranges.size()>10 && ranges.size()<10000))
      throw std::runtime_error("invalid scan");
    for (const auto& row : request.get_child("roads")) {
      auto a=values(row.second);
      if(a.size()!=6 || !(a[4]>0) || std::hypot(a[2]-a[0],a[3]-a[1])<.01) throw std::runtime_error("invalid road");
      roads.push_back({{a[0],a[1],a[2],a[3],a[4],a[5]}});
    }
    if(roads.empty() || roads.size()>256) throw std::runtime_error("invalid road count");
    grid.reset(new nav_msgs::OccupancyGrid);
    grid->info.width=size; grid->info.height=size; grid->info.resolution=resolution;
    grid->info.origin.position.x=lower_x; grid->info.origin.position.y=lower_y;
    grid->info.origin.orientation.w=1.; grid->data.resize(size*size,-1);
    // Fused local evidence grid. Unknown is never labelled as observed free.
    for(int y=0;y<size;++y) for(int x=0;x<size;++x) {
      const Point p={{lower_x+(x+.5)*resolution,lower_y+(y+.5)*resolution}};
      bool clear,blocked,near; query(p,clear,blocked,near);
      if(!onRoad(p)) grid->data[y*size+x]=100;
      else if(clear) { grid->data[y*size+x]=0; free_cells++; }
      else if(blocked) grid->data[y*size+x]=100;
    }
  }

  bool onRoad(Point p, double yaw=0, bool legal=false) const {
    for(const auto& r : roads) {
      const double dx=r[2]-r[0],dy=r[3]-r[1];
      const double u=std::max(0.,std::min(1.,((p[0]-r[0])*dx+(p[1]-r[1])*dy)/(dx*dx+dy*dy)));
      if(std::hypot(p[0]-r[0]-u*dx,p[1]-r[1]-u*dy)<=r[4] &&
          (!legal || r[5]==0 || (std::cos(yaw)*dx+std::sin(yaw)*dy)*r[5]>=-.1)) return true;
    }
    return false;
  }
  bool inCurrent(Point p) const {
    const double dx=p[0]-start[0],dy=p[1]-start[1],c=std::cos(start[2]),s=std::sin(start[2]);
    const double x=c*dx+s*dy,y=-s*dx+c*dy;
    // Grid poses use float. This two-micrometre numerical tolerance is not a
    // physical blind-zone/body expansion.
    return x>=xmin-2.e-6 && x<=xmax+2.e-6 && y>=ymin-2.e-6 && y<=ymax+2.e-6;
  }
  void query(Point p, bool& clear, bool& blocked, bool& near) const {
    const double dx=p[0]-capture[0],dy=p[1]-capture[1],c=std::cos(capture[2]),s=std::sin(capture[2]);
    const double x=c*dx+s*dy,y=-s*dx+c*dy,r=std::hypot(x,y);
    double angle=std::atan2(y,x);
    angle+=std::ceil((angle_min-angle)/(2*M_PI))*2*M_PI;
    const int index=int(std::nearbyint((angle-angle_min)/increment));
    const double measured=index>=0 && index<int(ranges.size()) ? ranges[index] : std::numeric_limits<double>::quiet_NaN();
    const bool valid=std::isfinite(measured) && measured>=range_min && measured<=range_max;
    clear=valid && r>=range_min && r+.06<measured;
    blocked=valid && r+.06>=measured; near=r<range_min;
  }
  bool valid(float x, float y, float yaw) {
    const Point center={{lower_x+x*resolution,lower_y+y*resolution}};
    if(!onRoad(center,yaw,true)) { road_rejected++; return false; }
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(int extra=0;extra<2;++extra) for(const Point& q : (extra ? buffer : body)) {
      const Point p={{center[0]+c*q[0]-s*q[1],center[1]+s*q[0]+c*q[1]}};
      if(p[0]<lower_x || p[1]<lower_y || p[0]>=lower_x+size*resolution || p[1]>=lower_y+size*resolution) {
        boundary_rejected++; return false;
      }
      if(!onRoad(p)) { road_rejected++; return false; }
      if(inCurrent(p)) continue;
      bool clear,blocked,near; query(p,clear,blocked,near);
      if(blocked) { scan_rejected["observed_obstacle_or_occlusion"]++; return false; }
      if(!clear && !(extra && near)) {
        scan_rejected[extra ? "clearance_buffer_unobserved" : (near ? "swept_body_enters_near_blind_zone" : "swept_body_unobserved")]++;
        return false;
      }
    }
    return true;
  }
  Node3D node(Pose p) const { return Node3D((p[0]-lower_x)/resolution,(p[1]-lower_y)/resolution,normalized(p[2]),0,0,nullptr); }
};

std::vector<Pose> trace(const Node3D* end, const Space& space) {
  std::unordered_set<const Node3D*> seen;
  std::vector<Pose> coarse;
  for(auto node=end;node;node=node->getPred()) {
    if(!seen.insert(node).second || seen.size()>30000) throw std::runtime_error("predecessor cycle");
    coarse.push_back({{space.lower_x+node->getX()*space.resolution,
                      space.lower_y+node->getY()*space.resolution,wrap(node->getT())}});
  }
  std::reverse(coarse.begin(),coarse.end());
  if(!coarse.empty()) coarse.front()=space.start;
  std::vector<Pose> dense;
  if(coarse.empty()) return dense;
  dense.push_back(coarse[0]);
  for(size_t i=1;i<coarse.size();++i) {
    const Pose a=coarse[i-1],b=coarse[i];
    const double chord=std::hypot(b[0]-a[0],b[1]-a[1]),dyaw=wrap(b[2]-a[2]);
    if(chord<1.e-6) continue;
    const double arc=std::abs(dyaw)<1.e-5 ? chord : chord*std::abs(dyaw)/(2*std::sin(std::abs(dyaw)/2));
    const double curvature=dyaw/arc;
    const int count=std::max(1,int(std::ceil(arc/.08)));
    for(int j=1;j<=count;++j) {
      const double d=arc*j/count,t=a[2]+curvature*d;
      Pose q={{a[0]+(std::abs(curvature)<1.e-8 ? d*std::cos(a[2]) : (std::sin(t)-std::sin(a[2]))/curvature),
               a[1]+(std::abs(curvature)<1.e-8 ? d*std::sin(a[2]) : (std::cos(a[2])-std::cos(t))/curvature),wrap(t)}};
      dense.push_back(j==count ? b : q);
    }
  }
  return dense;
}

void respond(long long id, bool success, const std::string& reason, int anchor,
             const std::vector<Pose>& path, const SearchOptions& options, const Space* space, double ms) {
  std::cout.precision(12);
  std::cout << "{\"id\":" << id << ",\"result\":{\"success\":" << (success ? "true":"false")
      << ",\"reason\":\"" << reason << "\",\"anchor\":" << anchor << ",\"path\":[";
  for(size_t i=0;i<path.size();++i) {
    if(i) std::cout << ',';
    std::cout << '[' << path[i][0] << ',' << path[i][1] << ',' << path[i][2] << ']';
  }
  std::cout << "],\"stats\":{\"algorithm\":\"shared_cpp_hybrid_astar_forward\",\"expanded\":" << options.expanded
      << ",\"analytic_shots\":" << options.shots << ",\"elapsed_ms\":" << ms;
  if(space) {
    std::cout << ",\"map_resolution_m\":" << space->resolution << ",\"map_width\":" << space->size
        << ",\"observed_free_cells\":" << space->free_cells << ",\"road_rejected\":" << space->road_rejected
        << ",\"map_boundary_rejected\":" << space->boundary_rejected << ",\"scan_rejected\":{";
    bool first=true;
    for(const auto& item : space->scan_rejected) {
      if(!first) std::cout << ','; first=false;
      std::cout << '\"' << item.first << "\":" << item.second;
    }
    std::cout << '}';
  }
  std::cout << "}}}" << std::endl;
}
}

int main() {
  using Clock=std::chrono::steady_clock;
  const auto started=Clock::now();
  SearchOptions options;
  long long id=0;
  try {
    std::string input;
    std::getline(std::cin,input);
    if(input.size()>1048576) throw std::runtime_error("request too large");
    ptree root; std::istringstream stream(input); boost::property_tree::read_json(stream,root);
    id=root.get<long long>("id");
    const ptree& request=root.get_child("request");
    const double budget=request.get<double>("budget_sec",1.0),radius=request.get<double>("radius");
    options.max_iterations=request.get<int>("max_iterations",12000);
    if(!(budget>=.02 && budget<=2. && radius>=1 && radius<=10 && options.max_iterations>0 && options.max_iterations<=50000))
      throw std::runtime_error("invalid search limits");
    options.deadline=started+std::chrono::milliseconds(int(budget*1000));
    Space space(request);
    options.radius=radius/space.resolution; options.step=.36/space.resolution;
    options.collision_step=.08/space.resolution; options.max_length=20./space.resolution;
    std::vector<int> anchors;
    for(const auto& item : request.get_child("goals")) {
      anchors.push_back(item.second.get<int>("anchor"));
      options.goals.push_back(space.node(pose(item.second.get_child("pose"))));
    }
    if(anchors.empty() || anchors.size()>3) throw std::runtime_error("invalid goals");
    Node3D start=space.node(space.start);
    CollisionDetection collision([&space](float x,float y,float t) { return space.valid(x,y,t); });
    collision.updateGrid(space.grid);
    Node3D* solution=nullptr;
    std::vector<Node3D> nodes(size_t(space.size)*space.size*Constants::headings);
    Visualize visual(false);
    // Legacy diagnostics must never corrupt the single JSON response on stdout.
    {
      QuietOutput quiet;
      if(collision.isTraversable(&start))
        solution=Algorithm::hybridAStar(start,options.goals[0],nodes.data(),nullptr,space.size,space.size,
                                      collision,nullptr,visual,&options);
      else if(space.road_rejected) options.reason="initial_footprint_outside_road_region";
      else if(space.boundary_rejected) options.reason="initial_footprint_outside_search_grid";
      else options.reason="initial_footprint_scan_unobserved_or_obstructed";
    }
    auto path=trace(solution,space);
    if(path.size()<2) {
      if(solution) options.reason="degenerate_solution";
      solution=nullptr; path.clear();
    }
    respond(id,solution,options.reason,solution ? anchors.at(options.goal_index) : -1,path,options,&space,
            std::chrono::duration<double,std::milli>(Clock::now()-started).count());
  } catch(const std::exception& error) {
    std::cerr << "Outdoor Hybrid A* request failed: " << error.what() << std::endl;
    respond(id,false,"invalid_request_or_search_error",-1,{},options,nullptr,
            std::chrono::duration<double,std::milli>(Clock::now()-started).count());
  }
  return 0;
}
