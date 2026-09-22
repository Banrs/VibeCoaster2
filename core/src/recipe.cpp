#include "coaster/recipe.hpp"
#include <charconv>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace coaster {
namespace {
double parseNumber(const std::string& value){double x{};const auto [end,ec]=std::from_chars(value.data(),value.data()+value.size(),x);if(ec!=std::errc{}||end!=value.data()+value.size()||!std::isfinite(x))throw std::runtime_error("Invalid recipe number");return x;}
void range(double x,double lo,double hi,const char* field){if(!std::isfinite(x)||x<lo||x>hi)throw std::runtime_error(std::string("Unsupported recipe value: ")+field);}
}
void validateRecipe(const Recipe& r){
    if(r.version!=1)throw std::runtime_error("Unsupported recipe schema");
    if(r.style!="balanced"&&r.style!="flow"&&r.style!="intense")throw std::runtime_error("Unsupported ride style");
    range(r.plateau,200,220,"plateau");range(r.openingHeight,55,95,"openingHeight");range(r.camelbackHeight,200,240,"camelbackHeight");range(r.loopHeight,100,150,"loopHeight");range(r.immelmannHeight,75,120,"immelmannHeight");range(r.topSpeedKph,285,310,"topSpeedKph");range(r.activeSeconds,180,180,"activeSeconds");range(r.terminalSeconds,5,10,"terminalSeconds");
}
Recipe readRecipe(const std::filesystem::path& path){
    std::ifstream file(path);if(!file)throw std::runtime_error("Cannot open recipe");Recipe r;std::string line;std::set<std::string> seen;
    while(std::getline(file,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty()||line[0]=='#')continue;const auto eq=line.find('=');if(eq==std::string::npos)throw std::runtime_error("Recipe needs key=value");const auto key=line.substr(0,eq),value=line.substr(eq+1);if(!seen.insert(key).second)throw std::runtime_error("Duplicate recipe field");
        if(key=="style")r.style=value;
        else if(key=="version"){const double n=parseNumber(value);if(n!=1)throw std::runtime_error("Unsupported recipe version");r.version=1;}
        else if(key=="seed"){const double n=parseNumber(value);range(n,0,4294967295.,"seed");if(std::floor(n)!=n)throw std::runtime_error("Seed must be integer");r.seed=static_cast<unsigned>(n);}
        else {double* field=nullptr;if(key=="plateau")field=&r.plateau;else if(key=="openingHeight")field=&r.openingHeight;else if(key=="camelbackHeight")field=&r.camelbackHeight;else if(key=="loopHeight")field=&r.loopHeight;else if(key=="immelmannHeight")field=&r.immelmannHeight;else if(key=="topSpeedKph")field=&r.topSpeedKph;else if(key=="activeSeconds")field=&r.activeSeconds;else if(key=="terminalSeconds")field=&r.terminalSeconds;else throw std::runtime_error("Unknown recipe field: "+key);*field=parseNumber(value);}
    }if(!seen.contains("version"))throw std::runtime_error("Missing recipe version");validateRecipe(r);return r;
}
void writeRecipe(const Recipe& r,const std::filesystem::path& path){validateRecipe(r);std::ofstream f(path);if(!f)throw std::runtime_error("Cannot write recipe");f<<std::setprecision(17)<<"version="<<r.version<<"\nseed="<<r.seed<<"\nstyle="<<r.style<<"\nplateau="<<r.plateau<<"\nopeningHeight="<<r.openingHeight<<"\ncamelbackHeight="<<r.camelbackHeight<<"\nloopHeight="<<r.loopHeight<<"\nimmelmannHeight="<<r.immelmannHeight<<"\ntopSpeedKph="<<r.topSpeedKph<<"\nactiveSeconds="<<r.activeSeconds<<"\nterminalSeconds="<<r.terminalSeconds<<'\n';if(!f)throw std::runtime_error("Recipe write failed");}
Program launch(const State& state,double targetSpeed,double seconds){
    Program p;p.initial=state;constexpr double ramp=.25,ease=.015;
    auto controls=[&](double peak){const double j=peak/(ramp-ease),a=.5*ease*j,b=seconds-ramp;p.controls={{0,1,0,0,0},{ease,1,0,0,a},{ramp-ease,1,0,0,peak-a},{ramp,1,0,0,peak},{b,1,0,0,peak},{b+ease,1,0,0,peak-a},{seconds-ease,1,0,0,a},{seconds,1,0,0,0}};p.controls[1].first[3]=p.controls[2].first[3]=j;p.controls[5].first[3]=p.controls[6].first[3]=-j;};
    double lo=0,hi=60;for(int i=0;i<38;++i){const double peak=(lo+hi)/2;controls(peak);if(shoot(p,.005).end.v<targetSpeed)lo=peak;else hi=peak;}controls((lo+hi)/2);return p;
}
Design generate(const Recipe& recipe,const Cancel& cancel){
    validateRecipe(recipe);Design d;d.recipe=recipe;std::vector<Program> source;
    State state;state.p={0,300,recipe.plateau+4};state.v=0;
    auto append=[&](Program p,const char* id,const char* label,Role role){poll(cancel);p.id=id;p.label=label;p.role=role;state=shoot(p,.01,cancel).end;source.push_back(std::move(p));};
    append(launch(state,50,1.4),"launch-180","180 km/h launch",Role::Launch);
    HillShape opening;opening.height=recipe.openingHeight;opening.bankDegrees=-25;opening.positive=3.6;opening.negative=-1.2;opening.descentNegative=-1.25;
    append(hill(state,opening,cancel),"opening","Rounded hill and twisted descending turn",Role::Opening);
    d.track=compile(std::move(source),.02,cancel);
    d.notes.push_back("Development candidate: full 180-second circuit not yet authored");return d;
}
}
