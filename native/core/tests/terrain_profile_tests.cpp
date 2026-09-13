#include "coaster/coaster.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks=0;
static void check(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
int main(){try{
    Terrain flat;check(flat.valid()&&flat.name()=="flat"&&flat.slopeBound()==0,"Flat ground is supported");
    for(double x:{-100000.,0.,100000.})for(double y:{-100000.,0.,100000.})
        check(flat.height(x,y)==0&&flat.localSlopeBound(x,y,100)==0,"Flat ground remains level across the supported footprint");
    for(int kind:{-1,1,2,3,99}){
        GenerationRequest request;request.terrain.kind=TerrainKind(kind);request.targets.requireIntensity=false;
        check(!request.terrain.valid(),"Removed or invalid terrain kind is refused");
        const auto report=validateRequest(request);
        check(!report.valid()&&report.errors.front().code=="TERRAIN_PROFILE","Unsupported surface fails at the request boundary");
        int callbacks=0;const auto rejected=generate(request,{},[&](int,const std::string&){++callbacks;});
        check(!rejected.accepted()&&rejected.track.knots.empty()&&callbacks==0,"Removed terrain cannot construct or simulate a ride");
    }
    for(double value:{NAN,INFINITY,-INFINITY})
        check(!std::isfinite(flat.localSlopeBound(value,0,1))&&!std::isfinite(flat.localSlopeBound(0,value,1))&&!std::isfinite(flat.localSlopeBound(0,0,value)),"Nonfinite footprint queries fail closed");
    check(!std::isfinite(flat.localSlopeBound(0,0,-1)),"Negative footprint radius is refused");
    std::cout<<"PASS "<<checks<<" flat-only surface and removed-terrain rejection checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
