#include "../src/terrain_transfer.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);}
std::pair<double,double> measured(const detail::TerrainTransfer& profile,double begin,double length){
    double grade=0,curvature=0;
    for(int i=1;i<4000;++i){
        const double at=begin+length*i/4000,step=length/100000;
        const double before=profile.height(at-step),middle=profile.height(at),after=profile.height(at+step);
        const double slope=(after-before)/(2*step),second=(after-2*middle+before)/(step*step);
        grade=std::max(grade,std::abs(slope));curvature=std::max(curvature,std::abs(second)/std::pow(1+slope*slope,1.5));
    }
    return {grade,curvature};
}
}
int main(){try{
    for(double rise:{1.,20.,80.,207.0828558894449,250.})for(double grade:{.25,1.,2.,10.})for(double curvature:{.0001,.001,.0056486304,.02}){
        const double length=detail::minimumTerrainWindowLength(rise,grade,curvature);
        const auto [actualGrade,actualCurvature]=measured({rise,0,length},0,length);
        check(actualGrade<=grade*1.00001&&actualCurvature<=curvature*1.00002,"Shared minimum span respects independently measured grade and curvature");
        check(std::max(actualGrade/grade,actualCurvature/curvature)>.9999,"At least one physical span constraint is active");
        const auto shortened=measured({rise,0,length*.999},0,length*.999);
        check(shortened.first>grade||shortened.second>curvature,"Shortening a minimum-span transition violates an independently measured physical requirement");
    }
    check(detail::minimumTerrainWindowLength(0,2,.005)==0,"Zero rise has no invented minimum terrain length");
    // Active windows are source geometry, with level extensions outside the
    // certified pulse. Adding return rail cannot rescale or move that pulse.
    const double length=detail::minimumTerrainWindowLength(210,1,.003);
    const detail::TerrainTransfer descent{220,10,600+length+500,600,length};
    auto extended=descent;extended.length+=1000;
    const double end=descent.activeStart+descent.activeLength;
    check(descent.height(600)==220&&descent.height(end)==10,"Active window ports retain exact heights");
    check(descent.height(599)==220&&descent.height(end+1)==10,"Outside the active window the level extensions are exact");
    bool monotone=true,unmoved=true;double previous=220;
    const detail::TerrainTransfer ascent{10,220,extended.length,extended.length-end,length};
    for(int i=0;i<=4000;++i){const double at=extended.length*i/4000,z=descent.height(at);
        monotone&=z<=previous&&z>=10;previous=z;unmoved&=z==extended.height(at)&&std::abs(ascent.height(extended.length-at)-z)<1e-8;
    }
    check(monotone,"The certified descent is monotone within its fixed port heights");
    check(unmoved,"Longer level extensions preserve the same pulse and its reflected climb");
    const double h=length*.001,entryOne=220-descent.height(600+h),entryTwo=220-descent.height(600+2*h);
    const double exitOne=descent.height(end-h)-10,exitTwo=descent.height(end-2*h)-10;
    check(entryTwo/entryOne>15.8&&entryTwo/entryOne<16.1&&exitTwo/exitOne>15.8&&exitTwo/exitOne<16.1,"Both window joins have fourth-order displacement and C3 level ports");
    const auto [grade,curvature]=measured(descent,600,length);
    check(grade<=1.000001&&curvature<=.00300001&&curvature>.00299,"An explicit active interval retains its independently measured geometric budget");
    for(int invalid=0;invalid<4;++invalid){bool rejected=false;
        try{detail::minimumTerrainWindowLength(invalid==0?-1:invalid==1?NAN:210,invalid==2?0:1,invalid==3?0:.003);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"Malformed source-span requirements remain programming errors");
    }
    std::cout<<"PASS "<<checks<<" terrain transfer checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
