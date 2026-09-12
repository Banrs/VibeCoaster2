#include "../src/terrain_baseline.hpp"
#include <iostream>
using namespace coaster;using namespace coaster::detail;
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
bool rejects(const std::function<void()>& operation){try{operation();return false;}catch(const std::runtime_error&){return true;}}
std::vector<double> stations(int count){std::vector<double> result;for(int i=0;i<=count;++i)result.push_back(double(i));return result;}
void joins(const TerrainBaseline& result){bool valid=true;for(const auto& gap:result.gaps){const auto& profile=gap.profile;
    for(int order=1;order<4;++order)valid&=profile.nodes.front()[order]==0&&profile.nodes.back()[order]==0;
    for(size_t i=0;i+1<profile.nodes.size();++i)for(int order=0;order<4;++order){const auto polynomial=baselineCell(profile.nodes[i],profile.nodes[i+1]);valid&=std::abs(baselineDerivative(polynomial,0,order)-profile.nodes[i][order])<1e-7&&std::abs(baselineDerivative(polynomial,1,order)-profile.nodes[i+1][order])<1e-7;}}
    check(valid,"Every ordinary cell and source port retains its declared C3 jets");}
void ordinaryFloor(const TerrainBaseline& result,const std::vector<double>& distance,const std::vector<double>& floor){double deficit=-INFINITY;
    for(const auto& gap:result.gaps)for(size_t i=0;i+1<distance.size();++i)if(distance[i]>=gap.begin&&distance[i+1]<=gap.end)for(int j=0;j<=20;++j){const double u=j/20.;deficit=std::max(deficit,floor[i]+u*(floor[i+1]-floor[i])-gap.profile.height(distance[i]+u*(distance[i+1]-distance[i])-gap.begin));}
    check(deficit<1e-8,"Continuous ordinary-gap floor clears dense independent points");}
int main(){try{
    {
        // Minimize 4*x^2+9*y^2 subject to x+y>=1. The independent
        // stationary equations give x=9/13, y=4/13.
        const auto transform=[](std::vector<double> value){value[0]/=2;value[1]/=3;return value;};
        const auto physical=transform(projectBaselineConstraints({{{1,1},1,1},{{1,0},0,1},{{0,1},0,1}},2,{},transform,transform));
        check(std::abs(physical[0]-9./13)<1e-12&&std::abs(physical[1]-4./13)<1e-12,
            "Sparse physical constraints retain the independent nonidentity objective optimum");
    }
    {
        constexpr int dimensions=1050;std::vector<BaselineConstraint> constraints;
        for(int i=0;i<dimensions;++i){std::vector<double> row(dimensions);row[i]=1;constraints.push_back({std::move(row),1,1});}
        const auto projected=projectBaselineConstraints(constraints,dimensions);
        check(std::all_of(projected.begin(),projected.end(),[](double x){return x==1;}),
            "Independent orthogonal halfspaces reach their analytic optimum beyond the former arbitrary iteration cap");
    }
    for(double epsilon:{1e-5,1e-9}){
        const double length=std::hypot(1.,epsilon);const std::vector<double> normal{1/length,epsilon/length};
        const auto projected=projectBaselineConstraints({{{1,0},1,100},{normal,2,1}},2);
        check(std::abs(projected[0]-2*normal[0])<1e-10&&std::abs(projected[1]-2*normal[1])<1e-10,
            "Near-parallel stricter plane removes the initially active plane and retains the independent analytic optimum");
    }
    {
        constexpr double delta=.001,epsilon=.00001;const double a=std::hypot(1.,delta),b=std::hypot(1.,epsilon);
        // Scale selects y, then x, then the nearly opposing y/z plane. The
        // last plane removes interior x while retaining both near-parallel
        // y/z columns. The optimum separates into two independent 2D problems.
        const auto projected=projectBaselineConstraints({{{0,1,0,0},1,1000},{{1,0,0,0},1,100},{{0,-1/a,delta/a,0},0,10},{{1/b,0,0,epsilon/b},2,1}},4);
        check(std::abs(projected[0]-2/b)<1e-8&&std::abs(projected[1]-1)<1e-8&&std::abs(projected[2]-1/delta)<1e-7&&std::abs(projected[3]-2*epsilon/b)<1e-8,
            "Interior active deletion preserves retained near-dependent columns and the independent separable optimum");
    }
    for(int cars:{6,12}){
        const auto s=stations(200);const std::vector<double> low(201,-100),zero(201);
        TrainConfig train;train.cars=cars;train.dragCdA=0;train.rollingResistance=0;
        const auto transport=solveTerrainBaseline(s,low,zero,zero,{{0,40,20,true},{160,200,-100,false}},{},{},{},{},{{20,180,50,40,40}},train);
        check(std::abs(transport.height(180)-(20+(50*50-40*40)/(2*gravity)))<1e-7,
            "Lossless finite-train transport agrees with the independent mean-potential energy balance");
    }
    {
        const auto s=stations(400);const std::vector<double> low(401,-100);std::vector<double> hill(401);
        for(int i=0;i<=400;++i)hill[i]=40*std::pow(std::sin(pi*i/400),2);
        TrainConfig train;train.cars=6;train.dragCdA=0;train.rollingResistance=0;
        check(rejects([&]{solveTerrainBaseline(s,low,hill,hill,{{0,400,0,true}},{},{},{},{},{{0,400,20,0,INFINITY}},train);}),
            "An interior crest rejects passive transport even when its final potential returns to the inlet height");
        train.dragCdA=24;
        // Independent fine Euler integration of dw/ds=-2*b*w-2*g*dh/ds.
        const auto mean=[&](double at){double z=0;for(int car=0;car<train.cars;++car){const double x=std::clamp(at+(2.5-car)*train.spacing,0.,400.);const int left=std::min(399,int(x));z+=hill[left]+(hill[left+1]-hill[left])*(x-left);}return z/train.cars;};
        double w=80*80,previous=mean(0);const double b=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
        for(int i=1;i<=40000;++i){const double current=mean(i*.01);w-=2*b*w*.01+2*gravity*(current-previous);previous=current;}
        const double exit=std::sqrt(w);
        const auto transported=solveTerrainBaseline(s,low,hill,hill,{{0,400,0,true}},{},{},{},{},{{0,400,80,exit-.01,exit+.01}},train);
        check(transported.height(200)==0,"Drag-weighted intermediate potential agrees with independent fine integration");
        check(rejects([&]{solveTerrainBaseline(s,low,hill,hill,{{0,400,0,true}},{},{},{},{},{{0,400,80,0,80*std::exp(-b*400)+.01}},train);}),
            "Equal endpoint heights cannot replace drag-weighted energy through an interior hill");
    }
    auto distance=stations(400);std::vector<double> floor(401,-1),target(401),authored(401);
    for(size_t i=0;i<floor.size();++i)floor[i]=2.5*std::exp(-std::pow((distance[i]-140)/12,2))-.4;
    std::vector<BaselineAnchor> anchors{{0,0,0,true},{150,250,0,false},{400,400,0,true}};
    auto joint=solveTerrainBaseline(distance,floor,target,authored,anchors);joins(joint);ordinaryFloor(joint,distance,floor);
    check(joint.sources[1].height>.1,"Approach clearance can move the rigid source datum in the same solve");
    check(joint.height(0)==0&&joint.height(400)==0,"Fixed station endpoints stay exact");
    bool rigid=true;for(int i=150;i<=250;++i)rigid&=joint.height(i)==joint.sources[1].height;check(rigid,"The complete source receives one rigid translation");
    std::fill(floor.begin(),floor.end(),-1);std::fill(target.begin(),target.end(),0);
    for(size_t i=0;i<floor.size();++i)floor[i]=2.5*std::exp(-std::pow((distance[i]-200)/70,2))-.5;
    const BaselineMotionBounds limit{0,400,.2,-.001,.001,.00005};
    auto bounded=solveTerrainBaseline(distance,floor,target,authored,{{0,0,0,true},{400,400,0,true}},{limit});joins(bounded);ordinaryFloor(bounded,distance,floor);
    bool derivatives=true;for(const auto& gap:bounded.gaps)for(size_t i=0;i+1<gap.profile.nodes.size();++i){const auto polynomial=baselineCell(gap.profile.nodes[i],gap.profile.nodes[i+1]);
        for(int sample=0;sample<=100;++sample){const double u=sample/100.,spacing=gap.profile.spacing;derivatives&=std::abs(baselineDerivative(polynomial,u,1)/spacing)<=limit.maximumGrade+1e-9;const double second=baselineDerivative(polynomial,u,2)/(spacing*spacing);derivatives&=second>=limit.minimumSecond-1e-9&&second<=limit.maximumSecond+1e-9;derivatives&=std::abs(baselineDerivative(polynomial,u,3)/std::pow(spacing,3))<=limit.maximumThird+1e-9;}}
    check(derivatives,"Independent derivative samples honor the entire constrained motion domain");
    std::vector<double> offsetHeight(distance.size(),100),offsetTarget(distance.size(),105),offsetFloor(distance.size(),-100);
    const auto offset=solveTerrainBaseline(distance,offsetFloor,offsetTarget,offsetHeight,{{0,400,-100,false}});
    check(std::abs(offset.height(200)-5)<1e-9,"Absolute elevation target adds only its difference from an already elevated rigid source");
    std::vector<double> liftHeight(distance.size());std::vector<BaselineJet> liftJets(distance.size());const TerrainTransfer lift{0,10,400};
    const BaselinePolynomial liftPolynomial{0,0,0,0,350,-840,700,-200};
    for(size_t i=0;i<distance.size();++i){for(int order=0;order<4;++order)liftJets[i][order]=baselineDerivative(liftPolynomial,distance[i]/400,order)/std::pow(400.,order);liftHeight[i]=liftJets[i][0];}
    check(rejects([&]{solveTerrainBaseline(distance,offsetFloor,liftHeight,liftHeight,{{0,0,0,true},{400,400,0,true}},{{0,400,.02,-1,1,1}},{},{},liftJets);}),
        "Whole authored rise cannot satisfy a grade bound below its independent mean grade");
    const auto whole=solveTerrainBaseline(distance,offsetFloor,liftHeight,liftHeight,{{0,0,0,true},{400,400,0,true}},{{0,400,.04,-.001,.001,.00005}},{},{},liftJets);
    bool completeGrade=true;for(int i=0;i<=4000;++i){const double at=i*.1;completeGrade&=std::abs(whole.gaps.front().profile.jet(at)[1]+terrainVerticalJet(lift,at)[1])<=.04+1e-8;}
    check(completeGrade,"Independent dense samples bound the complete authored plus solved vertical derivative");
    std::fill(floor.begin(),floor.end(),-1);for(int i=370;i<=400;++i)floor[i]=-.3+.069*(400-i);
    const BaselineMotionBounds impossible{0,400,2,-.02,.02,.003};
    check(6*(floor[393]-0)/std::pow(7.,3)>.003,"Independent Taylor bound witnesses the impossible fixed-port rate");
    bool siteInfeasible=false;
    try{solveTerrainBaseline(distance,floor,target,authored,{{0,0,0,true},{400,400,0,true}},{impossible});}
    catch(const TerrainTransferInfeasible&){siteInfeasible=true;}
    check(siteInfeasible,"A floor/endpoint/rate contradiction rejects this ranked site without becoming a programming error");
    const std::vector<BaselineAnchor> stationBudget{{0,0,0,false,.6,true},{400,400,0,false,.6,true}};
    const auto station=solveTerrainBaseline(distance,floor,target,authored,stationBudget,{impossible});joins(station);ordinaryFloor(station,distance,floor);
    check(station.sources.front().height==station.sources.back().height,"Station departure and arrival share exactly one solved datum");
    check(station.sources.front().height>.01&&station.sources.front().height<=.6+1e-8,"Real station height budget resolves the rate/floor constraint in the same solve");
    check(rejects([&]{solveTerrainBaseline(distance,floor,target,authored,{{0,0,0,false,.001,true},{400,400,0,false,.001,true}},{impossible});}),"An insufficient station budget stays infeasible");
    for(bool fixed:{false,true}){
        bool floorBudgetConflict=false;
        try{solveTerrainBaseline(distance,floor,target,authored,{{0,0,2,fixed,1,true},{400,400,0,false,1,true}});}
        catch(const TerrainTransferInfeasible&){floorBudgetConflict=true;}
        check(floorBudgetConflict,"A finite source floor above its ceiling remains physically infeasible for both fixed and solved station heights");
    }
    std::fill(floor.begin(),floor.end(),.4);std::fill(target.begin(),target.end(),.7);
    auto tied=solveTerrainBaseline(distance,floor,target,authored,{{0,0,0,false,1,true},{400,400,0,false,1,true}});
    check(std::abs(tied.height(0)-.7)<1e-10&&tied.height(0)==tied.height(400),"One shared station variable and its C3 gap attain the independent constant-height optimum");
    // Actual powered geometry is one rigid source. A terrain ridge moves its
    // datum, never its certified rise or the start/length of its active pulse.
    const TerrainTransfer profile{10,30,200,25,150};
    for(size_t i=0;i<authored.size();++i){authored[i]=profile.height(double(i)-100);target[i]=authored[i];floor[i]=-1;}
    floor[200]=8;
    const auto powered=solveTerrainBaseline(distance,floor,target,authored,{{0,20,0,true},{100,300,0,false},{380,400,0,true}});
    joins(powered);ordinaryFloor(powered,distance,floor);
    const double datum=powered.height(100);bool unchanged=true;
    for(int i=100;i<=300;++i)unchanged&=powered.height(i)==datum;
    check(unchanged&&datum>=8-1e-8,"Continuous source clearance raises the complete rigid powered source together");
    check(authored[125]+powered.height(125)==profile.start+datum&&authored[275]+powered.height(275)==profile.finish+datum,
        "Rigid source placement preserves the exact active-window endpoints and certified rise");
    distance=stations(500);floor.assign(501,-10);target.assign(501,0);authored.assign(501,0);
    const TerrainTransfer supplySource{0,50,250};
    for(size_t i=200;i<authored.size();++i)authored[i]=supplySource.height(double(i)-200);
    for(int cars:{6,12}){
        TrainConfig train;train.cars=cars;
        const auto supplied=solveTerrainBaseline(distance,floor,target,authored,{{0,50,30,true},{200,500,0,false}},{},{},{},{},{{50,200,65,0,60}},train);
        std::vector<AuthoredPoint> points;for(int i=0;i<=500;++i){
            const double height=authored[i]+supplied.height(i);
            points.push_back({{double(i),0,height},0,Element::Return,{0,0,1}});
        }
        const auto track=compile(points,false);const double inlet=track.spans[200].start;
        const auto actual=estimatePassiveTransfer(track,train,track.spans[50].start,inlet,65);
        check(actual.reached&&actual.speed<=60.01,"Variable motor inlet clears independent canonical finite-train transport for both train lengths");
    }
    floor.assign(501,-1);target.assign(501,0);authored.assign(501,0);
    const std::vector<BaselineAnchor> crossingAnchors{{0,20,2,true},{100,120,0,false},{260,280,0,false},{480,500,2,true}};
    const std::vector<BaselineCrossing> crossings{{100,260,.5,.25,9},{60,210,.7,.2,5}};
    auto crossed=solveTerrainBaseline(distance,floor,target,authored,crossingAnchors,{},crossings);joins(crossed);ordinaryFloor(crossed,distance,floor);
    for(const auto& crossing:crossings){const double under=crossed.height(double(crossing.under))*(1-crossing.underFraction)+crossed.height(double(crossing.under+1))*crossing.underFraction,over=crossed.height(double(crossing.over))*(1-crossing.overFraction)+crossed.height(double(crossing.over+1))*crossing.overFraction;check(over-under>=crossing.minimumSeparation-1e-8,"A planned source/gap crossing is separated by the joint solve");}
    check(rejects([&]{solveTerrainBaseline(distance,floor,target,authored,crossingAnchors,{},{{10,490,.5,.5,9}});}),"Contradictory fixed crossing heights fail without translating the tail afterward");
    const std::vector<BaselineAnchor> reversedPorts{{0,100,15,true},{400,500,2,true}};
    const auto automatic=solveTerrainBaseline(distance,floor,target,authored,reversedPorts,{},{{50,450,.5,.5,9,true}});
    check(automatic.height(50)==15&&automatic.height(450)==2,"A free crossing preserves physical port heights and chooses their feasible over/under order");
    check(rejects([&]{solveTerrainBaseline(distance,floor,target,authored,reversedPorts,{},{{50,450,.5,.5,9}});}),"An explicitly directed crossing still rejects the opposite fixed-port order");
    floor.assign(501,-20);
    const auto constrainedOrder=solveTerrainBaseline(distance,floor,target,authored,{{0,100,-20,false,4},{400,500,-1,true}},{},{{50,450,.5,.5,9,true}});
    check(std::abs(constrainedOrder.height(50)+10)<1e-7&&constrainedOrder.height(450)==-1,
        "A free crossing selects the feasible order when the unconstrained sign cannot meet separation within a source ceiling");
    // Preserve the original almost-parallel port regression directly at the
    // projector boundary, without retaining an unused source representation.
    const double progress=35*std::pow(.01,4)-84*std::pow(.01,5)+70*std::pow(.01,6)-20*std::pow(.01,7);
    const auto shortPort=projectBaselineConstraints({{{-1,0},0,1},{{1-progress,progress},1e-5,1}},2);
    check(std::abs(shortPort[0])<1e-8&&std::abs(shortPort[1]-1e-5/progress)<1e-4,"A feasible almost-parallel port constraint retains its independent analytic height");
    // This actual double rounds length/spacing just above seven. Domain
    // ownership must terminate exactly at the source, including derivative rows.
    const double awkwardLength=300.02702702702703;std::vector<double> awkwardDistance;
    for(int i=0;i<=101;++i)awkwardDistance.push_back(awkwardLength*i/101);
    std::vector<double> awkwardFloor(102,-1),awkwardTarget(102),awkwardAuthored(102);
    const auto awkward=solveTerrainBaseline(awkwardDistance,awkwardFloor,awkwardTarget,awkwardAuthored,{{0,0,0,true},{101,101,0,true}},{{0,101,1,-1,1,1}});
    joins(awkward);ordinaryFloor(awkward,awkwardDistance,awkwardFloor);
    check(awkward.height(awkwardLength)==0,"Nonrepresentable control spacing ends exactly at its fixed source port");
    int cancellationPolls=0;bool interrupted=false;
    try{solveTerrainBaseline(distance,floor,target,authored,crossingAnchors,{},{},[&]{return ++cancellationPolls==3;});}
    catch(const std::exception& error){interrupted=std::string(error.what())=="CANCELLED";}
    check(interrupted,"Cancellation is observed during the source/gap solve, not only before it");
    bool cancelled=false;try{solveTerrainBaseline(distance,floor,target,authored,crossingAnchors,{},{},[]{return true;});}catch(const std::exception& error){cancelled=std::string(error.what())=="CANCELLED";}check(cancelled,"Cancellation is observed before the joint solve");
    // Differentiate independently sampled positions on a circle plus a cubic
    // height profile, rather than reconstructing the composition formula.
    const Knot circle{{0,0,0},{1,0,0},{0,.001,0},{0,0,1},.2,Element::Turn};
    const auto composed=addBaselineJet(circle,{2,.1,-.002,.0001});
    const auto position=[](double s){return Vec3{1000*std::sin(s/1000),2000*std::pow(std::sin(s/2000),2),2+.1*s-.001*s*s+.0001*s*s*s/6};};
    constexpr double h=.01;
    const Vec3 before=position(-h),middle=position(0),after=position(h),first=(after-before)/(2*h),second=(after-middle*2+before)/(h*h);
    const Vec3 tangent=coaster::unit(first),curvature=(second-tangent*dot(tangent,second))/dot(first,first);
    check(norm(composed.position-middle)<1e-12&&norm(composed.tangent-tangent)<1e-8&&norm(composed.curvature-curvature)<1e-8,
        "Composed baseline jets agree with independent position differences on curved track");
    check(std::abs(dot(composed.up,composed.tangent))<1e-12&&composed.bank==circle.bank&&composed.element==circle.element,
        "Baseline composition preserves the bank and element while keeping an orthogonal reference frame");
    std::cout<<checks<<" independent checks passed; actual finite-train replay remains required.\n";return 0;
}catch(const std::exception& error){std::cout<<"FAILED after "<<checks<<" checks: "<<error.what()<<'\n';return 1;}}
