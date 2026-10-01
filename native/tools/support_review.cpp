#include "support_examples.hpp"
#include "coaster/support_fabrication.hpp"
#include "coaster/track_mesh.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace fs=std::filesystem;
static void exportDesign(Design& d,const fs::path& path,bool regenerate=true,bool jointsOnly=false,bool supportsOnly=false){
    if(regenerate)buildSupportLayout(d);
    const auto report=validateSupportLayout(d.supports,d.request.terrain);
    if(!report.valid())throw std::runtime_error(report.errors.front().code);
    auto sweep=buildClearanceSweep(d.track,d.request.train);
    for(const auto& s:d.supports)if(supportCollision(s,sweep)>=0||supportStationCollision(s,d.station))throw std::runtime_error("Review member clearance failure");
    std::ofstream f(path);if(!f)throw std::runtime_error("Cannot write support review");f<<std::setprecision(17);
    auto vec=[&](Vec3 p){f<<'['<<p.x<<','<<p.y<<','<<p.z<<']';};
    const auto section=trackSection(d.track.profile);
    f<<"{\"format\":2,\"seed\":"<<d.request.seed<<",\"geometryReviewOnly\":true,\"nativeSpineDepth\":"<<section.spineDepth<<",\"nativeSpineRadius\":"<<section.spineRadius<<",\"track\":[";
    Vec3 lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
    auto bounds=[&](Vec3 p){lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};};
    const size_t count=size_t(std::ceil(d.track.length/.35));
    std::vector<double> samples;
    for(size_t i=0;i<=count;++i)samples.push_back(std::min(d.track.length-1e-6,d.track.length*i/count));
    for(const auto& s:d.supports)samples.push_back(s.trackDistance);
    std::sort(samples.begin(),samples.end());samples.erase(std::unique(samples.begin(),samples.end()),samples.end());
    for(size_t i=0;i<samples.size();++i){auto q=d.track.sample(samples[i]);bounds(q.position);if(i)f<<',';f<<'[';vec(q.position);f<<',';vec(q.right);f<<',';vec(q.up);f<<','<<samples[i]<<']';}
    f<<"],\"regions\":[";bool comma=false;
    for(const auto& r:planSupportRegions(d.track,d.request.terrain)){if(comma)f<<',';comma=true;f<<"{\"begin\":"<<r.begin<<",\"end\":"<<r.end<<",\"kind\":"<<int(r.kind)<<'}';}
    f<<"],\"supports\":[";comma=false;size_t footings=0,wallAnchors=0,shared=0,members=0;
    for(const auto& s:d.supports){
        if(comma)f<<',';comma=true;f<<"{\"distance\":"<<s.trackDistance<<",\"attachment\":";vec(s.attachment);
        const auto q=d.track.sample(s.trackDistance);f<<",\"frame\":[";vec(q.position);f<<',';vec(q.right);f<<',';vec(q.up);f<<"],\"members\":[";bool memberComma=false,anchored=false;
        for(const auto& m:s.members){bounds(m.base);bounds(m.top);if(memberComma)f<<',';memberComma=true;
            f<<'[';vec(m.base);f<<',';vec(m.top);f<<','<<m.radiusBase<<','<<m.radiusTop<<','<<int(m.kind)<<','<<(m.spineContact?"true":"false")<<']';
            ++members;if(m.kind==SupportMemberKind::Footing){++footings;anchored=true;}if(m.kind==SupportMemberKind::RockAnchor){++wallAnchors;anchored=true;}
        }f<<"]}";shared+=!anchored;
    }
    f<<"],\"terrain\":{\"step\":8,\"points\":[";comma=false;
    const int xmin=int(std::floor((lo.x-45)/8)),xmax=int(std::ceil((hi.x+45)/8)),ymin=int(std::floor((lo.y-45)/8)),ymax=int(std::ceil((hi.y+45)/8));
    for(int y=ymin;y<=ymax;++y)for(int x=xmin;x<=xmax;++x){if(comma)f<<',';comma=true;vec({x*8.,y*8.,d.request.terrain.height(x*8.,y*8.)});}
    f<<"],\"columns\":"<<xmax-xmin+1<<",\"rows\":"<<ymax-ymin+1<<"},\"bounds\":[";vec(lo);f<<',';vec(hi);
    f<<"],\"counts\":{\"attachments\":"<<d.supports.size()<<",\"sharedAttachments\":"<<shared<<",\"footings\":"<<footings<<",\"wallAnchors\":"<<wallAnchors<<",\"members\":"<<members<<"}}\n";
    if(!f)throw std::runtime_error("Support review write failed");
    std::cout<<path.filename().string()<<": attachments="<<d.supports.size()<<" shared="<<shared<<" footings="<<footings<<" wall anchors="<<wallAnchors<<" members="<<members<<'\n';
    if(d.track.profile==TrackProfile::Exa){
        auto parts=buildSupportFabrication(d.track,d.supports);size_t vertices=0;
        if(!jointsOnly&&!supportsOnly){
        for(double start=0;start<d.track.length;start+=80){const double end=std::min(start+80,d.track.length);
            for(int tube=0;tube<3;++tube){const auto m=trackTubeMesh(d.track,start,end,tube==2?0:(tube?1:-1)*section.gauge*.5,tube==2?-section.spineDepth:0,tube==2?section.spineRadius:section.railRadius);
                FabricationPart p;p.name=tube==2?"Track spine":"Running rail";p.smooth=true;p.vertices=m.positions;
                for(size_t i=0;i<m.indices.size();i+=3)p.faces.push_back({m.indices[i],m.indices[i+1],m.indices[i+2]});parts.push_back(std::move(p));}
        }
        for(double s=0;s<d.track.length;s+=section.tieSpacing)parts.push_back(exaCrosshead(d.track.sample(s)));
        }else if(jointsOnly)std::erase_if(parts,[](const FabricationPart& p){return p.name!="Fitted tube"&&p.name!="Welded node can";});
        std::ofstream mesh(path.parent_path()/(path.stem().string()+"-fabrication.json"));mesh<<std::setprecision(17)<<"{\"profile\":\"exa-1\",\"parts\":[";
        for(size_t i=0;i<parts.size();++i){const auto& p=parts[i];if(i)mesh<<',';
            mesh<<"{\"name\":\""<<p.name<<"\",\"owner\":"<<p.owner<<",\"member\":"<<p.member<<",\"material\":"<<int(p.material)<<",\"smooth\":"<<(p.smooth?"true":"false")<<",\"vertices\":[";
            for(size_t j=0;j<p.vertices.size();++j){const auto v=p.vertices[j];if(j)mesh<<',';mesh<<'['<<v.x<<','<<v.y<<','<<v.z<<']';}
            mesh<<"],\"faces\":[";for(size_t j=0;j<p.faces.size();++j){if(j)mesh<<',';mesh<<'[';for(size_t k=0;k<p.faces[j].size();++k){if(k)mesh<<',';mesh<<p.faces[j][k];}mesh<<']';}mesh<<"]}";
            vertices+=fabricationMesh(p).positions.size();
        }
        mesh<<"]}\n";if(!mesh)throw std::runtime_error("Fabrication export failed");
        std::cout<<"  fabrication parts="<<parts.size()<<" render vertices="<<vertices<<std::endl;
    }
}
int main(int argc,char** argv){try{
    bool exa=false,save=false,current=false,joints=false,tall=false,supportsOnly=false;std::string source,example;
    if(argc<2)throw std::runtime_error("Usage: support_review OUTPUT_DIRECTORY [SAVED_DESIGN] [--save-review] [--exa] [--current-supports] [--joints-only] [--supports-only] [--tall-inversions] [--case=loop-0]");
    for(int i=2;i<argc;++i){const std::string arg=argv[i];if(arg=="--exa")exa=true;else if(arg=="--save-review")save=true;else if(arg=="--current-supports")current=true;else if(arg=="--joints-only")joints=true;else if(arg=="--supports-only")supportsOnly=true;else if(arg=="--tall-inversions")tall=true;else if(arg.starts_with("--case="))example=arg.substr(7);else if(source.empty()&&!arg.starts_with("--"))source=arg;else throw std::runtime_error("Unknown support review argument");}
    if(current&&(exa||save||source.empty()))throw std::runtime_error("Current-supports export requires an existing save and no migration/save request");
    if(save&&source.empty())throw std::runtime_error("Saving a review requires a complete source design");
    if(!source.empty()&&(tall||!example.empty()))throw std::runtime_error("Fixture options cannot modify a saved ride");
    const fs::path output=argv[1];fs::create_directories(output);
    if(!source.empty()){
        Design d;std::string error;if(!loadDesign(source,d,error))throw std::runtime_error(error);
        if(exa){std::cout<<"Migrating separate review to Exa profile"<<std::endl;if(!migrateTrackProfile(d,TrackProfile::Exa,error))throw std::runtime_error(error);}
        else if(save){std::cout<<"Regenerating and validating separate support review"<<std::endl;if(!regenerateSupports(d,error))throw std::runtime_error(error);}
        // A saved review must export the freshly accepted graph. Rebuilding
        // only its members here invalidates the acceptance fingerprint.
        exportDesign(d,output/"saved-layout.json",!exa&&!current&&!save,joints,supportsOnly);
        if(save){
            const auto path=output/"regenerated-supports.vcdesign";
            if(fs::equivalent(fs::absolute(source).parent_path(),fs::absolute(output))&&fs::path(source).filename()==path.filename())throw std::runtime_error("Review output must not replace its source save");
            if(!d.accepted()&&!regenerateSupports(d,error))throw std::runtime_error(error);
            if(!saveDesign(d,path.string(),error))throw std::runtime_error(error);
            Design checked;if(!loadDesign(path.string(),checked,error))throw std::runtime_error(error);
            std::ofstream report(output/"regenerated-supports-report.json");report<<reportJson(checked)<<'\n';
            std::cout<<"Freshly validated review save: "<<path.string()<<'\n';
        }
    }
    else {bool exported=false;for(auto kind:{SupportStructureKind::Camelback,SupportStructureKind::Loop,SupportStructureKind::Immelmann})for(unsigned seed:{0u,1u,3u}){
        auto d=supportExample(kind,seed,tall?2.5:1.);const std::string name=kind==SupportStructureKind::Camelback?"camelback":kind==SupportStructureKind::Loop?"loop":"immelmann";
        if(!example.empty()&&example!=name+"-"+std::to_string(seed))continue;
        if(exa)d.track.profile=d.request.trackProfile=TrackProfile::Exa;
        exportDesign(d,output/(name+"-"+std::to_string(seed)+".json"),true,joints,supportsOnly);
        exported=true;
    }if(!exported)throw std::runtime_error("Unknown fixture case");}
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
