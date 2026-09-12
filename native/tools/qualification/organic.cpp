#define main unused_generation_suite_main
#include ORGANIC_TEST_SOURCE
#undef main
#include <fstream>
int main(int argc,char** argv){try{
    if(argc<3||argc>4)throw std::runtime_error("Expected accepted save, matching generation plan and optional crossing requirement");
    Design design;std::string error;if(!loadDesign(argv[1],design,error))throw std::runtime_error(error);
    std::ifstream input(argv[2]);if(!input)throw std::runtime_error("Missing matching generation plan");
    design.planningDiagnostics=std::string(std::istreambuf_iterator<char>(input),{});
    classifyGeometry(design);checkFoldedGeometry(design,argc==4);checkOperationIntent(design);checkTurnSpeed(design);checkSupportSpacing(design);
    std::cout<<"PASS "<<checks<<" independent saved-ride organic checks\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL after "<<checks<<": "<<error.what()<<'\n';return 1;}}
