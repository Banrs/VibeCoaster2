#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace coaster {
inline bool validIdentifier(std::string_view id,size_t maximum=64){
    if(id.empty()||id.size()>maximum)return false;
    for(unsigned char c:id)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'))return false;
    return true;
}
enum class RideRole {
    Unspecified, Station, Departure, Opening, CliffApproach, CliffLip, CliffDrop,
    DownhillLaunch, Camelback, Wave, Loop, Immelmann, Signature, Return, Brakes
};
enum class TerrainAnchor { Station, Approach, Plateau, CliffFoot, WaveBench, LoopBasin, ImmelmannShoulder, Ravine };

// Recipe angles use degrees and distances use metres. Canonical authoring and
// replay still use SI/radians. A zero operation speed/acceleration is resolved
// from the requested ride target or the corresponding rated drive model.
struct OperationParameters {
    double lengthMeters{180},targetSpeedKmh{},gradeDegrees{},accelerationMps2{};
    bool operator==(const OperationParameters&) const=default;
};
struct HillParameters {
    double riseMeters{110},negativeG{-.65},twistDegrees{50},pulloutG{3.8};
    double releaseSeconds{1.2},recoverySeconds{1.5},profileScale{1};
    bool operator==(const HillParameters&) const=default;
};
struct CliffParameters {
    double summitHeightMeters{285},lipSpeedKmh{32},dropDegrees{88},outwardBankDegrees{30},approachLengthMeters{480};
    bool operator==(const CliffParameters&) const=default;
};
struct TurnParameters {
    double headingDegrees{180},riseMeters{65},bankDegrees{70},exitNormalG{3.8},lengthMeters{500};
    bool operator==(const TurnParameters&) const=default;
};
struct InversionParameters {
    // Reference height at 65m/s for a loop or 53m/s for an Immelmann. Actual
    // scale follows entry energy; the native solve verifies the resulting port.
    double referenceRiseMeters{100},yawDegrees{15},crestG{1},entryPitchDegrees{55},exitPitchDegrees{-15};
    bool operator==(const InversionParameters&) const=default;
};
struct SweepParameters {
    double lengthMeters{250},riseMeters{},headingDegrees{35},exitPitchDegrees{},negativeG{-.8},rollDegrees{};
    bool operator==(const SweepParameters&) const=default;
};
// Profile v1 preserves the approved planar asymmetric body. Only its uniform
// scale and localized low recovery are editable through this protected family.
struct CamelbackParameters {
    double profileScale{1},tailCutSeconds{.75},releaseSeconds{.30},exitNormalG{1.9},minimumExitPitchDegrees{1.1459155902616465};
    bool operator==(const CamelbackParameters&) const=default;
};
using ElementParameters=std::variant<OperationParameters,HillParameters,CliffParameters,TurnParameters,InversionParameters,SweepParameters,CamelbackParameters>;
struct RecipeElement {
    std::string id;
    RideRole role{RideRole::Return};
    TerrainAnchor anchor{TerrainAnchor::Ravine};
    ElementParameters parameters{SweepParameters{}};
    bool operator==(const RecipeElement&) const=default;
};
struct RideRecipe {
    unsigned version{1};
    std::string name{"Escarpment Rift"};
    std::vector<RecipeElement> elements;
    bool operator==(const RideRecipe&) const=default;
};
const char* roleName(RideRole);
const char* anchorName(TerrainAnchor);
uint64_t elementSeed(uint64_t rideSeed,const std::string& elementId);
RideRecipe defaultRideRecipe();
bool validateRecipe(const RideRecipe&,std::string& error);
std::string recipePayload(const RideRecipe&);
bool parseRecipe(const std::string&,RideRecipe&,std::string& error);
bool loadRecipe(const std::string& path,RideRecipe&,std::string& error);
bool saveRecipe(const RideRecipe&,const std::string& path,std::string& error);
}
