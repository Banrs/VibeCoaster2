#include "coaster/recipe.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <fcntl.h>
#include <io.h>
#include <share.h>
#include <sys/stat.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace coaster {
namespace {

constexpr std::size_t maxRecipeElements = 64;
constexpr std::size_t maxRecipeBytes = 1024 * 1024;

struct TemporaryRecipeFile {
    std::filesystem::path path;
    std::FILE* stream{};
    bool armed{};
    ~TemporaryRecipeFile() {
        if (stream) std::fclose(stream);
        if (armed) { std::error_code ignored; std::filesystem::remove(path, ignored); }
    }
};

std::filesystem::path utf8path(const std::string& value) {
    return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(value.data()), value.size()));
}

const char* variantName(const ElementParameters& parameters) {
    return std::visit([](const auto& value) -> const char* {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, OperationParameters>) return "operation";
        else if constexpr (std::is_same_v<T, HillParameters>) return "hill";
        else if constexpr (std::is_same_v<T, CliffParameters>) return "cliff";
        else if constexpr (std::is_same_v<T, TurnParameters>) return "turn";
        else if constexpr (std::is_same_v<T, InversionParameters>) return "inversion";
        else if constexpr (std::is_same_v<T, SweepParameters>) return "sweep";
        else { static_assert(std::is_same_v<T, CamelbackParameters>); return "approved-camelback-v1"; }
    }, parameters);
}

bool parseRole(std::string_view text, RideRole& result) {
    for (int value = int(RideRole::Unspecified); value <= int(RideRole::Brakes); ++value) {
        const auto role = RideRole(value);
        if (text == roleName(role)) { result = role; return true; }
    }
    return false;
}

bool parseAnchor(std::string_view text, TerrainAnchor& result) {
    for (int value = int(TerrainAnchor::Station); value <= int(TerrainAnchor::Ravine); ++value) {
        const auto anchor = TerrainAnchor(value);
        if (text == anchorName(anchor)) { result = anchor; return true; }
    }
    return false;
}

bool parseVariant(std::string_view text, int& result) {
    static constexpr std::string_view names[] = {"operation", "hill", "cliff", "turn", "inversion", "sweep", "approved-camelback-v1"};
    for (int i = 0; i < int(std::size(names)); ++i) if (text == names[i]) { result = i; return true; }
    return false;
}

bool finiteNumber(double value) { return std::isfinite(value); }

bool parseNumber(const std::string& text, double& result) {
    if (text.empty()) return false;
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    input >> result;
    if (!input || !finiteNumber(result)) return false;
    input >> std::ws;
    return input.eof();
}

bool parseQuotedLine(const std::string& line, std::string& result) {
    std::istringstream input(line);
    input.imbue(std::locale::classic());
    input >> std::quoted(result);
    if (!input) return false;
    input >> std::ws;
    return input.eof();
}

bool fieldToken(const std::string& token, std::string& key, std::string& value) {
    const auto equals = token.find('=');
    if (equals == std::string::npos || equals == 0 || equals + 1 >= token.size() || token.find('=', equals + 1) != std::string::npos)
        return false;
    key = token.substr(0, equals); value = token.substr(equals + 1); return true;
}

struct FieldDescriptor {
    const char* name;
    double (*get)(const ElementParameters&);
    void (*set)(ElementParameters&, double);
    double low;
    double high;
};

struct FieldSpan { const FieldDescriptor* data{}; std::size_t size{}; };

template<class T, double T::*Member>
double getField(const ElementParameters& parameters) { return std::get<T>(parameters).*Member; }

template<class T, double T::*Member>
void setField(ElementParameters& parameters, double value) { std::get<T>(parameters).*Member = value; }

template<class T, double T::*Member>
constexpr FieldDescriptor field(const char* name, double low, double high) {
    return {name, &getField<T, Member>, &setField<T, Member>, low, high};
}

FieldSpan fieldsForVariant(int variant) {
    static const std::array operation{
        field<OperationParameters, &OperationParameters::lengthMeters>("lengthMeters", 0, 20000),
        field<OperationParameters, &OperationParameters::targetSpeedKmh>("targetSpeedKmh", 0, 500),
        field<OperationParameters, &OperationParameters::gradeDegrees>("gradeDegrees", -89.9, 89.9),
        field<OperationParameters, &OperationParameters::accelerationMps2>("accelerationMps2", -100, 100)};
    static const std::array hill{
        field<HillParameters, &HillParameters::riseMeters>("riseMeters", -1000, 1000),
        field<HillParameters, &HillParameters::negativeG>("negativeG", -10, 10),
        field<HillParameters, &HillParameters::twistDegrees>("twistDegrees", -720, 720),
        field<HillParameters, &HillParameters::pulloutG>("pulloutG", -10, 10),
        field<HillParameters, &HillParameters::releaseSeconds>("releaseSeconds", 0, 60),
        field<HillParameters, &HillParameters::recoverySeconds>("recoverySeconds", 0, 60),
        field<HillParameters, &HillParameters::profileScale>("profileScale", .0001, 100)};
    static const std::array cliff{
        field<CliffParameters, &CliffParameters::summitHeightMeters>("summitHeightMeters", 0, 1000),
        field<CliffParameters, &CliffParameters::lipSpeedKmh>("lipSpeedKmh", 0, 500),
        field<CliffParameters, &CliffParameters::dropDegrees>("dropDegrees", 0, 180),
        field<CliffParameters, &CliffParameters::outwardBankDegrees>("outwardBankDegrees", -720, 720),
        field<CliffParameters, &CliffParameters::approachLengthMeters>("approachLengthMeters", 0, 20000),
        field<CliffParameters, &CliffParameters::approachHeadingDegrees>("approachHeadingDegrees", -200, 200)};
    static const std::array turn{
        field<TurnParameters, &TurnParameters::headingDegrees>("headingDegrees", -3600, 3600),
        field<TurnParameters, &TurnParameters::riseMeters>("riseMeters", -1000, 1000),
        field<TurnParameters, &TurnParameters::bankDegrees>("bankDegrees", -720, 720),
        field<TurnParameters, &TurnParameters::exitNormalG>("exitNormalG", -10, 10),
        field<TurnParameters, &TurnParameters::lengthMeters>("lengthMeters", .001, 20000)};
    static const std::array inversion{
        field<InversionParameters, &InversionParameters::referenceRiseMeters>("referenceRiseMeters", 20, 250),
        field<InversionParameters, &InversionParameters::yawDegrees>("yawDegrees", -3600, 3600),
        field<InversionParameters, &InversionParameters::crestG>("crestG", -10, 10),
        field<InversionParameters, &InversionParameters::entryPitchDegrees>("entryPitchDegrees", -89.9, 89.9),
        field<InversionParameters, &InversionParameters::exitPitchDegrees>("exitPitchDegrees", -89.9, 89.9)};
    static const std::array sweep{
        field<SweepParameters, &SweepParameters::lengthMeters>("lengthMeters", .001, 20000),
        field<SweepParameters, &SweepParameters::riseMeters>("riseMeters", -1000, 1000),
        field<SweepParameters, &SweepParameters::headingDegrees>("headingDegrees", -3600, 3600),
        field<SweepParameters, &SweepParameters::exitPitchDegrees>("exitPitchDegrees", -89.9, 89.9),
        field<SweepParameters, &SweepParameters::negativeG>("negativeG", -10, 10),
        field<SweepParameters, &SweepParameters::rollDegrees>("rollDegrees", -720, 720)};
    static const std::array camelback{
        field<CamelbackParameters, &CamelbackParameters::profileScale>("profileScale", .8, 1.2),
        field<CamelbackParameters, &CamelbackParameters::tailCutSeconds>("tailCutSeconds", 0, 1),
        field<CamelbackParameters, &CamelbackParameters::releaseSeconds>("releaseSeconds", .2, 2),
        field<CamelbackParameters, &CamelbackParameters::exitNormalG>("exitNormalG", 1, 2),
        field<CamelbackParameters, &CamelbackParameters::minimumExitPitchDegrees>("minimumExitPitchDegrees", 0, 15)};
    switch (variant) {
    case 0: return {operation.data(), operation.size()};
    case 1: return {hill.data(), hill.size()};
    case 2: return {cliff.data(), cliff.size()};
    case 3: return {turn.data(), turn.size()};
    case 4: return {inversion.data(), inversion.size()};
    case 5: return {sweep.data(), sweep.size()};
    case 6: return {camelback.data(), camelback.size()};
    default: return {};
    }
}

bool parseParameters(int variant, const std::vector<std::string>& tokens, ElementParameters& result, std::string& error) {
    std::unordered_set<std::string> fields;
    const auto descriptors = fieldsForVariant(variant);
    if (!descriptors.data) { error = "invalid recipe variant"; return false; }
    for (const auto& token : tokens) {
        std::string key, value;
        if (!fieldToken(token, key, value)) { error = "recipe fields must be key=value"; return false; }
        const auto found = std::find_if(descriptors.data, descriptors.data + descriptors.size, [&](const FieldDescriptor& field) { return key == field.name; });
        if (found == descriptors.data + descriptors.size) { error = "unknown recipe field: " + key; return false; }
        if (!fields.insert(key).second) { error = "duplicate recipe field: " + key; return false; }
        double number = 0;
        if (!parseNumber(value, number)) { error = "nonfinite or malformed recipe field: " + key; return false; }
        found->set(result, number);
    }
    return true;
}

bool editableField(const RecipeElement& element,std::string_view field) {
    switch(element.role){
        case RideRole::Station:return field=="lengthMeters";
        case RideRole::Brakes:return field=="lengthMeters"||field=="accelerationMps2";
        case RideRole::Departure:case RideRole::CliffLip:return field!="gradeDegrees";
        case RideRole::CliffApproach:return field=="summitHeightMeters"||field=="outwardBankDegrees"||field=="approachLengthMeters"||field=="approachHeadingDegrees";
        case RideRole::CliffDrop:return field=="dropDegrees";
        case RideRole::Wave:return field!="lengthMeters";
        case RideRole::Return:
            if(element.anchor==TerrainAnchor::Approach)return false; // Automatic FVD connection to the fixed station port.
            return !std::holds_alternative<SweepParameters>(element.parameters)||field!="negativeG";
        default:return true;
    }
}
ElementParameters reservedParameters(const RecipeElement& element){
    // Version1 files included fixed metadata from shared parameter families.
    // Retain those values when loading, but expose only parameters that the
    // selected role actually compiles. Silent no-op edits are rejected.
    static const auto prototype=defaultRideRecipe();
    for(const auto& item:prototype.elements)if(item.role==element.role&&item.parameters.index()==element.parameters.index())return item.parameters;
    return std::visit([](const auto& value)->ElementParameters{using T=std::decay_t<decltype(value)>;auto p=T{};if constexpr(std::is_same_v<T,SweepParameters>)p.negativeG=0;return p;},element.parameters);
}
void writeParameters(std::ostream& output, const RecipeElement& element) {
    const auto descriptors = fieldsForVariant(int(element.parameters.index()));
    for (std::size_t i = 0; i < descriptors.size; ++i)if(editableField(element,descriptors.data[i].name))
        output << ' ' << descriptors.data[i].name << '=' << std::setprecision(17) << descriptors.data[i].get(element.parameters);
}

bool variantMatches(RideRole role, const ElementParameters& parameters) {
    const auto is = [&](int index) {
        return int(parameters.index()) == index;
    };
    switch (role) {
    case RideRole::Station: case RideRole::Departure: case RideRole::CliffLip:
    case RideRole::DownhillLaunch: case RideRole::Brakes: return is(0);
    case RideRole::Opening: return is(1);
    case RideRole::Camelback: return is(6);
    case RideRole::CliffApproach: case RideRole::CliffDrop: return is(2);
    case RideRole::Wave: return is(3);
    case RideRole::Loop: case RideRole::Immelmann: return is(4);
    case RideRole::Signature: return is(5);
    case RideRole::Return: return is(1) || is(3) || is(5);
    default: return false;
    }
}

bool validParameters(const ElementParameters& parameters, std::string& error) {
    const auto descriptors = fieldsForVariant(int(parameters.index()));
    if (!descriptors.data) { error = "invalid recipe variant"; return false; }
    for (std::size_t i = 0; i < descriptors.size; ++i) {
        const auto& field = descriptors.data[i];
        const double value = field.get(parameters);
        if (!finiteNumber(value) || value < field.low || value > field.high) {
            error = std::string("recipe field outside bounds: ") + field.name;
            return false;
        }
    }
    return true;
}

RecipeElement element(std::string id, RideRole role, TerrainAnchor anchor, ElementParameters parameters) {
    return {std::move(id), role, anchor, std::move(parameters)};
}

bool parseCount(const std::string& line, std::size_t& count) {
    std::istringstream input(line); input.imbue(std::locale::classic());
    input >> count;
    if (!input || count > maxRecipeElements) return false;
    input >> std::ws; return input.eof();
}

} // namespace

const char* roleName(RideRole role) {
    switch (role) {
    case RideRole::Unspecified: return "unspecified";
    case RideRole::Station: return "station";
    case RideRole::Departure: return "departure";
    case RideRole::Opening: return "opening";
    case RideRole::CliffApproach: return "cliff-approach";
    case RideRole::CliffLip: return "cliff-lip";
    case RideRole::CliffDrop: return "cliff-drop";
    case RideRole::DownhillLaunch: return "downhill-lsm";
    case RideRole::Camelback: return "camelback";
    case RideRole::Wave: return "wave";
    case RideRole::Loop: return "loop";
    case RideRole::Immelmann: return "immelmann";
    case RideRole::Signature: return "signature";
    case RideRole::Return: return "return";
    case RideRole::Brakes: return "brakes";
    }
    return "unspecified";
}

const char* anchorName(TerrainAnchor anchor) {
    switch (anchor) {
    case TerrainAnchor::Station: return "station";
    case TerrainAnchor::Approach: return "approach";
    case TerrainAnchor::Plateau: return "plateau";
    case TerrainAnchor::CliffFoot: return "cliff-foot";
    case TerrainAnchor::WaveBench: return "wave-bench";
    case TerrainAnchor::LoopBasin: return "loop-basin";
    case TerrainAnchor::ImmelmannShoulder: return "immelmann-shoulder";
    case TerrainAnchor::Ravine: return "ravine";
    }
    return "station";
}

uint64_t elementSeed(uint64_t rideSeed, const std::string& elementId) {
    // Hash the seed bytes and stable key only; element position/order never enters
    // this stream. SplitMix64 then diffuses nearby FNV values for independent edits.
    uint64_t hash = 14695981039346656037ull;
    for (unsigned i = 0; i < 8; ++i) { hash ^= (rideSeed >> (i * 8)) & 0xffu; hash *= 1099511628211ull; }
    for (unsigned char c : elementId) { hash ^= c; hash *= 1099511628211ull; }
    hash ^= uint64_t(elementId.size()) + 0x9e3779b97f4a7c15ull;
    uint64_t z = hash + 0x9e3779b97f4a7c15ull;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

RideRecipe defaultRideRecipe() {
    RideRecipe recipe; recipe.name = "Escarpment Rift";
    recipe.elements = {
        element("station", RideRole::Station, TerrainAnchor::Station, OperationParameters{20, 0, 0, 0}),
        element("departure", RideRole::Departure, TerrainAnchor::Approach, OperationParameters{180, 230.4, 0, 0}),
        element("opening", RideRole::Opening, TerrainAnchor::Approach, HillParameters{160, .15, 50, 3.5, 1.2, 1.5, 1}),
        element("cliff-approach", RideRole::CliffApproach, TerrainAnchor::Plateau, CliffParameters{285, 50, 35, 30, 400, -148.016577}),
        element("cliff-lip", RideRole::CliffLip, TerrainAnchor::Plateau, OperationParameters{100, 32, 0, 7}),
        element("cliff-drop", RideRole::CliffDrop, TerrainAnchor::CliffFoot, CliffParameters{285, 32, 88, 30, 220}),
        element("downhill-lsm", RideRole::DownhillLaunch, TerrainAnchor::CliffFoot, OperationParameters{0, 0, -10, 16}),
        element("camelback", RideRole::Camelback, TerrainAnchor::CliffFoot, CamelbackParameters{}),
        element("wave", RideRole::Wave, TerrainAnchor::WaveBench, TurnParameters{180, 90, 73, 3.5, 500}),
        element("loop", RideRole::Loop, TerrainAnchor::LoopBasin, InversionParameters{145, 33, 2, 55, 0}),
        element("immelmann", RideRole::Immelmann, TerrainAnchor::ImmelmannShoulder, InversionParameters{85, 45, 3.8, 55, -5}),
        element("signature", RideRole::Signature, TerrainAnchor::Ravine, SweepParameters{205, -70, -86.720313, 0, -1.2, 45}),
        element("return-crest", RideRole::Return, TerrainAnchor::Ravine, HillParameters{35, -1.25, 0, 3.2, 1.0, 1.5, 1}),
        element("return-sweep", RideRole::Return, TerrainAnchor::Approach, SweepParameters{300, 0, 0, 0, 0, 0}),
        element("brakes", RideRole::Brakes, TerrainAnchor::Station, OperationParameters{250, 0, 0, -7})
    };
    return recipe;
}

bool validateRecipe(const RideRecipe& recipe, std::string& error) {
    if (recipe.version != 1) { error = "unsupported recipe version"; return false; }
    if (recipe.name.empty() || recipe.name.size() > 128) { error = "recipe name must be 1..128 bytes"; return false; }
    if (std::any_of(recipe.name.begin(), recipe.name.end(), [](unsigned char c) { return c < 32 || c == 127; })) { error = "recipe name contains control characters"; return false; }
    if (recipe.elements.empty() || recipe.elements.size() > maxRecipeElements) { error = "recipe element count must be 1..64"; return false; }
    std::unordered_set<std::string> ids;
    std::unordered_map<int, std::size_t> firstRole;
    std::array<bool, int(RideRole::Brakes) + 1> seen{};
    for (std::size_t i = 0; i < recipe.elements.size(); ++i) {
        const auto& current = recipe.elements[i];
        if (!validIdentifier(current.id)) { error = "invalid or oversized element id"; return false; }
        if (!ids.insert(current.id).second) { error = "duplicate element id: " + current.id; return false; }
        if (current.role <= RideRole::Unspecified || current.role > RideRole::Brakes || current.anchor < TerrainAnchor::Station || current.anchor > TerrainAnchor::Ravine) { error = "invalid element role or terrain anchor"; return false; }
        if (!variantMatches(current.role, current.parameters)) { error = "element variant does not match role: " + current.id; return false; }
        if (!validParameters(current.parameters, error)) { error += " in element " + current.id; return false; }
        const auto reserved=reservedParameters(current);const auto descriptors=fieldsForVariant(int(current.parameters.index()));
        for(size_t f=0;f<descriptors.size;++f){const auto& field=descriptors.data[f];
            if(!editableField(current,field.name)&&field.get(current.parameters)!=field.get(reserved)){error="field is fixed metadata for this role: "+current.id+"."+field.name;return false;}}
        if(current.role!=RideRole::Return){
            const auto defaults=defaultRideRecipe();const auto expected=std::find_if(defaults.elements.begin(),defaults.elements.end(),[&](const auto& item){return item.role==current.role;});
            if(expected==defaults.elements.end()||current.anchor!=expected->anchor){error="terrain anchor does not match the element role: "+current.id;return false;}
        }else if(current.anchor!=TerrainAnchor::Approach&&current.anchor!=TerrainAnchor::Ravine){error="return elements require a ravine or final approach anchor";return false;}
        if(const auto* p=std::get_if<OperationParameters>(&current.parameters)){
            if(current.role!=RideRole::DownhillLaunch&&p->lengthMeters<=0){error="operation length must be positive outside the automatic downhill launch";return false;}
            if(current.role!=RideRole::DownhillLaunch&&p->gradeDegrees!=0){error="station, departure and braking corridors require a level grade";return false;}
            if(current.role==RideRole::Brakes&&p->accelerationMps2>=0){error="terminal brake acceleration must be negative";return false;}
        }
        if(current.role==RideRole::Return){
            if(const auto* p=std::get_if<SweepParameters>(&current.parameters);p&&p->negativeG!=0){error="use a return hill for negative-G intent; return sweeps require negativeG=0";return false;}
            if(const auto* p=std::get_if<TurnParameters>(&current.parameters);p&&(p->bankDegrees<=0||p->bankDegrees>=85)){error="return turn bank envelope must be between zero and85degrees";return false;}
            if(current.anchor==TerrainAnchor::Approach&&(i+2!=recipe.elements.size()||!std::holds_alternative<SweepParameters>(current.parameters))){error="the station-approach return must be the final return sweep";return false;}
        }
        const int role = int(current.role);
        if (current.role != RideRole::Return && seen[role]) { error = "required macro role is duplicated: " + std::string(roleName(current.role)); return false; }
        seen[role] = true; firstRole.try_emplace(role, i);
    }
    if (recipe.elements.front().role != RideRole::Station) { error = "recipe must start with station"; return false; }
    if (recipe.elements.size() < 2 || recipe.elements[1].role != RideRole::Departure) { error = "station must be followed by departure"; return false; }
    if (recipe.elements.back().role != RideRole::Brakes) { error = "recipe must end with brakes"; return false; }
    if(recipe.elements.size()<3||recipe.elements[recipe.elements.size()-2].role!=RideRole::Return||recipe.elements[recipe.elements.size()-2].anchor!=TerrainAnchor::Approach){error="recipe requires a final station-approach return before brakes";return false;}
    for (RideRole required : {RideRole::Station, RideRole::Departure, RideRole::Opening, RideRole::CliffApproach,
                              RideRole::CliffLip, RideRole::CliffDrop, RideRole::DownhillLaunch, RideRole::Camelback,
                              RideRole::Wave, RideRole::Loop, RideRole::Immelmann, RideRole::Signature, RideRole::Return, RideRole::Brakes}) {
        if (!seen[int(required)]) { error = "required recipe role is missing: " + std::string(roleName(required)); return false; }
    }
    const auto at = [&](RideRole role) { return firstRole.at(int(role)); };
    if (!(at(RideRole::Opening) < at(RideRole::CliffApproach) && at(RideRole::CliffApproach) < at(RideRole::CliffLip) && at(RideRole::CliffLip) < at(RideRole::CliffDrop))) {
        error = "opening and cliff roles are out of order"; return false;
    }
    const auto cliff = at(RideRole::CliffDrop);
    if (cliff + 2 >= recipe.elements.size() || recipe.elements[cliff + 1].role != RideRole::DownhillLaunch || recipe.elements[cliff + 2].role != RideRole::Camelback) {
        error = "cliff-drop, downhill-lsm and camelback must be adjacent"; return false;
    }
    if (!(at(RideRole::Camelback) < at(RideRole::Wave) && at(RideRole::Wave) < at(RideRole::Loop) &&
          at(RideRole::Loop) < at(RideRole::Immelmann) && at(RideRole::Immelmann) < at(RideRole::Signature) &&
          at(RideRole::Signature) < at(RideRole::Return) && at(RideRole::Return) < at(RideRole::Brakes))) {
        error = "macro elements are out of order"; return false;
    }
    return true;
}

std::string recipePayload(const RideRecipe& recipe) {
    std::string error;
    if (!validateRecipe(recipe, error)) return {};
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << "VIBECOASTER_RECIPE 1\n" << std::quoted(recipe.name) << '\n' << recipe.elements.size() << '\n';
    for (const auto& item : recipe.elements) {
        output << std::quoted(item.id) << ' ' << roleName(item.role) << ' ' << anchorName(item.anchor) << ' ' << variantName(item.parameters);
        writeParameters(output, item);
        output << '\n';
    }
    const auto text = output.str();
    return text.size() <= maxRecipeBytes ? text : std::string();
}

bool parseRecipe(const std::string& text, RideRecipe& destination, std::string& error) {
    if (text.size() > maxRecipeBytes) { error = "recipe exceeds 1 MiB"; return false; }
    std::istringstream input(text); input.imbue(std::locale::classic());
    std::string line, name;
    std::size_t count = 0;
    if (!std::getline(input, line)) { error = "invalid recipe header"; return false; }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "VIBECOASTER_RECIPE 1") { error = "invalid recipe header"; return false; }
    if (!std::getline(input, line)) { error = "invalid quoted recipe name"; return false; }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!parseQuotedLine(line, name)) { error = "invalid quoted recipe name"; return false; }
    if (!std::getline(input, line)) { error = "invalid recipe element count"; return false; }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!parseCount(line, count) || count == 0) { error = "invalid recipe element count"; return false; }
    RideRecipe parsed; parsed.version = 1; parsed.name = std::move(name); parsed.elements.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::getline(input, line) || line.empty()) { error = "truncated recipe element list"; return false; }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream row(line); row.imbue(std::locale::classic());
        RecipeElement item; std::string role, anchor, variant;
        if (!(row >> std::quoted(item.id) >> role >> anchor >> variant)) { error = "malformed recipe element header"; return false; }
        if (!parseRole(role, item.role) || item.role == RideRole::Unspecified) { error = "unknown recipe role: " + role; return false; }
        if (!parseAnchor(anchor, item.anchor)) { error = "unknown recipe anchor: " + anchor; return false; }
        int variantIndex = -1;
        if (!parseVariant(variant, variantIndex)) { error = "unknown recipe variant: " + variant; return false; }
        switch (variantIndex) {
        case 0: item.parameters = OperationParameters{}; break;
        case 1: item.parameters = HillParameters{}; break;
        case 2: item.parameters = CliffParameters{}; break;
        case 3: item.parameters = TurnParameters{}; break;
        case 4: item.parameters = InversionParameters{}; break;
        case 5: item.parameters = SweepParameters{}; break;
        case 6: item.parameters = CamelbackParameters{}; break;
        }
        const auto reserved=reservedParameters(item);const auto descriptors=fieldsForVariant(variantIndex);
        for(size_t f=0;f<descriptors.size;++f){const auto& field=descriptors.data[f];if(!editableField(item,field.name))field.set(item.parameters,field.get(reserved));}
        std::vector<std::string> fields; std::string token;
        while (row >> token) fields.push_back(std::move(token));
        if (!parseParameters(variantIndex, fields, item.parameters, error)) return false;
        parsed.elements.push_back(std::move(item));
    }
    input >> std::ws;
    if (!input.eof()) { error = "trailing recipe data"; return false; }
    if (!validateRecipe(parsed, error)) return false;
    destination = std::move(parsed);
    return true;
}

bool loadRecipe(const std::string& path, RideRecipe& destination, std::string& error) {
    std::ifstream input(utf8path(path), std::ios::binary);
    if (!input) { error = "cannot open recipe file"; return false; }
    std::string text(maxRecipeBytes + 1, '\0');
    input.read(text.data(), std::streamsize(text.size()));
    const auto read = input.gcount();
    if (read < 0 || std::size_t(read) > maxRecipeBytes) { error = "recipe exceeds 1 MiB"; return false; }
    text.resize(std::size_t(read));
    RideRecipe parsed;
    if (!parseRecipe(text, parsed, error)) return false;
    destination = std::move(parsed);
    return true;
}

bool saveRecipe(const RideRecipe& recipe, const std::string& path, std::string& error) {
    const auto text = recipePayload(recipe);
    if (text.empty()) { error = "cannot save invalid or oversized recipe"; return false; }
    const auto target = utf8path(path);
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    static std::atomic<std::uint64_t> nextTemporary{0};
    TemporaryRecipeFile temporary;
    try {
        for (int attempt = 0; attempt < 64 && !temporary.stream; ++attempt) {
#ifdef _WIN32
            const auto process = GetCurrentProcessId();
#else
            const auto process = getpid();
#endif
            temporary.path = utf8path(path + ".tmp.recipe." + std::to_string(process) + "." +
                std::to_string(stamp) + "." + std::to_string(nextTemporary.fetch_add(1)));
#ifdef _WIN32
            int descriptor = -1;
            const int openError = _wsopen_s(&descriptor, temporary.path.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY,
                _SH_DENYRW, _S_IREAD | _S_IWRITE);
            if (descriptor < 0) {
                if (openError == EEXIST) continue;
                error = "cannot create exclusive temporary recipe"; return false;
            }
            temporary.stream = _fdopen(descriptor, "wb");
            if (!temporary.stream) { _close(descriptor); error = "cannot open temporary recipe stream"; return false; }
#else
            const int descriptor = ::open(temporary.path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
            if (descriptor < 0) {
                if (errno == EEXIST) continue;
                error = "cannot create exclusive temporary recipe"; return false;
            }
            temporary.stream = ::fdopen(descriptor, "wb");
            if (!temporary.stream) { ::close(descriptor); error = "cannot open temporary recipe stream"; return false; }
#endif
            temporary.armed = true;
        }
        if (!temporary.stream) { error = "cannot allocate unique temporary recipe"; return false; }
        const bool written = std::fwrite(text.data(), 1, text.size(), temporary.stream) == text.size();
        const int closed = std::fclose(temporary.stream); temporary.stream = nullptr;
        if (!written || closed != 0) { error = "cannot write temporary recipe"; return false; }
#ifdef _WIN32
        if (!MoveFileExW(temporary.path.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
            error = "atomic recipe replacement failed"; return false;
        }
#else
        std::error_code replacementError; std::filesystem::rename(temporary.path, target, replacementError);
        if (replacementError) { error = "atomic recipe replacement failed: " + replacementError.message(); return false; }
#endif
        temporary.armed = false;
        return true;
    } catch (const std::exception& exception) {
        error = exception.what(); return false;
    }
}

} // namespace coaster
