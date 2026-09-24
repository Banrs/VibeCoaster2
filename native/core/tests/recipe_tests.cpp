#include "coaster/recipe.hpp"
#include "coaster/coaster.hpp"
#include "../src/recipe_compiler.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
int checks = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) { std::cerr << "FAIL " << message << '\n'; std::exit(1); }
}

std::string replaceOnce(std::string text, const std::string& from, const std::string& to) {
    const auto original = text;
    const auto at = text.find(from);
    check(at != std::string::npos, "malformed recipe fixture mutation token exists");
    text.replace(at, from.size(), to);
    check(text != original, "malformed recipe fixture mutation changes payload");
    return text;
}

void authorshipTwistFlags() {
    using namespace coaster;
    Design original;
    for(const std::string name:{"source-one","source-two"}){ForceAuthoring f;f.name=name;f.sourceDistances={0,1};original.forcePrograms.push_back(f);}
    original.forcePrograms[1].program.gravityReferencedRoll=true;
    const auto legacy=authorshipPayload(original);
    check(legacy.find("ADDITIVE_TWISTS")==std::string::npos,"Historical replacement twists retain their exact source payload format");
    original.forcePrograms[1].program.additiveTwists=true;
    const auto extended=authorshipPayload(original);Design parsed;std::string error;
    check(parseAuthorshipPayload(extended,parsed,error)&&parsed.forcePrograms.size()==2&&
        !parsed.forcePrograms[0].program.additiveTwists&&parsed.forcePrograms[1].program.additiveTwists&&parsed.forcePrograms[1].program.gravityReferencedRoll,
        "Mixed additive/replacement twist semantics round trip alongside the existing bank-reference extension");
    check(authorshipPayload(parsed)==extended,"Additive-twist source serialization is canonical and byte stable");
    check(parseAuthorshipPayload(legacy,parsed,error)&&!parsed.forcePrograms[0].program.additiveTwists&&!parsed.forcePrograms[1].program.additiveTwists,
        "Absent additive flags reset to historical replacement semantics when loading into an existing design");
    check(authorshipPayload(parsed)==legacy,"Old bank-referenced sources resave without a new extension");
    const std::string valid="ADDITIVE_TWISTS 1 2 0 1\n";
    for(const std::string bad:{"ADDITIVE_TWISTS 2 2 0 1\n","ADDITIVE_TWISTS 1 1 1\n","ADDITIVE_TWISTS 1 3 0 1 0\n",
        "ADDITIVE_TWISTS 1 2 0 2\n","ADDITIVE_TWISTS 1 2 -1 1\n","ADDITIVE_TWISTS 1 2 0 true\n","ADDITIVE_TWISTS 1 2 0\n"}){
        const auto malformed=replaceOnce(extended,valid,bad);Design retained=original;
        check(!parseAuthorshipPayload(malformed,retained,error)&&authorshipPayload(retained)==extended,
            "Malformed twist version/count/flag rejects without replacing retained authored sources");
    }
    for(const auto& duplicate:std::array<std::string,2>{valid,"BANK_REFERENCE 1 1\n"}){
        Design retained=original;
        check(!parseAuthorshipPayload(extended+duplicate,retained,error)&&authorshipPayload(retained)==extended,
            "Duplicate source semantic extensions reject without mutating authored data");
    }
}

// A banked circular circuit has exact derivative jets and constant elevation.
// Its synthetic constant-speed trace makes the semantic timing independently
// calculable, without running generation or prescribing any real ride shape.
void motionDiagnostics() {
    using namespace coaster;
    Design d;d.track.closed=true;d.track.authoredGeometry=d.track.authoredFrame=true;
    constexpr double radius=200,speed=40,bank=.6;
    const double sb=std::sin(bank),cb=std::cos(bank);
    for(int i=0;i<=400;++i){const double a=2*pi*i/400,c=std::cos(a),s=std::sin(a);
        d.track.knots.push_back({{radius*s,radius*(1-c),20},{c,s,0},{-s/radius,c/radius,0},{-s*sb,c*sb,cb},0,Element::Turn,
            {-c/(radius*radius),-s/(radius*radius),0},{s/(radius*radius*radius),-c/(radius*radius*radius),0},
            {-c*sb/radius,-s*sb/radius,0},{s*sb/(radius*radius),-c*sb/(radius*radius),0},
            {c*sb/(radius*radius*radius),s*sb/(radius*radius*radius),0}});
    }
    d.track.knots.back()=d.track.knots.front();d.track.rebuild();
    d.request.recipe=defaultRideRecipe();const double span=d.track.length/d.request.recipe.elements.size();
    for(size_t i=0;i<d.request.recipe.elements.size();++i){const auto& element=d.request.recipe.elements[i];
        d.sections.push_back({element.id,span*i,span*(i+1),0,false,element.role,element.id});
    }
    auto section=[&](RideRole role)->const RideSection& {
        const auto found=std::find_if(d.sections.begin(),d.sections.end(),[&](const RideSection& q){return q.role==role;});
        check(found!=d.sections.end(),"Diagnostic fixture has the required semantic section");return *found;
    };
    const double plateau=section(RideRole::CliffApproach).start,lip=section(RideRole::CliffLip).start;
    const double departure=section(RideRole::CliffDrop).start,signatureEnd=section(RideRole::Signature).end;
    d.landmarks={{LandmarkKind::OpeningCrest,section(RideRole::Opening).start},
        {LandmarkKind::OpeningRecovery,section(RideRole::Opening).end},{LandmarkKind::PlateauArrival,plateau},
        {LandmarkKind::CliffDeparture,departure},{LandmarkKind::DownhillLaunchExit,section(RideRole::DownhillLaunch).end},
        {LandmarkKind::CamelbackCrest,section(RideRole::Camelback).start},{LandmarkKind::WaveCrest,section(RideRole::Wave).start},
        {LandmarkKind::LoopCrest,section(RideRole::Loop).start},{LandmarkKind::ImmelmannCrest,section(RideRole::Immelmann).start},
        {LandmarkKind::SignatureRelease,section(RideRole::Signature).start},{LandmarkKind::BrakeEntry,section(RideRole::Brakes).start}};
    for(int i=0;i<=1600;++i){Frame f;f.distance=d.track.length*i/1600;f.time=f.distance/speed;f.speed=speed;d.simulation.frames.push_back(f);}
    d.simulation.completed=true;d.simulation.metrics.duration=d.track.length/speed;
    assessMotion(d);
    check(d.motion.performed&&d.motion.longestLevelCoastSeconds>25&&d.motion.longestFlatCoastSeconds<1e-6,
        "Long banked horizontal coast is exposed separately from the historical frame-hold metric");
    check(d.motion.longestLevelCoastStartDistance>=0&&d.motion.longestLevelCoastEndDistance>d.motion.longestLevelCoastStartDistance,
        "The longest near-level coast identifies its actual circuit interval");
    double returnLength=0;for(const auto& q:d.sections)if(q.role==RideRole::Return)returnLength+=q.end-q.start;
    check(std::abs(d.motion.returnLevelCoastSeconds-returnLength/speed)<.05&&std::abs(d.motion.longestReturnLevelCoastSeconds-returnLength/speed)<.05,
        "Consecutive Return-role sections accumulate a separately measured banked level backhaul");
    check(std::abs(d.motion.clifftopActiveSeconds-(lip-plateau)/speed)<1e-9&&
        std::abs(d.motion.clifftopBrakingSeconds-(departure-lip)/speed)<1e-9,
        "Clifftop riding and lip intervals have independently calculated front-seat durations");
    check(std::abs(d.motion.returnSeconds-(d.track.length-signatureEnd+seatDistanceOffset(d.request.train,0))/speed)<1e-9,
        "Return duration runs from front-seat signature exit to the actual complete stop");
    check(std::none_of(d.report.errors.begin(),d.report.errors.end(),[](const Finding& f){return f.code=="CLIFFTOP_ACT"||f.code=="RETURN_PACING"||f.code=="WAITING_TRACK";}),
        "Composition diagnostics introduce no minimum-duration quota or new acceptance rejection");
    const auto json=motionReportJson(d);
    check(json.find("\"clifftopActiveSeconds\":")!=std::string::npos&&json.find("\"returnLevelCoastSeconds\":")!=std::string::npos,
        "Computed pacing and level-return evidence is included in the report");
    auto split=d;split.report={};
    for(size_t i=0;i<split.sections.size();++i)if(split.sections[i].role==RideRole::CliffLip||split.sections[i].role==RideRole::Signature){
        auto tail=split.sections[i];const double middle=(tail.start+tail.end)*.5;split.sections[i].end=middle;tail.start=middle;
        split.sections.insert(split.sections.begin()+i+1,tail);++i;
    }
    assessMotion(split);
    check(split.motion.clifftopActiveSeconds==d.motion.clifftopActiveSeconds&&split.motion.clifftopBrakingSeconds==d.motion.clifftopBrakingSeconds&&split.motion.returnSeconds==d.motion.returnSeconds,
        "Section subdivision preserves first-lip and final-signature timing boundaries");
    auto powered=d;powered.report={};powered.operations.push_back({section(RideRole::Return).start,section(RideRole::Brakes).start,DriveKind::Brake,10,15000,1500000,.5});
    assessMotion(powered);
    check(powered.motion.returnLevelCoastSeconds==0&&powered.motion.longestReturnLevelCoastSeconds==0,
        "Installed hardware contact is excluded from unpowered return coasting");
    auto unknown=d;unknown.report={};unknown.request.recipe.elements.clear();unknown.landmarks.clear();
    for(auto& q:unknown.sections)q.role=RideRole::Unspecified;
    assessMotion(unknown);const auto unknownJson=motionReportJson(unknown);
    check(std::isnan(unknown.motion.clifftopActiveSeconds)&&std::isnan(unknown.motion.returnSeconds)&&
        unknownJson.find("\"clifftopActiveSeconds\":null")!=std::string::npos&&unknownJson.find("\"returnSeconds\":null")!=std::string::npos,
        "Missing semantic boundaries remain unknown instead of reporting a fabricated zero duration");
}

void energyBootstrap() {
    using namespace coaster;
    Design partial;partial.track.closed=false;
    for(int i=0;i<=4;++i)partial.track.knots.push_back({{30.*i,0,20},{1,0,0},{0,0,0},{0,0,1},0,Element::Launch});
    partial.track.rebuild();
    partial.operations.push_back({3,95,DriveKind::Launch,30,30000,3000000,.5});
    partial.report.fail("AUTHORING_PARTIAL","Synthetic unfinished source");
    const auto originalKnots=partial.track.knots;
    const double originalLength=partial.track.length;
    RecipeCompileFailure failed("pending source",partial,{{"reached",70,5}},RecipePort{"pending",partial.track.length,5});
    RecipeFeedback feedback;feedback.energyCorrection["retained"]=17;
    const auto corrected=bootstrapRecipeEnergy(failed,feedback);
    check(corrected.corrected&&!corrected.cancelled&&corrected.report.valid()&&corrected.observations.size()==2,
        "Powered open prefix supplies reached and provisional pending source corrections");
    check(corrected.observations[0].port.id=="reached"&&!corrected.observations[0].provisional&&
        corrected.observations[1].port.id=="pending"&&corrected.observations[1].provisional&&
        corrected.maximumSpeedCorrection>.05&&feedback.energyCorrection["retained"]==17&&
        feedback.energyCorrection.count("reached")==1&&feedback.energyCorrection.count("pending")==1,
        "Bootstrap records real prefix observations and changes only source-energy feedback");
    for(const auto& observation:corrected.observations) {
        const double expected=observation.estimatedSpeed*observation.estimatedSpeed-observation.port.speed*observation.port.speed;
        check(std::abs(feedback.energyCorrection.at(observation.port.id)-expected)<1e-8,
            "Bootstrap feeds squared-speed differences, not assigned simulation speeds");
    }
    check(failed.partial.track.knots.size()==originalKnots.size()&&failed.partial.track.length==originalLength&&
        failed.partial.operations.size()==1&&failed.partial.simulation.frames.empty(),
        "Provisional continuation and replay leave the input Design geometry and simulation untouched");
    for(size_t i=0;i<originalKnots.size();++i)
        check(norm(failed.partial.track.knots[i].position-originalKnots[i].position)==0&&
            norm(failed.partial.track.knots[i].tangent-originalKnots[i].tangent)==0,
            "Bootstrap preserves every authored prefix knot");
    std::string saveError;
    check(!failed.partial.accepted()&&!saveDesign(failed.partial,"",saveError)&&saveError.find("REJECTED_DESIGN")==0,
        "A bootstrapped partial Design cannot be accepted or saved");

    auto noProgress=failed;noProgress.reachedPorts.clear();
    noProgress.pendingPort->speed=corrected.observations.back().estimatedSpeed;
    RecipeFeedback untouched;untouched.energyCorrection["retained"]=17;
    const auto stalled=bootstrapRecipeEnergy(noProgress,untouched);
    check(!stalled.corrected&&!stalled.report.valid()&&stalled.report.errors.back().code=="ENERGY_BOOTSTRAP_NO_PROGRESS"&&
        untouched.energyCorrection.size()==1&&untouched.energyCorrection.at("retained")==17,
        "A replay with no meaningful speed correction preserves feedback");

    auto unpowered=failed;unpowered.partial.operations.clear();
    const auto replayFailed=bootstrapRecipeEnergy(unpowered,untouched);
    check(!replayFailed.corrected&&!replayFailed.report.valid()&&replayFailed.report.errors.back().code=="ENERGY_BOOTSTRAP_REPLAY"&&
        untouched.energyCorrection.size()==1&&untouched.energyCorrection.at("retained")==17,
        "Failed provisional replay preserves feedback");
    int polls=0;
    const auto cancelled=bootstrapRecipeEnergy(failed,untouched,[&]{return ++polls>50;});
    check(cancelled.cancelled&&!cancelled.corrected&&polls>50&&untouched.energyCorrection.size()==1&&
        untouched.energyCorrection.at("retained")==17,
        "Cancellation during provisional replay preserves feedback");
}

} // namespace

int main() {
    using namespace coaster;
    authorshipTwistFlags();
    motionDiagnostics();
    energyBootstrap();
    const auto recipe = defaultRideRecipe();
    std::string error;
    check(validateRecipe(recipe, error), "default recipe validates");
    check(std::count_if(recipe.elements.begin(), recipe.elements.end(), [](const auto& item) { return item.role == RideRole::Return; }) >= 2, "default recipe contains multiple ordinary returns");
    check(recipe.elements.front().role == RideRole::Station && recipe.elements[1].role == RideRole::Departure && recipe.elements.back().role == RideRole::Brakes, "default starts station/departure and ends brakes");
    check(recipe.elements[5].role == RideRole::CliffDrop && recipe.elements[6].role == RideRole::DownhillLaunch && recipe.elements[7].role == RideRole::Camelback, "cliff/downhill/camelback adjacency is explicit");
    check(std::get<OperationParameters>(recipe.elements[6].parameters).targetSpeedKmh == 0, "downhill LSM zero target derives the global speed");
    const auto& camelback = std::get<CamelbackParameters>(recipe.elements[7].parameters);
    check(camelback.profileScale > 0 && camelback.tailCutSeconds >= 0 && camelback.releaseSeconds > 0,
        "camelback uses the protected approved parameter family");
    check(camelback.minimumExitPitchDegrees > 0 && camelback.exitNormalG > 0,
        "camelback retains its approved exit constraints");
    for(int field=0;field<5;++field){auto invalid=recipe;auto& sweep=std::get<SweepParameters>(invalid.elements[invalid.elements.size()-2].parameters);
        if(field==0)sweep.lengthMeters+=1;if(field==1)sweep.riseMeters=1;if(field==2)sweep.headingDegrees=1;
        if(field==3)sweep.exitPitchDegrees=1;if(field==4)sweep.rollDegrees=1;
        check(!validateRecipe(invalid,error),"Automatic station approach rejects ignored or unsupported geometry edits");}
    for(bool shortRelease:{false,true}){auto invalid=recipe;auto& protectedBody=std::get<CamelbackParameters>(invalid.elements[7].parameters);
        if(shortRelease)protectedBody.releaseSeconds=.15;else protectedBody.tailCutSeconds=1.5;
        check(!validateRecipe(invalid,error),"Protected camelback recipe bounds agree with its source constructor");}


    check(std::string(roleName(RideRole::DownhillLaunch)) == "downhill-lsm", "role names are stable human tokens");
    check(std::string(anchorName(TerrainAnchor::ImmelmannShoulder)) == "immelmann-shoulder", "anchor names are stable human tokens");

    const auto seedA = elementSeed(42, "camelback");
    const auto seedB = elementSeed(42, "camelback");
    check(seedA == seedB && seedA == 15497263119282846436ull, "camelback seed has a fixed stable vector");
    check(elementSeed(42, "wave") == 11811803777310210526ull && elementSeed(43, "camelback") == 9650049089560291631ull, "seed changes with stable key or ride seed");

    const auto payload = recipePayload(recipe);
    check(!payload.empty() && payload.rfind("VIBECOASTER_RECIPE 1\n", 0) == 0, "canonical recipe payload has versioned header");
    RideRecipe parsed;
    check(parseRecipe(payload, parsed, error) && parsed == recipe, "canonical recipe round trips exactly");
    check(recipePayload(parsed) == payload, "canonical writer is byte stable");
    auto historicalHeading=recipe;
    for(auto& e:historicalHeading.elements)if(e.role==RideRole::CliffApproach)
        std::get<CliffParameters>(e.parameters).approachHeadingDegrees=-65;
    auto oldHeadingPayload=recipePayload(historicalHeading);
    const std::string headingField=" approachHeadingDegrees=-65";
    for(auto at=oldHeadingPayload.find(headingField);at!=std::string::npos;at=oldHeadingPayload.find(headingField))
        oldHeadingPayload.erase(at,headingField.size());
    RideRecipe oldHeadingRecipe;
    check(parseRecipe(oldHeadingPayload,oldHeadingRecipe,error)&&oldHeadingRecipe==historicalHeading,
        "Old recipes retain their historical clifftop heading when the new field is absent");
    auto directed=recipe;
    for(auto& e:directed.elements)if(e.role==RideRole::CliffApproach)
        std::get<CliffParameters>(e.parameters).approachHeadingDegrees=-150;
    RideRecipe directedParsed;
    check(parseRecipe(recipePayload(directed),directedParsed,error)&&directedParsed==directed,
        "Explicit clifftop bearing remains editable and serializable");
    auto yawing=recipe;
    for(auto& e:yawing.elements)if(e.role==RideRole::Immelmann)std::get<InversionParameters>(e.parameters).yawDegrees=60;
    RideRecipe yawingParsed;
    check(parseRecipe(recipePayload(yawing),yawingParsed,error)&&yawingParsed==yawing,
        "The compiled Immelmann half-loop yaw remains editable and survives recipe persistence");
    check(payload.find("lipSpeedKmh=")==std::string::npos,"Unused cliff speed metadata is absent from editable recipes");
    auto noOp=recipe;std::get<CliffParameters>(noOp.elements[3].parameters).lipSpeedKmh+=1;
    check(!validateRecipe(noOp,error),"A silent no-op edit to role-specific fixed metadata is rejected");
    auto wrongAnchor=recipe;wrongAnchor.elements[7].anchor=TerrainAnchor::Station;
    check(!validateRecipe(wrongAnchor,error),"Protected placement intent cannot silently ignore an unrelated terrain anchor");

    RideRecipe customized = recipe;
    customized.elements.insert(customized.elements.end()-2,
        RecipeElement{"return-valley", RideRole::Return, TerrainAnchor::Ravine, TurnParameters{-40,-2,35,1,300}});
    std::swap(customized.elements[12], customized.elements[13]);
    customized.elements.insert(customized.elements.end() - 2,
        RecipeElement{"return-extra", RideRole::Return, TerrainAnchor::Ravine, SweepParameters{180, 8, 20, 0, 0, 5}});
    check(validateRecipe(customized, error), "compatible return reorder and unique insertion validate");
    RideRecipe customizedParsed;
    check(parseRecipe(recipePayload(customized), customizedParsed, error) && customizedParsed == customized,
        "compatible return reorder and unique insertion survive serialization");
    check(elementSeed(42, "camelback") == seedA, "existing element seed is unaffected by recipe edits");

    RideRecipe camelbackEdit = recipe;
    auto& editedCamelback = std::get<CamelbackParameters>(camelbackEdit.elements[7].parameters);
    editedCamelback.profileScale = 1.05;
    editedCamelback.tailCutSeconds = .8;
    check(validateRecipe(camelbackEdit, error), "approved camelback edits remain within its protected family");
    RideRecipe camelbackParsed;
    check(parseRecipe(recipePayload(camelbackEdit), camelbackParsed, error) && camelbackParsed == camelbackEdit,
        "approved camelback parameter edits round trip");

    RideRecipe hillReturn = recipe;
    hillReturn.elements[12].parameters = HillParameters{};
    check(validateRecipe(hillReturn, error), "return role accepts hill variant");
    hillReturn.elements[12].parameters = TurnParameters{};
    check(validateRecipe(hillReturn, error), "return role accepts turn variant");

    const auto malformed = [&](const std::string& text, const char* message) {
        RideRecipe retained = recipe;
        check(!parseRecipe(text, retained, error) && retained == recipe, message);
    };
    malformed(replaceOnce(payload, "lengthMeters=20", "lengthMeters=20 lengthMeters=21"), "duplicate fields reject without mutating output");
    malformed(replaceOnce(payload, "lengthMeters=20", "mystery=1 lengthMeters=20"), "unknown fields reject");
    malformed(replaceOnce(payload, "targetSpeedKmh=0", "targetSpeedKmh=nan"), "nonfinite numbers reject");
    malformed(replaceOnce(payload, "\"wave\" wave", "\"station\" wave"), "duplicate ids reject");
    malformed(payload + "trailing-junk\n", "trailing data rejects");
    malformed(replaceOnce(payload, std::to_string(recipe.elements.size())+"\n", "65\n"), "element count bound rejects");
    malformed(replaceOnce(payload, "approved-camelback-v1 profileScale=", "approved-camelback-v1 twistDegrees=1 profileScale="),
        "protected camelback rejects hill-only twist fields");

    const auto folder = std::filesystem::temp_directory_path() / ("vibecoaster-recipe-tests-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    check(std::filesystem::create_directory(folder), "unique recipe test directory created");
    const auto path = folder / "roundtrip.recipe";
    check(saveRecipe(recipe, path.string(), error), "recipe saves atomically");
    RideRecipe loaded;
    check(loadRecipe(path.string(), loaded, error) && loaded == recipe, "recipe file loads exactly");
    auto edited = loaded;
    std::get<SweepParameters>(edited.elements[11].parameters).rollDegrees = 52;
    check(saveRecipe(edited, path.string(), error), "edited recipe overwrites atomically");
    RideRecipe editedLoaded;
    check(loadRecipe(path.string(), editedLoaded, error) && editedLoaded == edited, "edited recipe survives save/load/re-edit roundtrip");
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::filesystem::remove(folder, ignored);

    std::cout << "PASS " << checks << " recipe format, validation, keyed seeds and atomic roundtrip checks\n";
    return 0;
}
