#include "CoordinateContract.h"
#include <iostream>
#include <stdexcept>

using namespace VibeCoordinates;
static coaster::Vec3 vector(Triple P) { return {P.X, P.Y, P.Z}; }
static void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
int main()
{
    const coaster::Vec3 Source{1.25, -7.2, 12};
    require(coaster::norm(CorePosition(Position(Source)) - Source) < 1e-12, "coordinate round trip");
    require(coaster::norm(vector(Position({1, 2, 3})) - coaster::Vec3{100, -200, 300}) < 1e-12, "scale and reflection");
    const coaster::Vec3 Forward{1, 0, 0}, Up{0, 0, 1}, Right = coaster::cross(Forward, Up);
    require(coaster::norm(vector(Direction(Right)) - coaster::Vec3{0, 1, 0}) < 1e-12, "rider right handedness");
    for (double angle : {0., .7, -1.3, coaster::pi})
    {
        const auto BankUp = coaster::rotate(Up, Forward, angle);
        const auto BankRight = coaster::cross(Forward, BankUp);
        require(coaster::norm(coaster::cross(vector(Direction(BankUp)), vector(Direction(Forward))) - vector(Direction(BankRight))) < 1e-12, "banked basis must agree with UE MakeFromXZ");
    }
    const auto A = vector(Position({0, 0, 0})), B = vector(Position({1, 0, 0})), C = vector(Position({0, 1, 0}));
    require(coaster::dot(coaster::cross(C - A, B - A), vector(Direction(Up))) > 0, "reflected terrain indices must reverse winding");
    for (const auto To : {coaster::Vec3{2, 3, 8}, coaster::Vec3{-4, 5, 9}})
    {
        const coaster::Vec3 From{2, 3, 0};
        const auto Member = SteelMember(From, To);
        const auto Centre = vector(Member.Centre), Axis = vector(Member.Axis);
        require(coaster::norm(Centre - Axis * (Member.LengthCm * .5) - vector(Position(From))) < 1e-9, "canonical member base endpoint");
        require(coaster::norm(Centre + Axis * (Member.LengthCm * .5) - vector(Position(To))) < 1e-9, "canonical member attachment endpoint");
    }
    for (double angle : {0., .7, -1.3})
    {
        const auto F = coaster::rotate(Forward, Up, angle), R = coaster::cross(F, Up);
        const coaster::StationBox Box{Source, F, R, Up, {41., 1.925, .4}, coaster::StationRole::Platform};
        const auto B = VibeCoordinates::StationBox(Box);
        const auto X = vector(B.Forward), Z = vector(B.Up), Y = coaster::cross(Z, X);
        for (double sx : {-1., 1.}) for (double sy : {-1., 1.}) for (double sz : {-1., 1.})
        {
            const auto EngineCorner = vector(B.Centre) + X*(sx*50*B.Scale.X) + Y*(sy*50*B.Scale.Y) + Z*(sz*50*B.Scale.Z);
            const auto CoreCorner = Box.center + Box.forward*(sx*Box.half.x) + Box.right*(sy*Box.half.y) + Box.up*(sz*Box.half.z);
            require(coaster::norm(EngineCorner-vector(Position(CoreCorner)))<1e-9, "canonical station cube corner");
        }
    }
    const coaster::TrainConfig Train;
    require(std::abs(coaster::seatDistanceOffset(Train, 0) - 8.5) < 1e-12, "front seat distance");
    require(std::abs(coaster::seatDistanceOffset(Train, 1) - 1.7) < 1e-12, "middle seat distance");
    require(std::abs(coaster::seatDistanceOffset(Train, 2) + 8.5) < 1e-12, "rear seat distance");
    std::cout << "PASS: native-to-Unreal coordinate, handedness, winding, support-member and seat contracts\n";
}


