#include "coaster/persistence.hpp"
#include <bit>
#include <fstream>
#include <span>
#include <chrono>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace coaster {
namespace {
struct Bytes {
    std::vector<unsigned char> data;
    std::size_t pos{};
    void integer(std::uint64_t value) {
        for (int i = 0; i < 8; ++i)
            data.push_back(static_cast<unsigned char>((value >> (i * 8)) & 255));
    }
    std::uint64_t integer() {
        if (pos + 8 > data.size())
            throw std::runtime_error("Truncated saved design");
        std::uint64_t value = 0;
        for (int i = 0; i < 8; ++i)
            value |= std::uint64_t(data[pos++]) << (i * 8);
        return value;
    }
    void number(double value) {
        if (!std::isfinite(value))
            throw std::runtime_error("Cannot persist nonfinite data");
        integer(std::bit_cast<std::uint64_t>(value));
    }
    double number() {
        const double value = std::bit_cast<double>(integer());
        if (!std::isfinite(value))
            throw std::runtime_error("Nonfinite saved data");
        return value;
    }
    void text(const std::string &text) {
        integer(text.size());
        data.insert(data.end(), text.begin(), text.end());
    }
    std::string text() {
        const auto size = integer();
        if (size > 256 || size > data.size() - pos)
            throw std::runtime_error("Invalid saved string");
        std::string value(data.begin() + pos, data.begin() + pos + std::size_t(size));
        pos += std::size_t(size);
        return value;
    }
    void vec(Vec3 v) {
        number(v.x);
        number(v.y);
        number(v.z);
    }
    Vec3 vec() {
        const double x = number(), y = number(), z = number();
        return {x, y, z};
    }
};
std::uint64_t checksum(std::span<const unsigned char> bytes) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (auto b : bytes) {
        hash ^= b;
        hash *= 1099511628211ULL;
    }
    return hash;
}
constexpr std::uint64_t magic = 0x315744464356;
} // namespace
void saveDesign(const Design &design, const std::filesystem::path &path, const Cancel &cancel) {
    validateRecipe(design.recipe);
    Bytes b;
    b.integer(magic);
    b.integer(2);
    const auto &r = design.recipe;
    b.integer(r.seed);
    b.text(r.style);
    for (double v : {r.plateau, r.openingHeight, r.camelbackHeight, r.loopHeight, r.immelmannHeight,
                     r.topSpeedKph, r.activeSeconds, r.terminalSeconds})
        b.number(v);
    b.integer(design.track.source.size());
    for (const auto &p : design.track.source) {
        poll(cancel);
        validateProgram(p);
        b.text(p.id);
        b.text(p.label);
        b.integer(static_cast<std::uint64_t>(p.role));
        b.vec(p.initial.p);
        b.vec(p.initial.t);
        b.vec(p.initial.u);
        b.number(p.initial.v);
        b.number(p.initial.s);
        b.number(p.initial.lossWork);
        b.number(p.initial.driveWork);
        b.number(p.rolling);
        b.number(p.drag);
        b.integer(p.gravityBank);
        b.integer(p.controls.size());
        for (const auto &c : p.controls) {
            for (double v : {c.time, c.normal, c.lateral, c.twist, c.drive})
                b.number(v);
            for (double v : c.first)
                b.number(v);
            for (double v : c.second)
                b.number(v);
        }
        b.integer(p.twists.size());
        for (const auto &t : p.twists) {
            b.number(t.begin);
            b.number(t.end);
            b.number(t.angle);
        }
        b.number(p.geometricDuration);
        b.integer(p.geometry.size());
        for (const auto &k : p.geometry) {
            b.number(k.s);
            for (const auto &values : {k.value, k.first, k.second, k.third})
                for (double x : values)
                    b.number(x);
        }
    }
    b.integer(checksum(b.data));
    poll(cancel);
    auto temporary = path;
    temporary += ".writing-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() {
            std::error_code e;
            std::filesystem::remove(path, e);
        }
    } cleanup{temporary};
    std::ofstream f(temporary, std::ios::binary | std::ios::trunc);
    if (!f)
        throw std::runtime_error("Cannot create saved design");
    f.write(reinterpret_cast<const char *>(b.data.data()), std::streamsize(b.data.size()));
    f.flush();
    f.close();
    if (!f)
        throw std::runtime_error("Saved design write failed");
    poll(cancel);
    // Keep the active save in place until the replacement is fully written.
    // A failed replace or a cancellation cannot strand the active save.
    std::error_code error;
    if (std::filesystem::exists(path)) {
        auto backup = path;
        backup += ".previous";
        std::filesystem::copy_file(path, backup, std::filesystem::copy_options::overwrite_existing, error);
        if (error)
            throw std::runtime_error("Could not preserve previous save");
    }
    poll(cancel);
#ifdef _WIN32
    if (!MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Could not atomically replace saved design");
#else
    std::filesystem::rename(temporary, path, error);
    if (error)
        throw std::runtime_error("Could not atomically replace saved design");
#endif
}
Design loadDesign(const std::filesystem::path &path, const Cancel &cancel) {
    poll(cancel);
    const auto size = std::filesystem::file_size(path);
    if (size < 32 || size > 8 * 1024 * 1024)
        throw std::runtime_error("Invalid saved design size");
    Bytes b;
    b.data.resize(std::size_t(size));
    std::ifstream f(path, std::ios::binary);
    if (!f.read(reinterpret_cast<char *>(b.data.data()), std::streamsize(size)))
        throw std::runtime_error("Cannot read saved design");
    b.pos = b.data.size() - 8;
    const auto expected = b.integer();
    if (checksum(std::span(b.data).first(b.data.size() - 8)) != expected)
        throw std::runtime_error("Saved design integrity check failed");
    b.pos = 0;
    if (b.integer() != magic || b.integer() != 2)
        throw std::runtime_error("Unsupported saved design version");
    Design d;
    const auto seed = b.integer();
    if (seed > 4294967295ULL)
        throw std::runtime_error("Invalid saved seed");
    d.recipe.seed = static_cast<unsigned>(seed);
    d.recipe.style = b.text();
    for (double *v : {&d.recipe.plateau, &d.recipe.openingHeight, &d.recipe.camelbackHeight,
                      &d.recipe.loopHeight, &d.recipe.immelmannHeight, &d.recipe.topSpeedKph,
                      &d.recipe.activeSeconds, &d.recipe.terminalSeconds})
        *v = b.number();
    validateRecipe(d.recipe);
    const auto count = b.integer();
    if (count < 1 || count > 100)
        throw std::runtime_error("Invalid saved source count");
    std::vector<Program> source;
    for (std::size_t i = 0; i < count; ++i) {
        poll(cancel);
        Program p;
        p.id = b.text();
        p.label = b.text();
        const auto role = b.integer();
        if (role > static_cast<std::uint64_t>(Role::Terminal))
            throw std::runtime_error("Invalid saved role");
        p.role = static_cast<Role>(role);
        p.initial.p = b.vec();
        p.initial.t = b.vec();
        p.initial.u = b.vec();
        p.initial.v = b.number();
        p.initial.s = b.number();
        p.initial.lossWork = b.number();
        p.initial.driveWork = b.number();
        p.rolling = b.number();
        p.drag = b.number();
        const auto bank = b.integer();
        if (bank > 1)
            throw std::runtime_error("Invalid bank mode");
        p.gravityBank = bank != 0;
        const auto controls = b.integer();
        if (controls > 1000)
            throw std::runtime_error("Invalid saved control count");
        for (std::size_t k = 0; k < controls; ++k) {
            Control c;
            c.time = b.number();
            c.normal = b.number();
            c.lateral = b.number();
            c.twist = b.number();
            c.drive = b.number();
            for (auto &v : c.first)
                v = b.number();
            for (auto &v : c.second)
                v = b.number();
            p.controls.push_back(c);
        }
        const auto twists = b.integer();
        if (twists > 100)
            throw std::runtime_error("Invalid saved twist count");
        for (std::size_t k = 0; k < twists; ++k) {
            const double begin = b.number(), end = b.number(), angle = b.number();
            p.twists.push_back({begin, end, angle});
        }
        p.geometricDuration = b.number();
        const auto geometry = b.integer();
        if (geometry > 2000 || (geometry && controls) || (!geometry && controls < 2) || geometry == 1)
            throw std::runtime_error("Invalid saved source representation");
        for (std::size_t k = 0; k < geometry; ++k) {
            poll(cancel);
            GeometryControl g;
            g.s = b.number();
            for (auto *values : {&g.value, &g.first, &g.second, &g.third})
                for (auto &x : *values)
                    x = b.number();
            p.geometry.push_back(g);
        }
        if (p.rolling < 0 || p.rolling > 1 || p.drag < 0 || p.drag > .01 || p.duration() <= 0 ||
            p.duration() > 200)
            throw std::runtime_error("Invalid saved physical bounds");
        validateProgram(p);
        source.push_back(std::move(p));
    }
    if (b.pos != b.data.size() - 8)
        throw std::runtime_error("Unexpected saved trailing data");
    // A checksum never accepts geometry. Rebuild from source and independently
    // replay before the caller runs the complete ride acceptance pipeline.
    d.track = compile(std::move(source), .02, cancel);
    const auto proof = assessReplay(d.track, .01, cancel);
    if (proof.position > .005 || proof.forward > 1e-4 || proof.up > 1e-4 || proof.portPosition > .001 ||
        proof.portTangent > 1e-5 || proof.portUp > 1e-5 || proof.portCurvature > 1e-5 ||
        proof.portThird > 1e-5 || proof.portUpThird > 1e-4)
        throw std::runtime_error("Saved source replay/continuity validation failed");
    const auto operation = simulate(d.track, {}, 1. / 240, cancel, false);
    if (!operation.completed)
        throw std::runtime_error("Saved operation reference cannot finish");
    for (const auto &q : operation.playback)
        d.track.operationSpeed.push_back({q.s, q.speed});
    d.baseline = std::make_shared<Simulation>(simulate(d.track, {}, 1. / 960, cancel));
    const auto clearance = assessClearance(d.track, d.recipe.plateau, .5, cancel);
    if (!d.baseline->failures.empty() || !d.track.closed ||
        std::abs(d.baseline->active - d.recipe.activeSeconds) > .05 || d.baseline->terminal < 5 ||
        d.baseline->terminal > 10 || clearance.terrainHits || clearance.trackHits)
        throw std::runtime_error("Saved design failed fresh nominal physical/clearance validation");
    d.notes.push_back("Fresh nominal validation passed; operating scenarios and convergence remain "
                      "activation requirements");
    return d;
}
} // namespace coaster
