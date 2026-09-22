#include "coaster/persistence.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
using namespace coaster;
namespace {
void require(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
std::vector<char> bytes(const std::filesystem::path &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Missing test save");
    return {std::istreambuf_iterator<char>(f), {}};
}
bool hasTemporary(const std::filesystem::path &folder) {
    for (const auto &file : std::filesystem::directory_iterator(folder))
        if (file.path().filename().string().find(".writing-") != std::string::npos)
            return true;
    return false;
}
} // namespace
int main() {
    try {
        const auto folder =
            std::filesystem::temp_directory_path() /
            ("vibe-save-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(std::filesystem::create_directory(folder), "Test directory must be new");
        const auto path = folder / "ride.vcd", unsafePath = folder / "unsafe.vcd",
                   brokenPath = folder / "broken.vcd";
        auto design = generate(Recipe{});
        require(design.baseline->failures.empty(), "Fixture must pass nominal physics");
        saveDesign(design, path);
        const auto original = bytes(path);
        auto loaded = loadDesign(path);
        require(loaded.baseline && loaded.baseline->assessed && loaded.baseline->failures.empty(),
                "Fresh loaded assessment missing");
        require(std::abs(loaded.track.length - design.track.length) < 1e-7,
                "Mixed source round trip changed geometry");
        require(loaded.track.source.size() == design.track.source.size(), "Mixed source count changed");
        require(std::filesystem::file_size(path) < 1024 * 1024, "Source save unexpectedly large");
        bool cancelled = false;
        try {
            saveDesign(design, path, [&] { return hasTemporary(folder); });
        } catch (const Cancelled &) {
            cancelled = true;
        }
        require(cancelled, "Late save cancellation was not exercised");
        require(bytes(path) == original, "Cancelled save changed active bytes");
        require(!hasTemporary(folder), "Cancelled save left temporary data");
        // This file has a valid freshly generated checksum and valid relative
        // authorship, but the whole ride is buried in the fixed site terrain.
        // It must still fail independent physical activation validation.
        auto unsafe = design;
        for (auto &program : unsafe.track.source)
            program.initial.p.z -= 20;
        saveDesign(unsafe, unsafePath);
        bool rejected = false;
        try {
            loadDesign(unsafePath);
        } catch (const std::runtime_error &e) {
            rejected = std::string(e.what()).find("physical/clearance") != std::string::npos;
        }
        require(rejected, "A valid checksum accepted buried track");
        auto corrupted = original;
        corrupted[corrupted.size() / 2] ^= 1;
        {
            std::ofstream f(brokenPath, std::ios::binary);
            f.write(corrupted.data(), std::streamsize(corrupted.size()));
        }
        rejected = false;
        try {
            loadDesign(brokenPath);
        } catch (const std::runtime_error &e) {
            rejected = std::string(e.what()).find("integrity") != std::string::npos;
        }
        require(rejected, "Byte corruption was accepted");
        // Only the explicitly created files and now-empty directory are removed.
        std::filesystem::remove(path);
        std::filesystem::remove(unsafePath);
        std::filesystem::remove(brokenPath);
        std::filesystem::remove(folder);
        std::cout << "PASS mixed-source save/load, fresh physics, late cancellation and rechecksummed "
                     "invalid geometry\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL " << e.what() << '\n';
        return 1;
    }
}
