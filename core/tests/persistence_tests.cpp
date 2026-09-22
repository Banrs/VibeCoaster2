#include "coaster/persistence.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <atomic>
#include <cstdint>
#include <thread>
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
void writeChecksummed(const std::filesystem::path &path, std::vector<char> data) {
    std::uint64_t sum = 14695981039346656037ULL;
    for (std::size_t i = 0; i < data.size() - 8; ++i) {
        sum ^= static_cast<unsigned char>(data[i]);
        sum *= 1099511628211ULL;
    }
    for (unsigned i = 0; i < 8; ++i)
        data[data.size() - 8 + i] = static_cast<char>((sum >> (8 * i)) & 255);
    std::ofstream f(path, std::ios::binary);
    f.write(data.data(), std::streamsize(data.size()));
    require(bool(f), "Could not write integrity-valid version fixture");
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
        require(loaded.validation && loaded.baseline && loaded.baseline->assessed &&
                    loaded.baseline->failures.empty(),
                "Fresh loaded assessment missing");
        require(std::abs(loaded.track.length - design.track.length) < 1e-7,
                "Mixed source round trip changed geometry");
        require(loaded.track.source.size() == design.track.source.size(), "Mixed source count changed");
        require(std::filesystem::file_size(path) < 1024 * 1024, "Source save unexpectedly large");
        // Changing a recipe target without changing the authored source is
        // an integrity-valid semantic mismatch, not an edited ride.
        const auto mismatchPath = folder / "metadata-mismatch.vcd";
        auto mismatch = design;
        mismatch.recipe.openingHeight += 5;
        saveDesign(mismatch, mismatchPath);
        bool metadataRejected = false;
        try {
            loadDesign(mismatchPath);
        } catch (const std::runtime_error &e) {
            metadataRejected = std::string(e.what()).find("authoring") != std::string::npos;
        }
        require(metadataRejected, "A checksum accepted a height target that was never authored");
        mismatch = design;
        mismatch.recipe.topSpeedKph += 1;
        saveDesign(mismatch, mismatchPath);
        metadataRejected = false;
        try {
            loadDesign(mismatchPath);
        } catch (const std::runtime_error &e) {
            metadataRejected = std::string(e.what()).find("authoring") != std::string::npos;
        }
        require(metadataRejected, "A checksum accepted a top speed that was never authored");
        mismatch = design;
        auto wave = std::find_if(mismatch.track.source.begin(), mismatch.track.source.end(),
                                 [](const auto &p) { return p.id == "wave"; });
        wave->role = Role::Journey;
        saveDesign(mismatch, mismatchPath);
        metadataRejected = false;
        try {
            loadDesign(mismatchPath);
        } catch (const std::runtime_error &e) {
            metadataRejected = std::string(e.what()).find("authoring") != std::string::npos;
        }
        require(metadataRejected, "A checksum accepted a missing required ride role");
        std::filesystem::remove(mismatchPath);
        std::filesystem::remove(folder / "metadata-mismatch.vcd.previous");
        auto cancelledValidation = loaded;
        int validationPolls = 0;
        bool validationCancelled = false;
        try {
            validateRide(cancelledValidation, [&] { return ++validationPolls > 300; });
        } catch (const Cancelled &) {
            validationCancelled = true;
        }
        require(validationCancelled && !cancelledValidation.validation,
                "Cancelled validation retained stale acceptance evidence");
        cancelledValidation = loaded;
        const auto caller = std::this_thread::get_id();
        std::atomic<bool> workerObserved{false};
        validationCancelled = false;
        try {
            validateRide(cancelledValidation, [&] {
                if (std::this_thread::get_id() == caller)
                    return false;
                workerObserved = true;
                return true;
            });
        } catch (const Cancelled &) {
            validationCancelled = true;
        }
        require(validationCancelled && workerObserved && !cancelledValidation.validation,
                "Parallel validation did not propagate cancellation or retained stale evidence");
        for (const auto [offset, message] :
             std::array<std::pair<std::size_t, const char *>, 2>{{{8, "version"}, {16, "site revision"}}}) {
            auto old = original;
            old[offset] = 1;
            writeChecksummed(brokenPath, std::move(old));
            bool versionRejected = false;
            try {
                loadDesign(brokenPath);
            } catch (const std::runtime_error &e) {
                versionRejected = std::string(e.what()).find(message) != std::string::npos;
            }
            require(versionRejected, "A valid checksum reinterpreted an unsupported save/site version");
        }
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
