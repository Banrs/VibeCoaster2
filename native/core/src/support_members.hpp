#pragma once
#include "coaster/coaster.hpp"
#include <span>

namespace coaster {
// Old saves store a column and optional arm instead of an explicit member list.
class SupportMembers {
    std::array<SupportMember, 2> fallback{};
    std::span<const SupportMember> view;
public:
    explicit SupportMembers(const Support& support) : view(support.members) {
        if (!view.empty()) return;
        fallback[0] = {support.base, support.top, supportRadius, supportRadius, SupportMemberKind::Steel, false};
        fallback[1] = {support.top, support.attachment, supportRadius, supportRadius, SupportMemberKind::Steel, true};
        view = {fallback.data(), support.hasAttachment ? size_t(2) : size_t(1)};
    }
    SupportMembers(const SupportMembers&) = delete;
    SupportMembers& operator=(const SupportMembers&) = delete;
    auto begin() const { return view.begin(); }
    auto end() const { return view.end(); }
};
}
