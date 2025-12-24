#pragma once
#include <string>
#include <vector>
#include "esp_err.h"

struct ReleaseInfo {
    std::string tag;
    std::string bin_url;
};

class GitHubClient {
public:
    static std::vector<ReleaseInfo> get_releases(const char* repo);
    static void start_ota_from_url(const char* url);

private:
    static void ota_task(void* pvParameter);
};