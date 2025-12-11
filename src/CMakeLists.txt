# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.
set(IDF_TARGET esp32s3)
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS
        "main.cpp"
        "ads1115.cpp"
        "variables.cpp"
        "INA226.cpp"
        "SerialConsole.cpp"
        "webOTAServer.cpp"
        "WiFiHelper.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_http_server
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        app_update
)