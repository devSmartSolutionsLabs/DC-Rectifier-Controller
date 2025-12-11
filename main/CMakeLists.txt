idf_component_register(
    SRCS "main.cpp" "ads1115.cpp" "mcp23017.cpp" "INA226.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer
)