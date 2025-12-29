idf_component_register(
    SRCS 
        "main.cpp" 
        "ads1115.cpp" 
        "mcp23017.cpp" 
        "ina226.cpp"
        "WifiManager.cpp"
        "GitHubClient.cpp"
        "PortalWeb.cpp"
        "LoggerFS.cpp"
        "CommandManager.cpp"
    INCLUDE_DIRS "."
    EMBED_TXTFILES 
        "github_root_ca.pem" 
        "index.html"
    REQUIRES 
        esp_adc
        driver 
        freertos 
        esp_timer 
        esp_wifi            
        esp_event           
        nvs_flash           
        esp_http_server     
        esp_http_client     
        esp_https_ota       
        app_update          
        json                # Este es cJSON
        esp-tls             # <--- ESTE habilita esp_crt_bundle.h
        mbedtls             # <--- Requerido para el motor de cifrado
        spiffs 
        esp_partition
        fatfs       # <--- AGREGAR ESTE
        sdmmc       # <--- AGREGAR ESTE
)