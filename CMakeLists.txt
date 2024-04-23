# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS fc)
set(EXCLUDE_COMPONENTS  esp_https_server tcp_transport wifi_provisioning http_parser esp_wifi esp-tls esp_https_server esp_https_ota esp_http_client )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_FC)