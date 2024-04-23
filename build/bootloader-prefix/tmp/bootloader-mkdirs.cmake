# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "F:/softwares/Espressif/esp-idf-v5.2.1/components/bootloader/subproject"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/tmp"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/src/bootloader-stamp"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/src"
  "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/Bijesh/ESP_Workspace/ESP32_FC/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
