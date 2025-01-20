# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/meho.kitovnica/ESP-IDF/esp-idf-v5.3.1/components/bootloader/subproject"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/tmp"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/src"
  "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/meho.kitovnica/Desktop/Projekte/IoTController/Software/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
