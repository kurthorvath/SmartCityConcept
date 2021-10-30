# CMakeList.txt: CMake-Projekt für "capture". Schließen Sie die Quelle ein, und definieren Sie
# projektspezifische Logik hier.
#
cmake_minimum_required (VERSION 3.8)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Fügen Sie der ausführbaren Datei dieses Projekts eine Quelle hinzu.
add_executable (capture "capture.cpp" "capture.h")

target_link_libraries( capture ${OpenCV_LIBS} )
# TODO: Fügen Sie bei Bedarf Tests hinzu, und installieren Sie Ziele.
