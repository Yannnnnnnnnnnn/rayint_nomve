
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(raycast obj_loader.h raycast.cpp)

target_link_libraries(raycast ${OpenCV_LIBS})
