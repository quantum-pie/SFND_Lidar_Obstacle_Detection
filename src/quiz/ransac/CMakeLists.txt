cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall \
		-Werror=delete-non-virtual-dtor \
                -Werror=return-type \
                -Werror=unused-result \
                -Werror=format-security \
                -Werror=deprecated-declarations \
                -Werror=delete-incomplete \
		-Werror=int-in-bool-context \
                -Werror=unused-but-set-variable \
		-Werror=old-style-cast")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES})




