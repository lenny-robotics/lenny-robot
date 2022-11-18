#COMMENT: Heavily inspired by: https://github.com/ORB-HD/URDF_Parser

project(urdf)

file(GLOB sources
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_library(${PROJECT_NAME} ${sources})
add_library(lenny::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        PUBLIC tinyxml2
        PUBLIC lenny::tools
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
