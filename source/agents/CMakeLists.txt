project(agents)

file(GLOB sources
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_library(${PROJECT_NAME} ${sources})
add_library(lenny::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        PUBLIC lenny::rapt
        PUBLIC lenny::kineloco
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

target_compile_definitions(${PROJECT_NAME}
        PUBLIC LENNY_PROJECT_FOLDER=${LENNY_PROJECT_FOLDER}
        PUBLIC LENNY_ROBOT_FOLDER=${LENNY_ROBOT_FOLDER}
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)