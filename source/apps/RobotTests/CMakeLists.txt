project(RobotTests)

file(GLOB sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        )

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME}
        PUBLIC lenny::robot
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        )

set(LENNYROBOT_TESTAPP_FOLDER "\"${CMAKE_CURRENT_SOURCE_DIR}\"" CACHE STRING "")

target_compile_definitions(${PROJECT_NAME}
        PUBLIC LENNY_PROJECT_FOLDER=${LENNY_PROJECT_FOLDER}
        PUBLIC LENNYROBOT_TESTAPP_FOLDER=${LENNYROBOT_TESTAPP_FOLDER}
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
