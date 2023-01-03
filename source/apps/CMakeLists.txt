# fetch gui repository
fetch_lenny_repository(gui-opengl master)

# cache folder path
set(LENNY_ROBOT_APP_FOLDER "\"${CMAKE_CURRENT_LIST_DIR}\"" CACHE STRING "")

# Add subdirectories
add_subdirectory(ControlApp)
add_subdirectory(RAPTApp)
add_subdirectory(RobotApp)