# fetch gui repository
fetch_lenny_repository(gui-opengl master)

# Add subdirectories
add_subdirectory(AgentsApp)
add_subdirectory(ControlApp)
add_subdirectory(KineLocoApp)
add_subdirectory(PrimitiveGeneratorApp)
add_subdirectory(RobotApp)
add_subdirectory(RobotTests)
add_subdirectory(URDFTest)
