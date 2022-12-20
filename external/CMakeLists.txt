# tinyxml2
if (NOT TARGET tinyxml2)
    add_subdirectory(tinyxml2)
endif ()

# lenny-collision
fetch_lenny_repository(collision softification-weights)