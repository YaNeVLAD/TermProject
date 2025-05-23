include(FetchContent)

# SFML
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML
        GIT_TAG 3.0.1
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(SFML)

# CGAL
