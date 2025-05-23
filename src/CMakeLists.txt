find_package(Boost 1.88.0 REQUIRED)

add_executable(Term ../main.cpp
        Window/Window.cpp
        Window/Window.h
        Utility/Shapes.hpp
        Utility/Color/Color.cpp
        Utility/Color/Color.h
        Algorithm/Math/Math.hpp
        Algorithm/VVC/VVC.cpp
        Algorithm/VVC/VVC.h
        Algorithm/HaloGenerator/HaloGenerator.cpp
        Algorithm/HaloGenerator/HaloGenerator.h
        Algorithm/PathFinder/PathFinder.cpp
        Algorithm/PathFinder/PathFinder.h
        Algorithm/Graph/Graph.cpp
        Algorithm/Graph/Graph.h
        Algorithm/Voronoi/Voronoi.h
        Algorithm/Voronoi/Voronoi.cpp
        Algorithm/Structures/BoostCompat.h
        Utility/Logger/Logger.h
        Utility/Logger/Logger.cpp
        Utility/Logger/DefaultAppenders.h
        Utility/Logger/ILogAppender.h
        Window/Keyboard.hpp)
target_compile_features(Term PRIVATE cxx_std_20)
target_link_libraries(Term PRIVATE SFML::Graphics)
target_link_libraries(Term PRIVATE Boost::boost)

target_compile_definitions(Term PRIVATE $<$<CONFIG:Debug>:_DEBUG>)