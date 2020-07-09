add_library(robust_vslam SHARED      
        frame.cpp                
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        System.cpp
        ORBextractor.cpp)

target_link_libraries(robust_vslam
        ${THIRD_PARTY_LIBS})
