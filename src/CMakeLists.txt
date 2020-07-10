add_library(robust_vslam SHARED      
        frame.cpp                
        sensors.cpp
        config.cpp
        feature.cpp
        tracking.cpp
        parameter.cpp
        System.cpp
        ORBextractor.cpp)

target_link_libraries(robust_vslam
        ${THIRD_PARTY_LIBS})
