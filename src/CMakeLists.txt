add_library(lzb_vio SHARED      
        frame.cpp                
        sensors.cpp
        config.cpp
        feature.cpp
        tracking.cpp
        parameter.cpp
        System.cpp
        ORBextractor.cpp)

target_link_libraries(lzb_vio
        ${THIRD_PARTY_LIBS})
