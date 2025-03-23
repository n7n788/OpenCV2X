add_subdirectory(artery)
add_subdirectory(gemv2)
add_subdirectory(highway-police)
if(WITH_STORYBOARD)
    add_subdirectory(storyboard)
endif()
if(WITH_TESTBED)
    add_subdirectory(testbed)
endif()
if(WITH_TRANSFUSION)
    add_subdirectory(transfusion)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mt-its2017/omnetpp.ini)
    add_opp_run(mtits2017 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mt-its2017)
    add_opp_test(mtits2017 SUFFIX test SIMTIME_LIMIT 30s)
endif()

if(TARGET lte)
    add_library(artery-lte INTERFACE)
    target_link_libraries(artery-lte INTERFACE artery lte)
    add_opp_run(simulte
        DEPENDENCY artery-lte
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/simulte)
    add_opp_test(simulte SUFFIX cellular CONFIG VoIP-UpLink SIMTIME_LIMIT 10s)
    add_opp_test(simulte SUFFIX d2d CONFIG AlertMulticast SIMTIME_LIMIT 25s)
    add_opp_run(simulte-cars
        DEPENDENCY artery-lte
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cars)
    add_opp_run(simulte-Mode4
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Mode4)
    add_opp_run(simulte-Example
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/example)
    add_opp_run(simulte-validation
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Mode4-Validation)
    add_opp_run(personal-experiments
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/personal-experiments)
    add_opp_run(personal-Validation
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/personal-Validation)
    add_opp_run(personal-non-periodic
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/personal-non-periodic)
    add_opp_test(simulte-cars SUFFIX d2d CONFIG D2D_MULTICAST SIMTIME_LIMIT 70s)
    add_opp_run(simulte-myscena
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/myscena)
    add_opp_run(simulte-myscena2
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/myscena2)
    add_opp_run(simulte-manhattan
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/manhattan)
    add_opp_run(simulte-highway-merge
            DEPENDENCY artery-lte
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/highway-merge)
endif()

if(TARGET lte AND WITH_STORYBOARD)
    add_subdirectory(lte-blackice)
endif()

add_opp_run(car2car-grid WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/car2car-grid)
add_opp_test(car2car-grid SUFFIX cam_bsp CONFIG cam_bsp RUN 0 SIMTIME_LIMIT 30s)
