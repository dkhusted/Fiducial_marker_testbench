cmake_minimum_required(VERSION 3.17.0)
project(testBench_)


IF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#Use these for building the project manually. 
#option(BUILD_STAG "Toggles building of STag" OFF)
#option(BUILD_PITAG "Toggles building of Pi-Tag" OFF)
#option(BUILD_APRILTAG "Toggles building of AprilTag" OFF)
#option(BUILD_CCTAG "Toggles building of CCTag" OFF)
#option(BUILD_ARUCO "Toggles building of ArUco" OFF)
#option(BUILD_MAIN "Toggles building of main" OFF)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -w  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/app)

## Find required packages
find_package(AprilTag REQUIRED) 
find_package(OpenCV REQUIRED core videoio imgproc imgcodecs highgui)
find_package(TinyXML REQUIRED)
find_package(CCTag CONFIG REQUIRED)
find_package(Mqtt REQUIRED)
find_package(DevIL COMPONENTS IL ILU) # yields IL_FOUND, IL_LIBRARIES, IL_INCLUDE_DIR
find_package(Boost 1.66.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS} QUIET)
find_package(cJSON REQUIRED)

#### Main configuration ###########################################################

#if(BUILD_MAIN)
  add_library(mqtt ${CMAKE_SOURCE_DIR}/include/mqtt.cpp)

  target_link_libraries(mqtt PRIVATE 
    ${Mqtt_LIBRARIES}
    ${CJSON_LIBRARIES}
    )

  target_include_directories(mqtt PRIVATE
    ${Mqtt_INCLUDE_DIR}
    ${CJSON_INCLUDE_DIR}
    )

  include_directories(
    common/include
    ${AprilTag_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})

  add_subdirectory(
  models/stag
  models/piTag)


  add_executable(main 
    ${CMAKE_SOURCE_DIR}/src/main.cpp 
    ${CMAKE_SOURCE_DIR}/include/arUco_model.cpp
    ${CMAKE_SOURCE_DIR}/include/aprilTag_model.cpp
    ${CMAKE_SOURCE_DIR}/include/stag_model.cpp
    ${CMAKE_SOURCE_DIR}/include/piTag_model.cpp
    ${CMAKE_SOURCE_DIR}/include/ccTag_model.cpp)
  
  target_include_directories(main PRIVATE
    mqtt
    ${Mqtt_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${AprilTag_INCLUDE_DIR}
    )

  target_link_libraries(main PRIVATE
    pitag
    Stag
    mqtt
    ${Mqtt_LIBRARIES}
    ${OpenCV_LIBS}
    ${AprilTag_LIBRARIES}
    ${TinyXML_LIBRARIES}
    CCTag::CCTag
    TBB::tbb
    Boost::filesystem Boost::program_options Boost::timer
    )


##### Camera Calibration ###########################

add_executable(camera_calibration
${CMAKE_SOURCE_DIR}/include/camera_calibration.cpp 
)

target_include_directories(camera_calibration PRIVATE
${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_calibration PRIVATE
${OpenCV_LIBS}
)

add_executable(imagelist_creator
${CMAKE_SOURCE_DIR}/include/imagelist_creator.cpp 
)

target_include_directories(imagelist_creator PRIVATE
${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imagelist_creator PRIVATE
${OpenCV_LIBS}
)


##### STag configuration ###########################

#if(BUILD_STAG)
  #add_subdirectory(models/stag)

  add_executable(${PROJECT_NAME}stag ${CMAKE_SOURCE_DIR}/samples/stag-test.cpp)

  target_include_directories(${PROJECT_NAME}stag PRIVATE ${CMAKE_SOURCE_DIR}/modules/stag/src/ ${CMAKE_SOURCE_DIR}/modules/stag/src/ED)

  target_link_libraries(${PROJECT_NAME}stag 
    Stag
    ${OpenCV_LIBS}
    )

#endif()

##### Pi-Tag configuration ###########################

#if(BUILD_PITAG)


  #include_directories(
  #  common/include
  #  ${OpenCV_INCLUDE_DIRS}
  #  ${TinyXML_INCLUDE_DIRS}
  #  )
    
  #add_subdirectory(models/piTag)

  add_executable(${PROJECT_NAME}piTag ${CMAKE_SOURCE_DIR}/samples/pitag-test.cpp)

  target_include_directories(${PROJECT_NAME}piTag PRIVATE models/piTag)

  target_link_libraries(${PROJECT_NAME}piTag
  pitag
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
  )

#endif()
##### aprilTag configuration ######################################################

#if(BUILD_APRILTAG)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${AprilTag_INCLUDE_DIRS}
    )
    
  add_executable(${PROJECT_NAME}aprilTag ${CMAKE_SOURCE_DIR}/samples/aprilTag-test.cpp)

  target_include_directories(${PROJECT_NAME}aprilTag PRIVATE ${AprilTag_INCLUDE_DIR})

  target_link_libraries(${PROJECT_NAME}aprilTag
    ${OpenCV_LIBS}
    ${AprilTag_LIBRARIES}
    )

#endif()

##### CCTag configuration #######################################################

#if(BUILD_CCTAG)
  add_executable(${PROJECT_NAME}cctag ${CMAKE_SOURCE_DIR}/samples/ccTag-test.cpp ${CMAKE_SOURCE_DIR}/samples/CmdLine.cpp)



  target_include_directories(${PROJECT_NAME}cctag PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    )
  target_link_libraries(${PROJECT_NAME}cctag PUBLIC
    CCTag::CCTag
    TBB::tbb
    ${OpenCV_LIBS}
    Boost::filesystem Boost::program_options Boost::timer
    )

    if(IL_FOUND OR DevIL_FOUND)
    message(STATUS "DevIL found")
    target_compile_options(${PROJECT_NAME}cctag PRIVATE -DUSE_DEVIL)
    target_include_directories(${PROJECT_NAME}cctag PUBLIC
      ${IL_INCLUDE_DIR}
      )

    target_link_libraries(${PROJECT_NAME}cctag PUBLIC
      ${IL_LIBRARIES}
      ${ILU_LIBRARIES}
      )
  else()
    message(STATUS "DevIL not found")
  endif()

#endif()


##### arUco Tag configuration ###########################

#if(BUILD_ARUCO)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    )
    
  add_executable(${PROJECT_NAME}arUcoTag ${CMAKE_SOURCE_DIR}/samples/aruco-test.cpp)

  target_include_directories(${PROJECT_NAME}arUcoTag PRIVATE 
  ${OpenCV_INCLUDE_DIRS})

  target_link_libraries(${PROJECT_NAME}arUcoTag
    ${OpenCV_LIBS}
    )

#endif()