#-------CMakeList for Ubuntu 12.04 with OpenRAVE 0.8-----------#

cmake_minimum_required (VERSION 2.6.0)
PROJECT( LearningLocomotion )
set(CMAKE_VERBOSE_MAKEFILE true)

find_package(OpenRAVE 0.8 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)


set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Make sure the openmr plugin is installed!
# Get the openrave share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)

find_library(OPENMR openmr ${OpenRAVE_SHARE_DIR}/plugins)
message(STATUS "Looking for OpenMR... ${OPENMR}")

if (NOT OPENMR)
  message(SEND_ERROR "OPENMR not found")
endif()


# Include and link directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE("OpenCV_LIBS : ${OpenCV_LIBS}")

ADD_DEFINITIONS(-std=c++0x)
ADD_DEFINITIONS(-pthread)

### include dirs
INCLUDE_DIRECTORIES(
Robot SimulationOpenRave Y1ModularRobot Controller ServoFeedback ExtKalmanFilter_SinusModel
GraphFile VisualTracker SerialCommunication Learn LearningAlgorithm QLearning ApproximateQLearning
)

add_executable(Learn
  Learn/Learn.cpp
  Robot/Robot.cpp
  SimulationOpenRave/SimulationOpenRave.cpp
  Y1ModularRobot/Y1ModularRobot.cpp
  LearningAlgorithm/LearningAlgorithm.cpp
  QLearning/QLearning.cpp
  ApproximateQLearning/ApproximateQLearning.cpp
  ServoFeedback/ServoFeedback.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
  GraphFile/GraphFile.cpp
  #OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
  SerialCommunication/SerialCommunication.c
)
### Learn executable
set_target_properties(Learn PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Learn PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Learn ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} -std=c++0x -pthread -larmadillo -g)

