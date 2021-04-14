include_directories(include)
add_library(SCServo
  src/SCS.cpp
  src/SCSCL.cpp
  src/SCSerial.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
  include/INST.h
  include/SCS.h
  include/SCSCL.h
  include/SCSerial.h
  include/SMSBL.h
  include/SMSCL.h
  include/SCServo.h
)
add_executable(gripper_control src/gripper_control.cpp)
target_link_libraries(gripper_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} SCServo)
install(TARGETS gripper_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
