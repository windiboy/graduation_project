set(Demo1 controller)
set(Demo2 arm_follow_demo)
set(Demo3 pick_item_demo)
set(Demo4 pick_item_demo_v2)
set(Demo5 open_door_demo)
set(Demo6 open_door_demo_v2)
set(Function1 end_effector_rpy_publisher)

add_executable(${Demo1} src/${Demo1}.cpp)
target_link_libraries(${Demo1} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo1} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Demo2} src/${Demo2}.cpp)
target_link_libraries(${Demo2} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo2} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Demo3} src/${Demo3}.cpp)
target_link_libraries(${Demo3} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo3} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Demo4} src/${Demo4}.cpp)
target_link_libraries(${Demo4} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo4} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Demo5} src/${Demo5}.cpp)
target_link_libraries(${Demo5} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo5} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Demo6} src/${Demo6}.cpp)
target_link_libraries(${Demo6} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo6} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${Function1} src/${Function1}.cpp)
target_link_libraries(${Function1} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Function1} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
