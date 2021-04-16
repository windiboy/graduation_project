set(Demo1 controller)
set(Demo2 arm_follow_demo)
set(Demo3 pick_item_demo)
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

add_executable(${Function1} src/${Function1}.cpp)
target_link_libraries(${Function1} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Function1} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
