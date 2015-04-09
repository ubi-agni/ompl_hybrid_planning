
option(OMPL_CONTRIB_TASK_LEVEL_CONTRIB "Enable task level planning contrib" ON)

if(OMPL_CONTRIB_TASK_LEVEL_CONTRIB)
  file(GLOB_RECURSE OMPL_TASK_LEVEL_CONTRIB_SOURCE_CODE src/*.cpp)
  set(OMPL_SOURCE_CODE
      ${OMPL_SOURCE_CODE} ${OMPL_TASK_LEVEL_CONTRIB_SOURCE_CODE}
      PARENT_SCOPE)

  add_ompl_test(test_task_level_planning_control tests/test_control.cpp)
  add_ompl_test(test_task_level_planning_geometric tests/test_geometric.cpp)
endif()
