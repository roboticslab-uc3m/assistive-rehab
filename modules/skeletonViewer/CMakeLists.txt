# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

find_package(VTK 8.1.0 QUIET)
if(VTK_FOUND)
  project(skeletonViewer)

  set(doc_files ${PROJECT_NAME}.xml)

  source_group("DOC Files" FILES ${doc_files})

  include(${VTK_USE_FILE})
  add_executable(${PROJECT_NAME} main.cpp ${doc_files})
  target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
  target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${VTK_LIBRARIES} AssistiveRehab)
  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()

