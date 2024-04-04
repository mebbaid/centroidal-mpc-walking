# This software may be modified and distributed under the terms of the
# BSD 3-Clause License
# Authors: Mohamed Elobaid

add_centroidal_mpc_walking_application(
  NAME            simple-planner
  SOURCES         src/SimplePlanner.cpp src/main.cpp
  HEADERS         include/SimplePlanner/SimplePlanner.h
  LINK_LIBRARIES  ${SDL_LIBRARY} YARP::YARP_os YARP::YARP_init YARP::YARP_sig
                  BipedalLocomotion::System
                  BipedalLocomotion::ParametersHandlerYarpImplementation
                  BipedalLocomotion::SystemYarpImplementation
                  BipedalLocomotion::Planners
                  BipedalLocomotion::Contacts
                  iDynTree::idyntree-high-level iDynTree::idyntree-core
                  UnicyclePlanner)