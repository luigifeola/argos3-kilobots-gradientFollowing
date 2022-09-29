include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(ALF_gradientFollowing_loop_function
  MODULE 
  gradientFollowing_ALF.h
  gradientFollowing_ALF.cpp
)

target_link_libraries(ALF_gradientFollowing_loop_function
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)
