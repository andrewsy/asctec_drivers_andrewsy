FILE(REMOVE_RECURSE
  "../src/flyer_controller/msg"
  "../src/flyer_controller/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/flyer_controller.dir/nodes/controller.o"
  "CMakeFiles/flyer_controller.dir/nodes/teleop_flyer.o"
  "CMakeFiles/flyer_controller.dir/src/control_mode.o"
  "CMakeFiles/flyer_controller.dir/src/attitude_mode.o"
  "CMakeFiles/flyer_controller.dir/nodes/control_mode_idle.o"
  "CMakeFiles/flyer_controller.dir/nodes/control_mode_attitude.o"
  "CMakeFiles/flyer_controller.dir/nodes/control_mode_hover.o"
  "CMakeFiles/flyer_controller.dir/nodes/control_mode_autosequence.o"
  "../lib/libflyer_controller.pdb"
  "../lib/libflyer_controller.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/flyer_controller.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
