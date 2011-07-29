FILE(REMOVE_RECURSE
  "../src/flyer_controller/msg"
  "../src/flyer_controller/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/flyer_controller/control_mode_autosequence_info.h"
  "../msg_gen/cpp/include/flyer_controller/control_mode_output.h"
  "../msg_gen/cpp/include/flyer_controller/Autosequence.h"
  "../msg_gen/cpp/include/flyer_controller/control_mode_cmd.h"
  "../msg_gen/cpp/include/flyer_controller/controller_status.h"
  "../msg_gen/cpp/include/flyer_controller/control_mode_status.h"
  "../msg_gen/cpp/include/flyer_controller/controller_cmd.h"
  "../msg_gen/cpp/include/flyer_controller/control_mode_hover_info.h"
  "../msg_gen/cpp/include/flyer_controller/HoverPoint.h"
  "../msg_gen/cpp/include/flyer_controller/AutosequencePoint.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
