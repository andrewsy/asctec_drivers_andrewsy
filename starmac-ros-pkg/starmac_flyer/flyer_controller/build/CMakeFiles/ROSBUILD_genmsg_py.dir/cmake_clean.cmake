FILE(REMOVE_RECURSE
  "../src/flyer_controller/msg"
  "../src/flyer_controller/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/flyer_controller/msg/__init__.py"
  "../src/flyer_controller/msg/_control_mode_autosequence_info.py"
  "../src/flyer_controller/msg/_control_mode_output.py"
  "../src/flyer_controller/msg/_Autosequence.py"
  "../src/flyer_controller/msg/_control_mode_cmd.py"
  "../src/flyer_controller/msg/_controller_status.py"
  "../src/flyer_controller/msg/_control_mode_status.py"
  "../src/flyer_controller/msg/_controller_cmd.py"
  "../src/flyer_controller/msg/_control_mode_hover_info.py"
  "../src/flyer_controller/msg/_HoverPoint.py"
  "../src/flyer_controller/msg/_AutosequencePoint.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
