FILE(REMOVE_RECURSE
  "../src/flyer_controller/msg"
  "../src/flyer_controller/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/flyer_controller/controllerConfig.h"
  "../docs/controllerConfig.dox"
  "../docs/controllerConfig-usage.dox"
  "../src/flyer_controller/cfg/controllerConfig.py"
  "../docs/controllerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
