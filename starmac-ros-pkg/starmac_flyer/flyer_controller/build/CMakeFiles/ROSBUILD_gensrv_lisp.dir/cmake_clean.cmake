FILE(REMOVE_RECURSE
  "../src/flyer_controller/msg"
  "../src/flyer_controller/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/GetAutosequence.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetAutosequence.lisp"
  "../srv_gen/lisp/control_modes.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_control_modes.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
