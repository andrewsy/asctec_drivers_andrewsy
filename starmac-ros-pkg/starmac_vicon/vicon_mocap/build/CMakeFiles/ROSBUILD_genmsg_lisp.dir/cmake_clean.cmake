FILE(REMOVE_RECURSE
  "../src/vicon_mocap/msg"
  "../src/vicon_mocap/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Marker.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Marker.lisp"
  "../msg_gen/lisp/Markers.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Markers.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
