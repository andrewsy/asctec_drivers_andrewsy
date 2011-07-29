FILE(REMOVE_RECURSE
  "../src/vicon_mocap/msg"
  "../src/vicon_mocap/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/vicon_mocap/srv/__init__.py"
  "../src/vicon_mocap/srv/_viconGrabPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
