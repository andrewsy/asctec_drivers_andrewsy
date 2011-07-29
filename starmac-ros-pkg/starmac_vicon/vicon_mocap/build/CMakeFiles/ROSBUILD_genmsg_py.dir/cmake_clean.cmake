FILE(REMOVE_RECURSE
  "../src/vicon_mocap/msg"
  "../src/vicon_mocap/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/vicon_mocap/msg/__init__.py"
  "../src/vicon_mocap/msg/_Marker.py"
  "../src/vicon_mocap/msg/_Markers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
