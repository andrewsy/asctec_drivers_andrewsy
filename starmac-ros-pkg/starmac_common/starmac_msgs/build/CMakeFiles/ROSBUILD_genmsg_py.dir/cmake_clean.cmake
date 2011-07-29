FILE(REMOVE_RECURSE
  "../src/starmac_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/starmac_msgs/msg/__init__.py"
  "../src/starmac_msgs/msg/_EulerAnglesStamped.py"
  "../src/starmac_msgs/msg/_EulerAngles.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
