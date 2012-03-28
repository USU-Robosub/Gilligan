FILE(REMOVE_RECURSE
  "msg_gen"
  "src/subSim/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/subSim/msg/__init__.py"
  "src/subSim/msg/_MotorMessage.py"
  "src/subSim/msg/_mixer.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
