FILE(REMOVE_RECURSE
  "msg_gen"
  "src/subSim/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/subSim/MotorMessage.h"
  "msg_gen/cpp/include/subSim/mixer.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
