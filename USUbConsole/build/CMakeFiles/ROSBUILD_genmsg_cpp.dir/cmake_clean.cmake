FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/USUbConsole/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/USUbConsole/Image.h"
  "../msg_gen/cpp/include/USUbConsole/motorMsg.h"
  "../msg_gen/cpp/include/USUbConsole/imuMsg.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
