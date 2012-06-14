FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/SubImageRecognition/msg"
  "../src/SubImageRecognition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/SubImageRecognition/SwitchAlgorithm.h"
  "../srv_gen/cpp/include/SubImageRecognition/ListAlgorithms.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
