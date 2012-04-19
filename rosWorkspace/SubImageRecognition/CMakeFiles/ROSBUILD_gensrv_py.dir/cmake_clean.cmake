FILE(REMOVE_RECURSE
  "src/SubImageRecognition/msg"
  "src/SubImageRecognition/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/SubImageRecognition/srv/__init__.py"
  "src/SubImageRecognition/srv/_ListAlgorithms.py"
  "src/SubImageRecognition/srv/_SwitchAlgorithm.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
