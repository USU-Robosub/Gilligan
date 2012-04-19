FILE(REMOVE_RECURSE
  "src/SubImageRecognition/msg"
  "src/SubImageRecognition/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/SubImageRecognition/msg/__init__.py"
  "src/SubImageRecognition/msg/_ImgRecObject.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
