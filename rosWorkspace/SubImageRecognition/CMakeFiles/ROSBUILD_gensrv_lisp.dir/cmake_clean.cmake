FILE(REMOVE_RECURSE
  "src/SubImageRecognition/msg"
  "src/SubImageRecognition/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/ListAlgorithms.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_ListAlgorithms.lisp"
  "srv_gen/lisp/SwitchAlgorithm.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_SwitchAlgorithm.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
