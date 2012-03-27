FILE(REMOVE_RECURSE
  "msg_gen"
  "src/subSim/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/MotorMessage.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_MotorMessage.lisp"
  "msg_gen/lisp/mixer.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_mixer.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
