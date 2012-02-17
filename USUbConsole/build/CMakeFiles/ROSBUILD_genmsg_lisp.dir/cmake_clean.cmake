FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/USUbConsole/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Image.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Image.lisp"
  "../msg_gen/lisp/motorMsg.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_motorMsg.lisp"
  "../msg_gen/lisp/imuMsg.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_imuMsg.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
