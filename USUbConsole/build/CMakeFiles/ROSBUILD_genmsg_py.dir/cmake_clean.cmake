FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/USUbConsole/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/USUbConsole/msg/__init__.py"
  "../src/USUbConsole/msg/_Image.py"
  "../src/USUbConsole/msg/_motorMsg.py"
  "../src/USUbConsole/msg/_imuMsg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
