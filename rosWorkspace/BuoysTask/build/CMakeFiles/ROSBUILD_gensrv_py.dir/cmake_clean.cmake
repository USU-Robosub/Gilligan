FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/BuoysTask/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/BuoysTask/srv/__init__.py"
  "../src/BuoysTask/srv/_Toggle.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
