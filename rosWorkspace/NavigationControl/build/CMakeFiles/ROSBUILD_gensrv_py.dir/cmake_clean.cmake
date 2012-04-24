FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/NavigationControl/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/NavigationControl/srv/__init__.py"
  "../src/NavigationControl/srv/_Navigate.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
