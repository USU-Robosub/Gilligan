FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/GateTask/srv"
  "../srv_gen"
  "CMakeFiles/test-results-run"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-results-run.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
