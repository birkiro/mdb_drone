FILE(REMOVE_RECURSE
  "../src/mdb_drone/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/mdb_drone/msg/__init__.py"
  "../src/mdb_drone/msg/_chatter.py"
  "../src/mdb_drone/msg/_Obs.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
