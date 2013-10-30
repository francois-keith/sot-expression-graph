INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES test_exp_graph.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

