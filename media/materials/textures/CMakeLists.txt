set (files
  dirt_diffusespecular.png
  grass_diffusespecular.png
  fungus_diffusespecular.png
  flat_normal.png
  water.jpg
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/materials/textures/)
