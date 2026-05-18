# cmake_modules/vtk.cmake

# find_package(VTK REQUIRED)
find_package(VTK REQUIRED COMPONENTS 
  vtkCommonCore 
  vtkIOImage
)

# include_directories(${VTK_INCLUDE_DIRS})
# link_libraries(${VTK_LIBRARIES})