# cmake_modules/boost.cmake

find_package(Boost REQUIRED COMPONENTS 
  serialization 
  thread 
  timer 
  chrono 
)
