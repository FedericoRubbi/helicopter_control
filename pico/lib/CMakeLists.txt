# Include RF24's CMakeLists.txt
# giving the path depending on where the library
# is cloned to in your project
include(RF24/CMakeLists.txt)
add_subdirectory(transmitter)
add_subdirectory(ins)
add_subdirectory(control)
add_subdirectory(datalogger)
add_subdirectory(test_module)