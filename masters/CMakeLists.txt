# CMakeLists.txt im "masters"-Ordner
cmake_minimum_required(VERSION 3.27)
# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Configuring Binary ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "Configuring Source ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(RL COMPONENTS MDL KIN MATH REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
# Find Python 3 interpreter, compiler, and development environment
find_package(Python3 COMPONENTS Interpreter Development NumPy)

# Check if Python 3 is found
if (Python3_FOUND)
    message(STATUS "Python 3 found: ${Python3_EXECUTABLE}")

    # Include directories for Python 3 and NumPy
    include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

    # Link options for Python 3
    set(PYTHON_LINK_OPTIONS ${Python3_LINK_OPTIONS})

    # Additional configurations for your target, if needed

else ()
    message(WARNING "Python 3 not found. Please make sure Python 3 is installed.")
endif ()

find_package(Doxygen
             REQUIRED dot
             OPTIONAL_COMPONENTS mscgen dia)
message(STATUS "Configuring Project Binary ${PROJECT_SOURCE_DIR}")
if (DOXYGEN_FOUND)
    doxygen_add_docs(doxygen_m
                     ${PROJECT_SOURCE_DIR}
                     CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
                     COMMENT "Generating API documentation with Doxygen"
                     )
else ()
    message(WARNING "Doxygen not found, documentation will not be generated.")
endif ()

# Fügen Sie Ihre Quelldateien und Header-Dateien hinzu
file(GLOB MASTERS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE MASTERS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
# Question. This is also adding all cpp from tests to my mastersAPP
# Group source files in CMakeLists.txt
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MASTERS_SOURCES} ${MASTERS_HEADERS})
# Erstellen Sie die Masters-Bibliothek
add_library(mastersLibrary ${MASTERS_SOURCES} ${MASTERS_HEADERS})
# Fügen Sie die include-Verzeichnisse hinzu
target_include_directories(mastersLibrary
                           PUBLIC
                           ${CMAKE_CURRENT_BINARY_DIR}/../${FRIClient_SDK_NAME}/include
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
                           ${PYTHON_HEADERS}
                           )


# Linken Sie gegen FRIClient und andere erforderliche Bibliotheken
target_link_libraries(mastersLibrary
                      PUBLIC
                      ${RL_LIBRARIES}
                      Boost::iostreams
                      FRIClient
                      PRIVATE
                      ${PYTHON_LINK_OPTIONS}
                      ${Python3_LIBRARIES}
                      # Weitere Bibliotheken hinzufügen, falls erforderlich
                      )

# Erstellen Sie das Masters-Beispielprogramm
add_executable(mastersApp mastersApp.cpp)

# Linken Sie gegen die erstellte Masters-Bibliothek
target_link_libraries(mastersApp PRIVATE mastersLibrary)
add_subdirectory(tests)