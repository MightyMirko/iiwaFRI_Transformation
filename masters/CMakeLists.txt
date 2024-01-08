# CMakeLists.txt im "masters"-Ordner
cmake_minimum_required(VERSION 3.14)
# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Configuring Binary ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "Configuring Source ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(RL COMPONENTS MDL KIN MATH REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(PythonLibs REQUIRED)
find_package(Doxygen
             REQUIRED dot
             OPTIONAL_COMPONENTS mscgen dia)

include_directories(${PYTHON_INCLUDE_DIRS})
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
                      PRIVATE
                      ${PYTHON_LIBRARIES}
                      FRIClient
                      Boost::iostreams
                      # Weitere Bibliotheken hinzufügen, falls erforderlich
                      )

# Erstellen Sie das Masters-Beispielprogramm
add_executable(mastersApp ${MASTERS_SOURCES} ${MASTERS_HEADERS})

# Linken Sie gegen die erstellte Masters-Bibliothek
target_link_libraries(mastersApp PRIVATE mastersLibrary)
add_subdirectory(tests)