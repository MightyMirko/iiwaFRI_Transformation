# CMakeLists.txt im "masters"-Ordner


find_package(RL COMPONENTS MDL KIN MATH REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Fügen Sie Ihre Quelldateien und Header-Dateien hinzu
file(GLOB_RECURSE MASTERS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE MASTERS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

message(STATUS "Configuring Binary ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "Configuring Source ${CMAKE_CURRENT_SOURCE_DIR}")

# Erstellen Sie die Masters-Bibliothek
add_library(mastersLibrary ${MASTERS_SOURCES} ${MASTERS_HEADERS})

# Fügen Sie die include-Verzeichnisse hinzu
target_include_directories(mastersLibrary
                           PUBLIC
                           ${CMAKE_CURRENT_BINARY_DIR}/../${FRIClient_SDK_NAME}/include
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
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
