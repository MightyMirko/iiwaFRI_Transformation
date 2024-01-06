
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
        )
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
# Erstellen Sie das Test-Programm
add_executable(mastersTests
               myfirsttest.cpp)
# Linken Sie gegen die erstellte Masters-Bibliothek und Google Test
target_link_libraries(mastersTests
                      PUBLIC
                      GTest::gtest_main

                      mastersLibrary
                      )

include(GoogleTest)

gtest_discover_tests(mastersTests)