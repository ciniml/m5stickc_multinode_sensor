set(COMPONENT_SRCS main.cpp bmm150/bmm150.c src/clock_source.cpp)
set(COMPONENT_PRIV_INCLUDEDIRS "$ENV{HOME}/boost_1_70_0" include bmm150)

set(COMPONENT_EMBED_FILES "index.html" "script.js")

register_component()

target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-DBMM150_USE_FLOATING_POINT")
