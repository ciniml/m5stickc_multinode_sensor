set(COMPONENT_SRCS main.cpp bmm150/bmm150.c)
set(COMPONENT_PRIV_INCLUDEDIRS "$ENV{HOME}/boost_1_70_0" include bmm150)

register_component()

target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-DBMM150_USE_FLOATING_POINT")
