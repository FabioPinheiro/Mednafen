AUTOMAKE_OPTIONS = subdir-objects
DEFS = -DLOCALEDIR=\"$(datadir)/locale\" @DEFS@  @SDL_CFLAGS@
DEFAULT_INCLUDES = -I$(top_builddir)/include -I$(top_srcdir)/intl -I$(top_srcdir)

noinst_LIBRARIES	=	libmdfnsdl.a

libmdfnsdl_a_SOURCES = input.cpp main.cpp args.cpp help.cpp ers.cpp sound.cpp netplay.cpp input-config.cpp
libmdfnsdl_a_SOURCES += console.cpp cheat.cpp fps.cpp video-state.cpp remote.cpp

libmdfnsdl_a_SOURCES += opengl.cpp shader.cpp
libmdfnsdl_a_SOURCES += overlay.cpp

libmdfnsdl_a_SOURCES += video.cpp

libmdfnsdl_a_SOURCES += nongl.cpp nnx.cpp

if WANT_FANCY_SCALERS
libmdfnsdl_a_SOURCES += hqxx-common.cpp hq2x.cpp hq3x.cpp hq4x.cpp scale2x.c scale3x.c scalebit.c 2xSaI.cpp
endif

libmdfnsdl_a_SOURCES += Joystick.cpp Joystick_SDL.cpp

if HAVE_LINUX_JOYSTICK
libmdfnsdl_a_SOURCES += Joystick_Linux.cpp
endif

if HAVE_POSIX_SOCKETS
libmdfnsdl_a_SOURCES += NetClient_POSIX.cpp
endif

if WIN32
libmdfnsdl_a_SOURCES += Joystick_XInput.cpp
libmdfnsdl_a_SOURCES += Joystick_DX5.cpp
#Joystick_DX8.cpp
libmdfnsdl_a_SOURCES += NetClient_WS2.cpp
endif

if WANT_DEBUGGER
libmdfnsdl_a_SOURCES += debugger.cpp gfxdebugger.cpp memdebugger.cpp logdebugger.cpp prompt.cpp
endif
