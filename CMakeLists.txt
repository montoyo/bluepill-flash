cmake_minimum_required(VERSION 3.8)
project(bluepill-flash)

set(CMAKE_CXX_STANDARD 11)

# MGPCL inclusion
set(MGPCL_INCLUDE_DIR "" CACHE PATH "Where the MGPCL's include directory is located")
set(MGPCL_LIBRARY_DIR "" CACHE PATH "Where the MGPCL static library is located")
include_directories(${MGPCL_INCLUDE_DIR})
link_directories(${MGPCL_LIBRARY_DIR})

# Read MGPCL settings
include(CheckSymbolExists)
check_symbol_exists(MGPCL_NO_GUI ${MGPCL_INCLUDE_DIR}/mgpcl/Config.h MGPCL_NO_GUI)
check_symbol_exists(MGPCL_NO_SSL ${MGPCL_INCLUDE_DIR}/mgpcl/Config.h MGPCL_NO_SSL)

if(UNIX AND NOT MGPCL_NO_GUI)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
    include_directories(${GTK3_INCLUDE_DIRS})
    link_directories(${GTK3_LIBRARY_DIRS})
endif()

if(WIN32 AND NOT MGPCL_NO_SSL)
	set(OPENSSL_INCLUDE_DIR "" CACHE PATH "Where the OpenSSL's include directory is located")
	set(OPENSSL_LIBRARY_DIR "" CACHE PATH "Where the OpenSSL library is located")
	include_directories(${OPENSSL_INCLUDE_DIR})
	link_directories(${OPENSSL_LIBRARY_DIR})
endif()

# bluepill-flash settings
set(SOURCE_FILES main.cpp)
add_executable(bluepill-flash ${SOURCE_FILES})

# MGPCL static libraries
if(UNIX)
	target_link_libraries(bluepill-flash mgpcl pthread)
else()
	target_link_libraries(bluepill-flash mgpcl ws2_32 Shlwapi Wbemuuid)
endif()

if(NOT MGPCL_NO_SSL)
    message(STATUS "Linking OpenSSL")
	
	if(WIN32)
		target_link_libraries(bluepill-flash libcrypto libssl crypt32)
	else()
		target_link_libraries(bluepill-flash crypto ssl)
	endif()
endif()

if(UNIX AND NOT MGPCL_NO_GUI)
    message(STATUS "Linking GTK3")
    target_link_libraries(bluepill-flash ${GTK3_LIBRARIES})
endif()
