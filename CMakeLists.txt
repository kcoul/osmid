cmake_minimum_required (VERSION 3.5)

project (osmid)

include(FetchContent)
# set fetch content verbose
set(FETCHCONTENT_QUIET OFF)

# fetch JUCE
FetchContent_Declare(
        juce
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE
        GIT_TAG 4f43011b96eb0636104cb3e433894cda98243626
)
FetchContent_Populate(juce)
FetchContent_MakeAvailable(juce)

# add juce subdirectory
add_subdirectory(${juce_SOURCE_DIR} ${juce_BINARY_DIR})

set(CMAKE_VERBOSE_MAKEFILE ON)

if(CMAKE_SYSTEM_NAME MATCHES "^(Linux)")
    find_package(ALSA REQUIRED)
endif()


set(oscpack_path ${PROJECT_SOURCE_DIR}/external_libs/oscpack_1_1_0)
add_subdirectory(${oscpack_path})

include_directories(${oscpack_path}
        ${PROJECT_SOURCE_DIR}/external_libs/spdlog-0.11.0/include
        ${PROJECT_SOURCE_DIR}/external_libs/cxxopts)

set(m2o_sources
    src/m2o.cpp
    src/midiin.cpp
    src/oscout.cpp
    src/midiinprocessor.cpp
    src/midicommon.cpp
    src/utils.cpp
)

set(o2m_sources
    src/o2m.cpp
    src/midiout.cpp
    src/oscin.cpp
    src/oscout.cpp
    src/midicommon.cpp
    src/oscinprocessor.cpp
    src/utils.cpp
)

# m2o
juce_add_console_app(m2o VERSION "0.0.1")
juce_generate_juce_header(m2o)
target_sources(m2o PRIVATE
        ${m2o_sources})

target_link_libraries(m2o PRIVATE
        juce::juce_audio_devices
        juce::juce_audio_formats
        juce::juce_audio_processors
        juce::juce_audio_utils
        juce::juce_core
        juce::juce_data_structures
        juce::juce_events
        juce::juce_graphics
        juce::juce_gui_basics
        juce::juce_gui_extra
        oscpack)

# o2m
juce_add_console_app(o2m VERSION "0.0.1")
juce_generate_juce_header(o2m)
target_sources(o2m PRIVATE
        ${o2m_sources})

target_link_libraries(o2m PRIVATE
        juce::juce_audio_devices
        juce::juce_audio_formats
        juce::juce_audio_processors
        juce::juce_audio_utils
        juce::juce_core
        juce::juce_data_structures
        juce::juce_events
        juce::juce_graphics
        juce::juce_gui_basics
        juce::juce_gui_extra
        oscpack)

#add_definitions(-DJUCE_ALSA_MIDI_NAME="osmid_midi")

if(MSVC)
    #add_definitions(-D_WIN32_WINNT=0x0600 -DJUCER_VS2015_78A5022=1 -DJUCE_APP_VERSION=1.0.0 -DJUCE_APP_VERSION_HEX=0x10000)
    #target_link_libraries(m2o winmm Ws2_32)
    #target_link_libraries(o2m winmm Ws2_32)
elseif(APPLE)
    #add_definitions(-DNDEBUG=1 -DJUCER_XCODE_MAC_F6D2F4CF=1 -DJUCE_APP_VERSION=1.0.0 -DJUCE_APP_VERSION_HEX=0x10000)
    #set_target_properties(m2o PROPERTIES XCODE_ATTRIBUTE_CLANG_LINK_OBJC_RUNTIME "NO")
    #set_target_properties(o2m PROPERTIES XCODE_ATTRIBUTE_CLANG_LINK_OBJC_RUNTIME "NO")
    #set(CMAKE_EXE_LINKER_FLAGS "-framework CoreMIDI -framework CoreAudio -framework CoreFoundation -framework Accelerate -framework QuartzCore -framework AudioToolbox -framework IOKit -framework DiscRecording -framework Cocoa")
elseif(UNIX)
    #add_definitions(-DLINUX=1 -DNDEBUG=1 -DJUCER_LINUX_MAKE_6D53C8B4=1 -DJUCE_APP_VERSION=1.0.0 -DJUCE_APP_VERSION_HEX=0x10000)
    #target_link_libraries(m2o pthread ${ALSA_LIBRARY} dl rt X11)
    #target_link_libraries(o2m pthread ${ALSA_LIBRARY} dl rt X11)
endif()

if(UNIX)
    install (TARGETS m2o DESTINATION bin)
    install (TARGETS o2m DESTINATION bin)
endif(UNIX)
