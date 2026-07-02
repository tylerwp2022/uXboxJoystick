#--------------------------------------------------------
# The CMakeLists.txt for: uXboxJoystick
# Author(s): Tyler Errico
#--------------------------------------------------------

SET(SRC
  XboxJoystick.cpp
  XboxJoystick_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uXboxJoystick ${SRC})

TARGET_LINK_LIBRARIES(uXboxJoystick
  ${MOOS_LIBRARIES}
  -Wl,--start-group
  contacts
  apputil
  geometry
  mbutil
  -Wl,--end-group
  ${SYSTEM_LIBS}
)
