# 阶段四 ELF RPATH 重写器：只允许构建入口传入的明确旧值和新相对值。
foreach(required STAGE4_RPATH_TARGET STAGE4_RPATH_OLD STAGE4_RPATH_NEW)
  if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
    message(FATAL_ERROR "${required} is required")
  endif()
endforeach()

if(NOT IS_ABSOLUTE "${STAGE4_RPATH_TARGET}" OR NOT EXISTS "${STAGE4_RPATH_TARGET}")
  message(FATAL_ERROR "STAGE4_RPATH_TARGET must name an existing absolute file")
endif()

file(RPATH_CHANGE
  FILE "${STAGE4_RPATH_TARGET}"
  OLD_RPATH "${STAGE4_RPATH_OLD}"
  NEW_RPATH "${STAGE4_RPATH_NEW}"
)
