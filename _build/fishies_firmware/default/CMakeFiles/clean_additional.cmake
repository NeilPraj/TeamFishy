# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.cmf"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.hex"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.hxl"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.mum"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.o"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.sdb"
  "/home/cappy/Projects/2026/BMC/TeamFishy/out/fishies_firmware/default.sym"
  )
endif()
