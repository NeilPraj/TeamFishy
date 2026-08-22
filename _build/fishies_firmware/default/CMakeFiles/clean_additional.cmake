# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.cmf"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.hex"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.hxl"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.mum"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.o"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.sdb"
  "C:\\Users\\Kingsley Dike\\fishies_firmware\\TeamFishy\\out\\fishies_firmware\\default.sym"
  )
endif()
