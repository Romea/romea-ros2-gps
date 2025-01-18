cmake_minimum_required(VERSION 3.5)
project(romea_gps_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_gps_description.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gps_urdf test/test_gps_urdf.py)
  ament_add_pytest_test(test_drotek_f9p_description test/test_drotek_f9p_description.py)
  ament_add_pytest_test(test_septentrio_asterx_description test/test_septentrio_asterx_description.py)
  ament_add_pytest_test(test_ashtech_proflex800_description test/test_ashtech_proflex800_description.py)
  ament_add_pytest_test(test_ublox_evk_m8_description test/test_ublox_evk_m8_description.py)
endif()

ament_package()
