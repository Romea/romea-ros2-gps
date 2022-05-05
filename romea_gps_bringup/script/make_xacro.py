#!/usr/bin/env python3

import xml.etree.cElementTree as ET
import os
import sys
import glob

if __name__ == "__main__":

   argv = sys.argv

   options = {}
   for argument in argv[1:]:
        name, value = argument.split('=')
        options[name] = value

   xacro_element = ET.Element("robot")
   xacro_element.set("name",options['name'])
   xacro_element.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

   ET.SubElement(
      xacro_element,
      "xacro:include",
      filename="$(find romea_gps_description)/urdf/gps2.xacro",
   )

   gps_xacro_element = ET.SubElement(xacro_element, "xacro:gps_sensor")
   gps_xacro_element.set("xyz", options["xyz"].strip('[]').replace(',', ' '))
   gps_xacro_element.set("parent_link", options["parent_link"])
   gps_xacro_element.set("rate", options["rate"])
   gps_xacro_element.set("name", options["name"])
#   gps_xacro_element.set("wgs84_anchor", options["wgs84_anchor"].strip('[]').replace(',', ' '))

   xacro_directory = os.path.expanduser(options["xacro_directory"])
   if not os.path.exists(xacro_directory) :
      os.makedirs(xacro_directory)

   xacro_filename = options['name']+".xacro"
   xacro_filename = os.path.join(xacro_directory,xacro_filename)

   xacro_tree = ET.ElementTree(xacro_element)
   xacro_tree.write(xacro_filename, xml_declaration=True, encoding="utf-8", method="xml")

   print(ET.tostring(xacro_element, encoding='unicode'))


#   #!/usr/bin/env python3

#   import xml.etree.cElementTree as ET
#   import os
#   import sys
#   import glob

#   if __name__ == "__main__":

#      argv = sys.argv

#      options = {}
#      for argument in argv[1:]:
#           name, value = argument.split('=')
#           options[name] = value

#      xacro_element = ET.Element("robot")
#      xacro_element.set("name",options['name'])
#      xacro_element.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

#      ET.SubElement(
#         xacro_element,
#         "xacro:include",
#         filename="$(find romea_gps_description)/urdf/gps.xacro",
#      )

#      gps_xacro_element = ET.SubElement(xacro_element, "xacro:gps_sensor")
#      gps_xacro_element.set("xyz", options["xyz"].strip('[]').replace(',', ' '))
#      gps_xacro_element.set("parent_link", options["parent_link"])
#      gps_xacro_element.set("rate", options["rate"])
#      gps_xacro_element.set("name", options["name"])
#      gps_xacro_element.set("wgs84_anchor", options["wgs84_anchor"].strip('[]').replace(',', ' '))

#      xacro_directory = os.path.expanduser(options["xacro_directory"])
#      if not os.path.exists(xacro_directory) :
#         os.makedirs(xacro_directory)

#      xacro_filename = options['name']+".xacro"
#      xacro_filename = os.path.join(xacro_directory,xacro_filename)

#      xacro_tree = ET.ElementTree(xacro_element)
#      xacro_tree.write(xacro_filename, xml_declaration=True, encoding="utf-8", method="xml")

#      print(ET.tostring(xacro_element, encoding='unicode'))
