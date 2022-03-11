

ESP-Modbus revision and migration guide
---------------------------------------

1. ESP-Modbus, Revision v1.1.0 - the first revision as a separate project with component manager support.

2. Includes the `test` folder with the unit tests that can run without setup of communication interfaces. The input and output data stored in the internal SPIFFS file system are stored in the partition.

3. The APIs of Modbus Controller interface are still the same compare to previous version (See API reference above).

4. Using of the ESP-Modbus component in the projects requires specified idf_component.yml file:

.. code-block:: cpp

    targets:
      - esp32
      - esp32s2
    description: modbus tcp master
    dependencies:
      idf/esp-modbus:
      version: "1.1.0"


Refer to `Component manager documentation <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html>`__


