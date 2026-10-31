# airaware-detecting-asthma-attack-severity
MIT Maker Portfolio Github Repository Containing All of AirAware's Major Parts

This project features AirAware, something I designed in a group of four during my junior year of high school. I conducted all of the circuitry and code-level work for this project, programming the BLE connections, soldering the circuits, and getting both circuits to read the names accurately. AirAware has 2 major components: a box worn on the wrist (like a watch) that contains a Pulse Oximeter to read heart rate and SPO2 (oxygen saturation in the blood), and a patch that can be stuck onto the stomach that uses 3 electrode patches worn in a certain formation to detect contractions in the diaphragm.

Circuitry Components (major ones, amplifiers/voltage regulators not included): Arduino NANO 33 IoT, Seeed Studio XIAO microcontroller, MAX30102 Pulse Oximeter, Grove EMG Detector, 3 electrode patches.

This device has 2 code files, one for the wristband and one for the stomach patch, and they are connected via Bluetooth-Low-Energy (BLE). The wristband takes all of the information from itself and the stomach patch and prints it to the serial monitor.

Please feel free to view the circuitry diagrams and overall images that went into making AirAware! They are all in this Github repository.
