# Indoor Positioning System Architecture

- **ESP32 Node Layer**: 4 sniffers run in WiFi promiscuous mode, hop across channels 1-13, and emit newline-delimited JSON records over a dedicated hardware serial/USB link to the Raspberry Pi.
- **Transport Layer**: each node uses a separate serial connection, so the ESP32 radio stays dedicated to sniffing instead of trying to remain associated to a WiFi access point while channel hopping.
- **Raspberry Pi Layer**: async serial readers receive frames from one or more ESP32 ports, build 200 ms per-device windows, and estimate `(x, y, z)`.
- **Position Estimation Layer**: SciPy least-squares trilateration with a basic Kalman smoother per tracked MAC.
