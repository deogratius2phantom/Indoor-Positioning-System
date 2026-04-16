# Indoor Positioning System Architecture

- **ESP32 Node Layer**: 4 sniffers in WiFi promiscuous mode collect `(MAC, RSSI, channel, timestamp)` and forward readings over UDP.
- **Raspberry Pi Layer**: async UDP service receives readings, builds 200 ms per-device windows, and estimates `(x, y, z)`.
- **Position Estimation Layer**: SciPy least-squares trilateration with a basic Kalman smoother per tracked MAC.
