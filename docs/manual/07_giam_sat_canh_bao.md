# 7. Device Monitoring & Alarms

## Device status lights

Four lights on the Main View show whether key hardware is connected. **Green** means connected, **red** means disconnected, **gray** means unknown/not yet determined.

| Light | Green (connected) means | Red (disconnected) means |
|---|---|---|
| **Lidar** | The system can reach the scanner over the network. | The system cannot reach the scanner — check the scanner's network cable and power. **Note:** this light reflects the network connection, not whether a scan is currently running — it can stay green even when no scan is in progress. |
| **Encoder** | The system is still receiving data from the encoder. | The encoder has stopped sending data — check its cabling. |
| **PCAN** (CAN gateway) | The system is still receiving data through the CAN gateway. | The CAN gateway has stopped sending data — check its cabling and power. |
| **PLC** | The system is still hearing from the PLC. | The PLC has stopped responding — check its cabling and power. |

See Chapter 8 for what to do when a light turns red.

## Notification bar and history

- A line near the bottom of the screen shows the latest status or warning message.
- Color meaning: **green** = informational, **amber** = warning, **red** = error.
- An error message stays on screen for at least 10 seconds, even if a routine info message would otherwise replace it — so an important warning isn't missed.
- Tap the notification bar to open the full history (the most recent messages) if you need to check what happened earlier.
