# Wireless Gamecube Controller

# About 
This guide documents how to turn an original wired Nintendo GameCube controller into a wireless one using two Pro Micro nRF52840 boards. While this started as more of an experiment, the project was very useful and achieved very good performance so I put together a guide to make it easy for anyone to understand and install.

[![CI](https://img.shields.io/github/actions/workflow/status/codycarter1763/Wireless-Gamecube-Controller/build-nrf.yml?branch=main&style=for-the-badge&label=CI)](https://github.com/codycarter1763/Wireless-Gamecube-Controller/actions/workflows/build-nrf.yml)

## Features

| Feature | Details |
|---|---|
| **Low latency** | 1 kHz pad polling and a 2 Mbps radio. The receiver answers the console from a pre-built reply, so the console never waits on the radio. |
| **Long battery life** | About 19 days or 450+ hours of non-stop play with integrated 2000 mAh battery. |
| **Power saving** | Uses adaptive polling and auto deep-sleep logic for long playing and storage battery life. |
| **Robust link** | Automatic retries, duplicate and out-of-order packet filtering, and auto-reconnection if the receiver disappears. |
| **Wireless updates** | Controller button binds to reboot into Bluetooth DFU mode to flash new controller firmware wirelessly. |

# Gamecube Controller Protocol
Referring to a very useful reverse-engineering experiment for the Gamecube protocol done [here](https://www.int03.co.uk/crema/hardware/gamecube/gc-control.html), it helped document specific timing requirements and decode how the controller talks to the console and vice versa.

## The Data Wire
 
The data line is **open-drain**. A pull-up
resistor holds it at 3.3 V, and the console or the pad pulls it low to
send the data packet.
 
| Pin | Signal |
|---|---|
| 1 | 5 V (rumble motor) |
| 2 | Data (3.3 V, open-drain) |
| 3 | Ground |
| 4 | Ground |
| 5 | Not connected |
| 6 | 3.3 V (logic) |
| 7 | Cable shield / ground |
## Data Packet



# Installation

# Testing

# Conclusion
