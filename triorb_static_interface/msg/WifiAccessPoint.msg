# Network name advertised by the access point.
string ssid
# Access point MAC address reported by NetworkManager.
string bssid
# Signal strength as a percentage reported by NetworkManager.
uint8 signal_percent
# Wi-Fi channel number.
uint32 channel
# Center frequency in MHz.
uint32 frequency_mhz
# PHY mode reported by NetworkManager, for example Infra.
string mode
# Maximum bitrate reported by NetworkManager, for example "130 Mbit/s".
string rate
# Security type reported by NetworkManager, for example WPA2.
string security
# True when this access point is the currently connected network.
bool in_use
# True when NetworkManager already has a saved profile for this SSID.
bool saved
