# NetworkManager connection ID of this hotspot profile. Empty on create unless replacing a known profile.
string connection_id
# True when the hotspot should be enabled, false when it should be disabled.
bool enabled
# Network name advertised by the hotspot.
string ssid
# Authentication mode. Typical values: "open" or "wpa-psk".
string security
# Write-only PSK or passphrase. Services returning this message leave it empty.
string password
# True when a password is stored for this profile. The password value itself is not returned.
bool has_password
# Radio band. Typical NetworkManager values: "bg" for 2.4 GHz, "a" for 5 GHz.
string band
# Wi-Fi channel number. 0 means let NetworkManager choose.
uint32 channel
# IPv4 settings of this hotspot. Hotspots often use method "shared".
IPv4Config ipv4
