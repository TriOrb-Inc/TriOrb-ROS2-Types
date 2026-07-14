# DRAFT: managed NetworkManager hotspot profile.
string connection_id
bool enabled
string ssid
string security
# Write-only. Services returning this message leave it empty.
string passphrase
bool has_passphrase
string band
uint32 channel
IPv4Config ipv4
