# Saved or newly requested Wi-Fi connection profile.
string connection_id
string ssid
# "open", "wpa-psk", or "eap-tls".
string security
# Write-only. Services returning this message leave it empty.
string password
bool has_password
bool hidden
bool autoconnect
int32 priority
bool enabled
bool connected
# EAP-TLS fields handled by launcher/api/routers/network.py.
string identity
string ca_cert
string pkcs12
string client_cert
string private_key
# Write-only. Services returning this message leave it empty.
string private_key_password
bool has_private_key_password
IPv4Config ipv4
