# NetworkManager connection ID of this Wi-Fi profile.
string connection_id
# Network name of this Wi-Fi profile.
string ssid
# Authentication mode. Known values: "open", "wpa-psk", or "eap-tls".
string security
# Write-only PSK or passphrase. Services returning this message leave it empty.
string password
# True when a password is stored for this profile. The password value itself is not returned.
bool has_password
# True when the SSID is hidden.
bool hidden
# True when NetworkManager may auto-connect this profile.
bool autoconnect
# Auto-connect priority. Higher values are preferred.
int32 priority
# True when this profile is enabled.
bool enabled
# True when this profile is currently connected.
bool connected
# EAP identity for eap-tls.
string identity
# Path or identifier of the CA certificate for eap-tls.
string ca_cert
# Path or identifier of the PKCS#12 bundle for eap-tls.
string pkcs12
# Path or identifier of the client certificate for eap-tls.
string client_cert
# Path or identifier of the client private key for eap-tls.
string private_key
# Write-only private key password. Services returning this message leave it empty.
string private_key_password
# True when a private key password is stored. The password value itself is not returned.
bool has_private_key_password
# IPv4 settings of this profile.
IPv4Config ipv4
