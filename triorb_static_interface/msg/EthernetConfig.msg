# NetworkManager connection ID of this Ethernet profile. Empty on create unless replacing a known profile.
string connection_id
# OS network interface name, for example eth0. Empty means the service may choose a default wired interface.
string interface_name
# True when NetworkManager may auto-connect this profile.
bool autoconnect
# Auto-connect priority. Higher values are preferred.
int32 priority
# True when this profile should be enabled, false when it should be disabled.
bool enabled
# IPv4 settings of this profile.
IPv4Config ipv4
