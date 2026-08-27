# NetworkManager connection ID of this interface profile.
string connection_id
# OS network interface name, for example wlan0 or eth0.
string interface_name
# NetworkManager connection type, for example ethernet, wifi, or hotspot.
string connection_type
# Runtime connection state reported by NetworkManager.
string state
# Interface MAC address.
string mac_address
# True when this connection is the default IPv4 route.
bool default_route
# Runtime IPv4 addresses of this connection.
IPv4Config ipv4
# Wi-Fi profile details when connection_type is wifi. Otherwise fields are empty defaults.
WifiConfig wifi
