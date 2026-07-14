# DRAFT: runtime state and saved configuration of one network connection.
string connection_id
string interface_name
# ethernet, wifi, hotspot or another NetworkManager connection type.
string connection_type
string state
string mac_address
bool default_route
IPv4Config ipv4
WifiConfig wifi
