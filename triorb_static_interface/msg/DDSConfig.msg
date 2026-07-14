# DRAFT: ROS 2 discovery settings applied when processes start.
string rmw_implementation
# SYSTEM_DEFAULT, SUBNET, LOCALHOST or OFF.
string automatic_discovery_range
string[] static_peers
# Fast DDS discovery server endpoints, when used.
string[] discovery_servers
string network_interface
bool multicast_enabled
