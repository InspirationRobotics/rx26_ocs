# Purpose of the project
Create a Operator Control Station for robotX 2026 competition. The OCS is required to communicate with RoboNation's DHCP server. 3 vehicles, UAV/UUV/USV will need to be able to report status to OCS and OCS will report these status to RoboNation's DHCP server. RoboNation server will also send information back to OCS and OCS will send information back to vehicles.

# Resources
Robotx 2026 communication requirement: https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.
4-communications-protocol
RoboNation RoboCommand repo: https://github.com/robonation/robocommand/tree/main/RobotX_2026
USV repo: https://github.com/InspirationRobotics/rx26_asv/tree/clean_usv
UUV repo: https://github.com/InspirationRobotics/robotx_graey_2026.git

# Network between vehilces
USV communicate with OCS through WIFI
UUV communicate with USV, then the USV forwards its information to OCS
UAV communicate with OCS thorugh WIFI
USV,UAV,OCS will be under the same subnet

# OCS requirement (Read all resources before you continue with this section)
OCS shall be able to connect with all three vehicles. 
OSC shall have all the capability that USV&UUV web interface has, including ability to start individual node; view video/lidar stream if able to; turn LED off for land testing; shutdown jetson
OSC shall be able to view all rx_report/rx_common information.
OSC shall be able to send rx_requests to RoboNation server to declare a run.
OSC shall eb able to receive rx_command from RoboNation server and send msg back to corresponded vehicle.
OSC shall be able to start autonomous missoins on all three vehicles.









