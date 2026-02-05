from pymavlink import mavutil

# get from sim computer 'ip a' in bash
# Wi-Fi or Ethernet will be inet xxx.xxx.x.xxx global {dynamic/enp3s0}
IP = ""
PORT = 14570

master = mavutil.mavlink_connection(
    f'udp:{IP}:{PORT}'
)

master.wait_heartbeat()
print("Connected to ArduSub SITL")

while True:
    msg = master.recv_match(blocking=True)
    if msg:
        print(msg.get_type())