from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("✅ Heartbeat alındı.")

# ACİL DURDURMA: DISARM
master.arducopter_disarm()
print("🛑 ACİL DURDURMA: Disarm edildi.")

# (İsteğe bağlı) RC override'ı sıfırla
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("RC override sıfırlandı.")

master.close()