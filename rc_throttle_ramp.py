import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("✅ Heartbeat alındı.")

master.arducopter_arm()
print("🔧 ARM gönderildi.")
time.sleep(2)

# Uçuş modunu STABILIZE yap
master.set_mode_apm('STABILIZE')
time.sleep(2)

# Throttle stickini yavaşça artır (3. kanal)
for pwm in range(1100, 1700, 20):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,  # Roll (1. kanal) - ortada
        1500,  # Pitch (2. kanal) - ortada
        pwm,   # Throttle (3. kanal) - artırılıyor
        1500,  # Yaw (4. kanal) - ortada
        0, 0, 0, 0
    )
    print(f"Throttle PWM: {pwm}")
    time.sleep(0.1)

# Throttle'ı tekrar düşür
for pwm in range(1700, 1100, -20):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, pwm, 1500, 0, 0, 0, 0
    )
    print(f"Throttle PWM: {pwm}")
    time.sleep(0.1)

master.arducopter_disarm()
print("🛑 Disarm edildi.")

# Override'ı sıfırla
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Override sıfırlandı.")


master.close()