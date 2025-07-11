#BU KOD BIR TANE MOTOR HIZINI ARTTIRIP AZALTMAK ICIN
import time
from pymavlink import mavutil

# Fonksiyon: Belirli bir motora PWM (%) ile throttle gönderir
def test_motor(master, motor_id, throttle=20, duration=2):
    msg = mavutil.mavlink.MAVLink_command_long_message(
        target_system=master.target_system,
        target_component=master.target_component,
        command=mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        confirmation=0,
        param1=motor_id,   # Motor ID (1–4)
        param2=0,          # 0 = duty cycle (%)
        param3=throttle,   # PWM yüzdesi
        param4=duration,   # Süre (saniye)
        param5=0,
        param6=0,
        param7=0
    )
    master.mav.send(msg)
    print(f"Motor {motor_id} → %{throttle} hızla çalıştı ({duration} sn)")

# MAVLink bağlantısı başlat
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("✅ Heartbeat alındı.")

# ARM işlemi
master.arducopter_arm()
print("🔧 ARM gönderildi.")
time.sleep(1)

MOTOR_ID = 1
DURATION = 0.1  # Her adımda motorun çalışma süresi (saniye)

# Yavaş yavaş artır: %40 → %70
for throttle in range(15, 20, 1):  # 40, 45, 50, ..., 70
    test_motor(master, MOTOR_ID, throttle=throttle, duration=DURATION)
    time.sleep(DURATION)

# Yavaş yavaş azalt: %70 → %0
for throttle in range(20, 15, -1):  # 70, 65, ..., 0
    test_motor(master, MOTOR_ID, throttle=throttle, duration=DURATION)
    time.sleep(DURATION)

# DISARM
master.arducopter_disarm()
print("🛑 Disarm edildi.")
master.close()
