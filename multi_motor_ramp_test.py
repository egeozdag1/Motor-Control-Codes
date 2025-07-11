#BU KOD TUM MOTORLARI AYNI ANDA BELIRLI BIR HIZDAN BELIRLI BIR HIZA CIKARIP
#AYNI ANDA SIFIRLAMAK ICIN
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

MOTOR_IDS = [1,2,3,4]
DURATION = 0.1  # Her adımda motorun çalışma süresi (saniye)
min_throttle = 15
max_throttle = 25
step = 1

# Yavaş yavaş artır: %40 → %70
for throttle in range(min_throttle, max_throttle, step):
    for motor_id in MOTOR_IDS:
        test_motor(master, motor_id, throttle=throttle, duration=DURATION)
    time.sleep(DURATION)

# Azalt
for throttle in range(max_throttle, min_throttle, -step):
    for motor_id in MOTOR_IDS:
        test_motor(master, motor_id, throttle=throttle, duration=DURATION)
    time.sleep(DURATION)

# DISARM
master.arducopter_disarm()
print("🛑 Disarm edildi.")
master.close()
