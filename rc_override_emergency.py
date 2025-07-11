import time
from pymavlink import mavutil

def acil_durdur(master):
    print("Durdurma başlatıldı!")
    # Throttle minimum
    for i in range(10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, 1000, 1500, 0, 0, 0, 0
        )
        time.sleep(0.1)
    # Override sıfırla
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    # Modu STABILIZE yap
    master.set_mode_apm('STABILIZE')
    time.sleep(1)
    # Disarm'ı birkaç kez dene
    for i in range(5):
        master.arducopter_disarm()
        time.sleep(0.5)
    print("Acil durdurma tamamlandı.")

# UART portunu kendi bağlantına göre değiştir (ör: /dev/ttyAMA0, /dev/serial0, /dev/ttyUSB0)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("✅ Heartbeat alındı.")

master.arducopter_arm()
print("🔧 ARM gönderildi.")
time.sleep(2)

# Uçuş modunu STABILIZE yap
master.set_mode_apm('STABILIZE')
time.sleep(2)

# Tüm stickler ortada, throttle yüksek (ör: 1600)
THROTTLE_PWM = 1200
DURATION = 3  # saniye

for i in range(DURATION * 10):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,  # Roll (1. kanal)
        1500,  # Pitch (2. kanal)
        THROTTLE_PWM,  # Throttle (3. kanal)
        1500,  # Yaw (4. kanal)
        0, 0, 0, 0
    )
    time.sleep(0.1)

# Kodunun sonunda:
acil_durdur(master)
master.close()