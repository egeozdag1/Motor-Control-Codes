#BU KOD 1-2-3-4 SIRAYLA MOTORLARI DIREKT OLARAK ISTENILEN THROTTLE DEGERINE CEKER.

import time
from pymavlink import mavutil

# MAVLink bağlantısını başlat
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
#master.wait_heartbeat()
#print("✅ Heartbeat alındı.")

# Arming işlemi
master.arducopter_arm()
print("🔧 ARM gönderildi.")
time.sleep(1)

# Tüm motorları sırayla çalıştır
for motor_id in range(1, 5):  # 1'den 4'e (Motor 1, 2, 3, 4)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,                  # confirmation
        motor_id,           # param1: motor index
        0,                  # param2: throttle type (0 = duty cycle %)
        75,                # param3: throttle % (örnek: %50)
        5,                  # param4: duration (saniye)
        0, 0, 0             # param5-7: kullanılmıyor
    )
    print(f"🚀 Motor {motor_id} çalışıyor (%100, 3 sn)")
    time.sleep(0.5)  # Komutlar arasında kısa gecikme

# 5 saniye bekle
time.sleep(5)

# Disarm işlemi
master.arducopter_disarm()
print("🛑 Disarm edildi. Motorlar durdu.")

# Bağlantıyı kapat
master.close()
