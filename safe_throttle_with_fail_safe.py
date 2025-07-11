import time
from pymavlink import mavutil

def acil_durdur(master):
    print("Acil durdurma başlatıldı!")
    for i in range(10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, 1000, 1500, 0, 0, 0, 0
        )
        time.sleep(0.1)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.set_mode_apm('STABILIZE')
    time.sleep(1)
    for i in range(5):
        master.arducopter_disarm()
        time.sleep(0.5)
    print("Acil durdurma tamamlandı.")

try:
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print("✅ Heartbeat alındı.")

    # Kodun başında override'ı sıfırla
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    master.arducopter_arm()
    print("🔧 ARM gönderildi.")
    time.sleep(2)

    master.set_mode_apm('STABILIZE')
    print("✈️ Mod: STABILIZE")
    time.sleep(2)

    THROTTLE_PWM = 1200
    DURATION = 5

    for i in range(DURATION * 10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, THROTTLE_PWM, 1500, 0, 0, 0, 0
        )
        time.sleep(0.1)

    acil_durdur(master)
    master.close()
    print("✅ Program sonlandı.")

except Exception as e:
    print(f"❌ Hata oluştu: {e}")
    try:
        acil_durdur(master)
        master.close()
    except:
        print("Acil durdurma sırasında hata!")