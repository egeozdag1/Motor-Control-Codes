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

    # 1. Sabit düşük PWM'de başlat (ör: 1200)
    START_PWM = 1100
    STOP_PWM = 1300
    HOLD_TIME = 2  # saniye
    for i in range(HOLD_TIME * 10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, START_PWM, 1500, 0, 0, 0, 0
        )
        print(f"Motorlar şu an {START_PWM} PWM'de çalışıyor.")
        time.sleep(0.2) ## HOLD_TIME/time.sleep

    # 2. PWM'i yavaşça 1700'e çıkar
    for pwm in range(START_PWM, STOP_PWM, 8):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, pwm, 1500, 0, 0, 0, 0
        )
        print(f"Motorlar şu an {pwm} PWM'de çalışıyor.")
        time.sleep(0.2)

    # 3. PWM'i yavaşça 1100'e indir
    for pwm in range(STOP_PWM, START_PWM, -8):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, pwm, 1500, 0, 0, 0, 0
        )
        print(f"Motorlar şu an {pwm} PWM'de çalışıyor.")
        time.sleep(0.2)

    # 4. Acil durdurma
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