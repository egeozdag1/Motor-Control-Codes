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

def force_disarm(master):
    """Zorla disarm yapar"""
    print("Zorla disarm yapılıyor...")
    for i in range(10):
        master.arducopter_disarm()
        time.sleep(0.1)
    print("Disarm tamamlandı.")

def clear_rc_override(master):
    """RC override'ı temizler"""
    print("RC override temizleniyor...")
    for i in range(5):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)
    print("RC override temizlendi.")

try:
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print("✅ Heartbeat alındı.")

    # 1. Önceki durumu temizle
    print("\n1. Önceki durumu temizleme...")
    clear_rc_override(master)
    force_disarm(master)
    time.sleep(2)

    # 2. ARM işlemi
    print("\n2. ARM işlemi...")
    arm_result = master.arducopter_arm()
    print(f"ARM sonucu: {arm_result}")
    
    if not arm_result:
        print("❌ ARM başarısız! Alternatif yöntem deneniyor...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
    
    time.sleep(2)

    # 3. Uçuş modunu ayarla
    print("\n3. Uçuş modu ayarlanıyor...")
    master.set_mode_apm('STABILIZE')
    print("✈️ Mod: STABILIZE")
    time.sleep(2)

    # 4. ARM durumunu kontrol et
    print("\n4. ARM durumu kontrol ediliyor...")
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        print(f"System Status: {msg.system_status}")
        if msg.system_status in [3, 4]:  # MAV_STATE_STANDBY veya MAV_STATE_ACTIVE
            print("✅ ARM başarılı!")
        else:
            print(f"⚠️ Sistem durumu: {msg.system_status} - ARM başarısız olabilir")

    # 5. Motor testi başlat
    print("\n5. Motor testi başlatılıyor...")
    
    # Sabit düşük PWM'de başlat (ör: 1200)
    START_PWM = 1100
    HOLD_TIME = 2  # saniye
    for i in range(HOLD_TIME * 10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, START_PWM, 1500, 0, 0, 0, 0
        )
        time.sleep(0.1)

    # PWM'i yavaşça 1700'e çıkar
    for pwm in range(START_PWM, 1200, 10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, pwm, 1500, 0, 0, 0, 0
        )
        print(f"Motorlar şu an {pwm} PWM'de çalışıyor.")
        time.sleep(0.1)

    # PWM'i yavaşça 1100'e indir
    for pwm in range(1200, 1099, -10):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1500, pwm, 1500, 0, 0, 0, 0
        )
        print(f"Motorlar şu an {pwm} PWM'de çalışıyor.")
        time.sleep(0.1)

    # 6. Acil durdurma
    print("\n6. Acil durdurma...")
    acil_durdur(master)
    master.close()
    print("✅ Program sonlandı.")

except Exception as e:
    print(f"❌ Hata oluştu: {e}")
    try:
        if 'master' in locals():
            acil_durdur(master)
            master.close()
    except:
        print("Acil durdurma sırasında hata!") 