import time
from pymavlink import mavutil

def check_arm_status(master):
    """ARM durumunu kontrol eder"""
    try:
        # ARM durumunu kontrol et
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1, 1
        )
        time.sleep(0.1)
        
        # Heartbeat mesajını al ve ARM durumunu kontrol et
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            print(f"System Status: {msg.system_status}")
            print(f"Base Mode: {msg.base_mode}")
            print(f"Custom Mode: {msg.custom_mode}")
            return True
    except Exception as e:
        print(f"ARM durumu kontrol edilirken hata: {e}")
    return False

def check_safety_switches(master):
    """Güvenlik anahtarlarını kontrol eder"""
    try:
        # Safety switch durumunu kontrol et
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1, 1
        )
        time.sleep(0.1)
        
        # SYS_STATUS mesajını al
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if msg:
            print(f"System Status: {msg.system_status}")
            print(f"Voltage: {msg.voltage_battery/1000.0:.2f}V")
            print(f"Current: {msg.current_battery/100.0:.2f}A")
            return True
    except Exception as e:
        print(f"Güvenlik anahtarları kontrol edilirken hata: {e}")
    return False

def force_disarm(master):
    """Zorla disarm yapar"""
    print("Zorla disarm yapılıyor...")
    for i in range(10):
        master.arducopter_disarm()
        time.sleep(0.1)
    print("Disarm tamamlandı.")

def test_motor(master, motor_id, throttle=20, duration=2):
    """Belirli bir motora PWM (%) ile throttle gönderir"""
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

try:
    print("=== DRONE MOTOR TEST DEBUG ===")
    
    # MAVLink bağlantısı başlat
    print("1. MAVLink bağlantısı kuruluyor...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print("✅ Heartbeat alındı.")
    
    # Sistem durumunu kontrol et
    print("\n2. Sistem durumu kontrol ediliyor...")
    check_arm_status(master)
    check_safety_switches(master)
    
    # Önce zorla disarm yap
    print("\n3. Önceki durumu temizleme...")
    force_disarm(master)
    time.sleep(2)
    
    # RC override'ı sıfırla
    print("\n4. RC override sıfırlanıyor...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)
    
    # ARM işlemi
    print("\n5. ARM işlemi başlatılıyor...")
    arm_result = master.arducopter_arm()
    print(f"ARM sonucu: {arm_result}")
    time.sleep(2)
    
    # ARM durumunu tekrar kontrol et
    print("\n6. ARM durumu kontrol ediliyor...")
    if check_arm_status(master):
        print("✅ ARM başarılı görünüyor.")
    else:
        print("❌ ARM başarısız olabilir.")
    
    # Motor testi
    print("\n7. Motor testi başlatılıyor...")
    MOTOR_ID = 1
    DURATION = 0.1
    
    # Yavaş yavaş artır: %15 → %20
    print("Motor hızı artırılıyor...")
    for throttle in range(15, 21, 1):
        test_motor(master, MOTOR_ID, throttle=throttle, duration=DURATION)
        time.sleep(DURATION)
    
    # Yavaş yavaş azalt: %20 → %15
    print("Motor hızı azaltılıyor...")
    for throttle in range(20, 14, -1):
        test_motor(master, MOTOR_ID, throttle=throttle, duration=DURATION)
        time.sleep(DURATION)
    
    # DISARM
    print("\n8. Disarm işlemi...")
    master.arducopter_disarm()
    print("🛑 Disarm edildi.")
    
    master.close()
    print("✅ Program sonlandı.")

except Exception as e:
    print(f"❌ Hata oluştu: {e}")
    try:
        if 'master' in locals():
            force_disarm(master)
            master.close()
    except:
        print("Acil durdurma sırasında hata!") 