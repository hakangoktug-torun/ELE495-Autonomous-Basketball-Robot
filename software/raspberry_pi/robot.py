import time
from software.raspberry_pi.robot import RobotController, RobotMode

def run_autonomous_test(robot: RobotController):
    print("Otonom test basliyor... Hedef: Duvara 10 cm kala durmak.")
    
    # Robot modunu otonom yapıyoruz
    robot.mode = RobotMode.AUTONOMOUS
    
    try:
        while True:
            # Güncel sensör verilerini oku
            readings = robot.sensors.read()
            dist = readings.front_distance_cm
            
            print(f"Mesafe: {dist} cm | Ön Açık mı?: {readings.front_clear}")
            
            if not readings.front_clear:
                # Engel algılandı (Ultrasonik <= 10cm VEYA IR'lar tetiklendi)
                robot.drive.stop()
                print("!!! ENGELE ULASILDI: Robot durduruldu !!!")
                robot.mode = RobotMode.IDLE
                break
            else:
                # Önü temizse belirlediğin güvenli bir hızda düz git (%40 hız)
                # İleri gitmek için sol ve sağ motorlara pozitif hız verilir
                robot.drive.move(left_speed=0.4, right_speed=0.4)
                
            time.sleep(0.05) # 50ms bekle ve tekrar kontrol et
            
    except KeyboardInterrupt:
        # Ctrl+C ile test yarıda kesilirse motorları güvenli bir şekilde durdur
        robot.drive.stop()
        print("\nTest kullanıcı tarafından durduruldu.")