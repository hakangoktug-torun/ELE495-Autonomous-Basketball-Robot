"""
ELE495 - HASSAS DONUS KONTROLU (BNO055 kapali dongu) - TEMIZ SURUM

donus_kapali_dongu.py + test_surus.py'deki donus mantiginin SADELESTIRILMIS
ve DUZELTILMIS hali. Amac: hedef aciyi TEK SEFERDE (+/- TOLERANS icinde)
tutturmak, tutturamazsa duzeltme atislariyla toparlamak.

ESKI KODDAKI IKI ANA HATA VE COZUMLERI:

  1) FREN_KATSAYISI = 2.1 s/(derece/s) MATEMATIKSEL OLARAK HATALIYDI:
     Normal donus hizi ~90-150 derece/s oldugu icin fren suresi hesabi
     2.1 * 100 = 210 saniye cikiyor, her seferinde 0.2s'lik TAVANA
     yapisiyordu. Yani fren hicbir zaman "hiza orantili" olmadi - HEP
     maksimum guctaydi. Robotu ters yone fazla gonderen buydu.
     COZUM: katsayi 0.0008'e cekildi (100 derece/s -> 80ms fren),
     tavan 0.10s'ye indirildi.

  2) DURMA KARARI GEC VERILIYORDU:
     "kalan <= TOLERANS" ani, sensor gecikmesi (I2C + Arduino dongusu +
     seri + Python thread, toplam ~80-120ms) yuzunden robot FIZIKSEL
     olarak hedefi coktan gecmisken algilaniyordu. 100 derece/s hizda
     100ms gecikme = 10 derece overshoot, daha motor durmadan.
     COZUM: durma karari artik OLCULEN anlik hiza gore ERKEN veriliyor:
     kalan <= hiz * GECIKME_TAHMINI oldugu anda motor kesiliyor.

  3) (YENI - BU SURUM) KUCUK HEDEFLERDE ASIRI DONME: bkz. asagidaki
     KUCUK ACI MODU notu. Gozlemlenen ornekler:
       - hedef 2 derece -> ana donus 13.6 derece (7 KAT fazla!)
       - hedef 6 derece -> ana donus 11.1 derece
     Bu, SWEEP ADIMLARININ HEPSINI (sweep_adim=2.0) ve kucuk gecis
     donuslerini (2, 5, 6, 10 derece gibi) etkiliyordu - duzeltme
     atislari sonunda toparliyordu ama MAKS_DUZELTME hakkinin cogunu/
     tamamini tuketerek, riskli bir sekilde.

KULLANIM (mevcut sweep kodunla uyumlu):
  ozel_navigasyon_testi_sweep.py icinde SADECE su satiri degistir:
      from test_surus import guvenli_donus, ...
  ->  from donus_hassas import guvenli_donus
      from test_surus import ileri_git_sabit_mesafe, ileri_git_engel_bulunca
  (guvenli_donus imzasi birebir ayni tutuldu.)

Tek basina test:
  python3 donus_hassas.py          # 90 sol + 90 sag + 2 derece (kucuk aci) test
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge

# ================= Motor pinleri (BCM) =================
IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16
SERIAL_PORT = "/dev/ttyUSB0"

# ================= Hiz ayarlari (mevcut 1.5x kalibrasyonun korundu) =================
HIZ_NORMAL = 45
HIZ_YAVAS = 33
HIZ_COK_YAVAS = 27

YAVASLAMA_ESIGI = 45.0       # kalan derece bu esigin altinda -> 1. kademe yavasla
COK_YAVAS_ESIGI = 12.0       # kalan derece bu esigin altinda -> 2. kademe yavasla
MINIMUM_HIZLI_SURE = 0.15    # s - yavaslamanin cok erken tetiklenip stall yaratmamasi icin

# ================= Hassasiyet ayarlari =================
TOLERANS = 0.5               # derece - bu hatanin altinda "tuttu" sayilir (1.5'ten
                               # dusuruldu, kullanici istegiyle: robot hedef aciyi
                               # olabildigince tam tutturmali). DAHA DA KUCULTME
                               # ONERILMEZ: log'larda gozlemlenen stick-slip davranisi
                               # (kucuk pulse'lar bazen HIC hareket ettirmiyor, sonra
                               # birikmis gerilim aniden 8-10 derecelik siramalarla
                               # bosaliyor) yuzunden asiri sikilastirmak sonsuz
                               # duzeltme dongusune/salinima yol acabilir.
MAKS_DUZELTME = 6            # normal (kucuk-aci MODU DISI) duzeltme hakki (5->6,
                               # daha sıkı TOLERANS'a yakinsamak icin fazladan 1 hak)
ZAMAN_ASIMI = 8.0            # s - sensor/motor arizasinda sonsuz donmeyi engeller

# --- Erken durma (gecikme telafisi) ---
GECIKME_TAHMINI = 0.05       # 0.04'ten 0.05'e çıkardım s - Hala FAZLA donuyorsa BUYUT, EKSIK kaliyorsa KUCULT

# --- Fren darbesi (duzeltilmis katsayi) ---
FREN_KATSAYISI = 0.0008      # s / (derece/s)  -> 100 derece/s hizda 80ms fren
FREN_MAKS_SURE = 0.10        # s - guvenlik tavani
FREN_MIN_HIZ = 30.0          # derece/saniye - bunun altinda fren atma (zaten duruyor)
FREN_DUTY = HIZ_YAVAS        # ters yonde uygulanan duty (tam hiz DEGIL)

# --- Hiz olcumu (EMA) ---
HIZ_EMA_ALPHA = 0.4
MAKS_GECERLI_HIZ = 300.0     # derece/saniye - tek gurultulu ornegin EMA'yi bozmamasi icin tavan

# --- Heading sicrama filtresi ---
MAKS_ANA_ADIM = 30.0         # derece - 20ms'de bundan buyugu gurultu sayilir
MAKS_SETTLE_ADIM = 8.0       # derece - motor kapaliyken bundan buyugu gurultu sayilir

# --- Settle (motor durduktan sonra sensorun oturmasi) ---
SETTLE_SURE = 1.0            # s - maksimum bekleme
SETTLE_SESSIZLIK = 0.4       # s - heading bu sure sabit kalirsa erken cik
SETTLE_EPSILON = 0.3         # derece

# --- Duzeltme atislari ---
DUZELTME_DUTY = 45           # dusuk duty kisa atista tekeri hic dondurmuyor -> tam hiz
DUZELTME_SANIYE_PER_DERECE = 1.0 / 150.0   # atis suresi = hata * bu katsayi
DUZELTME_MIN_SURE = 0.045    # s
DUZELTME_MIN_SURE_INCE = 0.048  # s - hata < 3 derece iken (daha nazik adim)
DUZELTME_MAKS_SURE = 0.09    # s - kisa tut: overshoot'u duzeltirken yeni overshoot yaratma
DUZELTME_SETTLE = 0.4        # s - her atis sonrasi olcum oncesi bekleme

# --- KUCUK ACI MODU (BUG DUZELTMESI - "hedeften COK fazla donme" sorunu) ---
KUCUK_ACI_ESIGI = 15.0        # derece - bu esigin ALTINDAKI/ESIT hedefler pulse-only modda donuyor
MAKS_DUZELTME_KUCUK = 10      # kucuk aci modunda pulse hakki (8->10, daha sıkı
                                # TOLERANS'a sifirdan pulse'larla ulasmak icin ek pay)


# =====================================================================
# Yardimcilar
# =====================================================================

def aci_farki(baslangic, bitis):
    """(-180, +180] araliginda sarmal-dogru aci farki."""
    fark = bitis - baslangic
    if fark < -180:
        fark += 360
    elif fark > 180:
        fark -= 360
    return fark


def _durdurma_istendi_mi(dur_bayragi):
    return dur_bayragi is not None and dur_bayragi.is_set()


def motorlari_ayarla():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4, ENA, ENB):
        GPIO.setup(p, GPIO.OUT)
    pwm_a = GPIO.PWM(ENA, 1000)
    pwm_b = GPIO.PWM(ENB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b


def motorlari_durdur(pwm_a, pwm_b):
    """Coast durdurma (aktif fren yok - o is fren DARBESI ile ayrica yapiliyor)."""
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.output(p, GPIO.LOW)


def donus_yonu_ayarla(yon):
    if yon == "sol":
        GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
    elif yon == "sag":
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    else:
        raise ValueError("yon 'sol' ya da 'sag' olmali")


class HeadingTakip:
    """
    Kumulatif donus takibi + sicrama filtresi.
    """

    def __init__(self, ilk_heading):
        self.onceki = ilk_heading
        self.bekleyen = None
        self.toplam = 0.0

    def guncelle(self, yeni, maks_adim):
        if self.onceki is None or yeni is None:
            return None
        adim = aci_farki(self.onceki, yeni)
        if abs(adim) <= maks_adim:
            self.onceki = yeni
            self.bekleyen = None
            self.toplam += adim
            return adim
        if self.bekleyen is not None and abs(aci_farki(self.bekleyen, yeni)) <= 5.0:
            self.onceki = yeni
            self.bekleyen = None
            self.toplam += adim
            return adim
        self.bekleyen = yeni
        return None


# =====================================================================
# Ana donus fonksiyonu
# =====================================================================

def hassas_donus(hedef_derece, yon, bridge, pwm_a, pwm_b, dur_bayragi=None):
    if _durdurma_istendi_mi(dur_bayragi):
        return 0.0

    bridge.request_fast_mode()
    time.sleep(0.1)

    ilk = bridge.get_heading()
    if ilk is None:
        print("HATA: heading okunamadi, donus iptal.")
        return 0.0

    takip = HeadingTakip(ilk)
    hedef_isaretli = -hedef_derece if yon == "sol" else hedef_derece

    donus_yonu_ayarla(yon)

    kucuk_aci_modu = hedef_derece <= KUCUK_ACI_ESIGI

    if kucuk_aci_modu:
        print(f"  [KUCUK ACI MODU] hedef={hedef_derece} derece (esik "
              f"{KUCUK_ACI_ESIGI} derecenin altinda/esit) - surekli tam "
              f"hizli surus atlaniyor, dogrudan kisa pulse'larla donulecek.")
    else:
        if hedef_derece > 45:
            for adim in range(1, 6):
                if _durdurma_istendi_mi(dur_bayragi):
                    motorlari_durdur(pwm_a, pwm_b)
                    return takip.toplam
                pwm_a.ChangeDutyCycle(HIZ_NORMAL * adim / 5)
                pwm_b.ChangeDutyCycle(HIZ_NORMAL * adim / 5)
                time.sleep(0.03)
        else:
            pwm_a.ChangeDutyCycle(HIZ_NORMAL)
            pwm_b.ChangeDutyCycle(HIZ_NORMAL)

        anlik_hiz = 0.0
        son_hiz_zamani = time.time()
        yavas1 = yavas2 = False
        baslangic = time.time()
        son_degisim_zamani = time.time()
        son_bilinen = ilk

        while True:
            if _durdurma_istendi_mi(dur_bayragi):
                break
            gecen = time.time() - baslangic
            if gecen > ZAMAN_ASIMI:
                print("UYARI: zaman asimi - donus zorla durduruldu.")
                break
            if bridge.is_stale(max_age_sec=0.5):
                print("UYARI: sensor akisi kesildi - guvenlik durusu.")
                break

            h = bridge.get_heading()

            if h is not None:
                if abs(aci_farki(son_bilinen, h)) > 0.3:
                    son_bilinen = h
                    son_degisim_zamani = time.time()
                elif gecen > 0.8 and time.time() - son_degisim_zamani > 0.8:
                    print("UYARI: BNO055 kilitlenmis gorunuyor - reset + iptal.")
                    motorlari_durdur(pwm_a, pwm_b)
                    bridge.request_heading_reset()
                    time.sleep(1.0)
                    return 0.0

            delta = takip.guncelle(h, MAKS_ANA_ADIM)
            if delta is not None:
                simdi = time.time()
                dt = simdi - son_hiz_zamani
                if dt > 0:
                    ornek = min(abs(delta) / dt, MAKS_GECERLI_HIZ)
                    anlik_hiz = HIZ_EMA_ALPHA * ornek + (1 - HIZ_EMA_ALPHA) * anlik_hiz
                son_hiz_zamani = simdi

            kalan = hedef_derece - abs(takip.toplam)

            durma_payi = max(TOLERANS, anlik_hiz * GECIKME_TAHMINI)
            if kalan <= durma_payi:
                motorlari_durdur(pwm_a, pwm_b)
                if anlik_hiz > FREN_MIN_HIZ:
                    fren = min(FREN_MAKS_SURE, anlik_hiz * FREN_KATSAYISI)
                    ters = "sag" if yon == "sol" else "sol"
                    print(f"  [DEBUG] Erken durma: kalan={kalan:.1f} "
                          f"hiz={anlik_hiz:.0f} d/s, fren={fren*1000:.0f}ms ({ters})")
                    donus_yonu_ayarla(ters)
                    pwm_a.ChangeDutyCycle(FREN_DUTY)
                    pwm_b.ChangeDutyCycle(FREN_DUTY)
                    time.sleep(fren)
                    motorlari_durdur(pwm_a, pwm_b)
                else:
                    print(f"  [DEBUG] Erken durma: kalan={kalan:.1f}, "
                          f"hiz dusuk ({anlik_hiz:.0f} d/s) - fren yok.")
                break

            if gecen > MINIMUM_HIZLI_SURE:
                if kalan <= COK_YAVAS_ESIGI and not yavas2:
                    pwm_a.ChangeDutyCycle(HIZ_COK_YAVAS)
                    pwm_b.ChangeDutyCycle(HIZ_COK_YAVAS)
                    yavas2 = yavas1 = True
                elif kalan <= YAVASLAMA_ESIGI and not yavas1:
                    pwm_a.ChangeDutyCycle(HIZ_YAVAS)
                    pwm_b.ChangeDutyCycle(HIZ_YAVAS)
                    yavas1 = True

            time.sleep(0.02)

        motorlari_durdur(pwm_a, pwm_b)
        if _durdurma_istendi_mi(dur_bayragi):
            return takip.toplam

        _settle_bekle(bridge, takip, dur_bayragi)

    hata = hedef_isaretli - takip.toplam
    print(f"Ana donus: {takip.toplam:.1f} derece (hata: {hata:+.1f})")

    maks_duzeltme_bu_sefer = MAKS_DUZELTME_KUCUK if kucuk_aci_modu else MAKS_DUZELTME

    for deneme in range(1, maks_duzeltme_bu_sefer + 1):
        if abs(hata) <= TOLERANS:
            break
        if _durdurma_istendi_mi(dur_bayragi):
            break

        atis_yonu = "sag" if hata > 0 else "sol"
        if abs(hata) < 3.0:
            sure = DUZELTME_MIN_SURE_INCE
        else:
            sure = min(DUZELTME_MAKS_SURE,
                       max(DUZELTME_MIN_SURE,
                           abs(hata) * DUZELTME_SANIYE_PER_DERECE))

        print(f"  Duzeltme #{deneme}/{maks_duzeltme_bu_sefer}: hata={hata:+.1f} derece, "
              f"yon={atis_yonu}, sure={sure*1000:.0f}ms")

        donus_yonu_ayarla(atis_yonu)
        pwm_a.ChangeDutyCycle(DUZELTME_DUTY)
        pwm_b.ChangeDutyCycle(DUZELTME_DUTY)
        time.sleep(sure)
        motorlari_durdur(pwm_a, pwm_b)
        time.sleep(DUZELTME_SETTLE)

        takip.guncelle(bridge.get_heading(), MAKS_SETTLE_ADIM)
        hata = hedef_isaretli - takip.toplam

    print(f"SONUC: hedef {hedef_derece} / gerceklesen {abs(takip.toplam):.1f} derece "
          f"(kalan hata: {abs(hata):.1f})")
    return takip.toplam


def _settle_bekle(bridge, takip, dur_bayragi):
    son_deger = takip.onceki
    son_degisim = time.time()
    baslangic = time.time()
    while time.time() - baslangic < SETTLE_SURE:
        if _durdurma_istendi_mi(dur_bayragi):
            return
        h = bridge.get_heading()
        takip.guncelle(h, MAKS_SETTLE_ADIM)
        if h is not None:
            if son_deger is None or abs(aci_farki(son_deger, h)) > SETTLE_EPSILON:
                son_deger = h
                son_degisim = time.time()
            elif time.time() - son_degisim > SETTLE_SESSIZLIK:
                return
        time.sleep(0.05)


# =====================================================================
# Mevcut sweep koduyla uyumluluk katmani
# =====================================================================

def guvenli_donus(hedef_derece, yon, bridge, pwm_a, pwm_b, maks_deneme=2,
                  dur_bayragi=None):
    for deneme in range(1, maks_deneme + 1):
        if _durdurma_istendi_mi(dur_bayragi):
            return False
        sonuc = hassas_donus(hedef_derece, yon, bridge, pwm_a, pwm_b,
                             dur_bayragi=dur_bayragi)
        if abs(sonuc) >= hedef_derece * 0.5:
            return True
        if _durdurma_istendi_mi(dur_bayragi):
            return False
        print(f"UYARI: donus basarisiz ({sonuc:.1f}/{hedef_derece}), "
              f"deneme {deneme}/{maks_deneme}")
        time.sleep(1.0)
    return False


# =====================================================================
# Tek basina test
# =====================================================================

def main():
    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()

    print("Sensor baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)
    if bridge.is_stale(max_age_sec=1.0):
        print("HATA: sensor verisi yok.")
        bridge.stop()
        return

    pwm_a, pwm_b = motorlari_ayarla()
    try:
        hassas_donus(90, "sol", bridge, pwm_a, pwm_b)
        time.sleep(1.0)
        hassas_donus(90, "sag", bridge, pwm_a, pwm_b)
        time.sleep(1.0)
        hassas_donus(2, "sag", bridge, pwm_a, pwm_b)
    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
