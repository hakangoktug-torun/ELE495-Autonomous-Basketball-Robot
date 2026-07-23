"""
ELE495 - Skor Dinleyici
Break-beam sensorlerinden (Arduino R4 WiFi -> /skor HTTP GET) gelen top
gecisi bildirimlerini, o an calisan navigasyon testine (sweep dongusune)
thread-safe bicimde aktaran paylasilan durum nesnesi.

Iki farkli thread bu nesneyi kullanir:
  1) Flask'in istek isleyen thread'i - Arduino'dan istek geldiginde
     gecis_bildir() cagrilir.
  2) Navigasyon test kodunun calistigi arka plan thread'i - sweep dongusu
     icinde iki_gecis_oldu_mu() periyodik olarak kontrol edilir.

GUNCELLEME (COOLDOWN - top sekmesi telafisi): Bazen basariyla cemberden
gecen bir top, zeminden sekip potanin agzindan GERI CIKABILIYOR - bu da
break-beam sensorlerini ikinci kez tetikleyip AYNI atisin YANLISLIKLA 2
basarili gecis olarak sayilmasina (ve pozisyonun erken/hatali bitmesine)
yol aciyordu. Bunu onlemek icin: bir gecis sayildiktan HEMEN sonra
COOLDOWN_SURESI (1.5s) boyunca YENI tetiklemeler YOKSAYILIR - top sekip
geri cikarsa bu ikinci tetikleme sayilmaz.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve ozel_navigasyon_testi_esc_sweep_2.py
ile ayni yerde olmali)

Kullanim:
    skor_dinleyici = SkorDinleyici(puan_fn=..., olay_fn=...)

    # Bir atis pozisyonuna girerken:
    skor_dinleyici.saymaya_basla(puan=3)

    # Flask route icinde (Arduino'dan istek geldiginde):
    skor_dinleyici.gecis_bildir(sensor_no)

    # Sweep dongusu icinde:
    if skor_dinleyici.iki_gecis_oldu_mu():
        ...  # pozisyonu bitir

    # Cooldown, sweep bekleme suresinden CALMASIN diye - her kontrol
    # turunde, yeni baslamis bir cooldown olup olmadigini sor ve varsa
    # bekleme suresini o kadar UZAT:
    ek_sure = skor_dinleyici.cooldown_suresini_tuket()
    if ek_sure > 0:
        bitis_zamani += ek_sure

    # Pozisyon bitince (hareket/gecis sirasinda yanlissik sayilmasin diye):
    skor_dinleyici.saymayi_durdur()
"""

import threading
import time


class SkorDinleyici:
    # Basarili bir gecis sayildiktan sonra, YENI tetiklemelerin (muhtemel
    # top sekmesi) yoksayilacagi sure (saniye).
    COOLDOWN_SURESI = 1.5

    def __init__(self, puan_fn=None, olay_fn=None):
        self._lock = threading.Lock()
        self._sayma_aktif = False
        self._gecis_sayaci = 0
        self._aktif_puan = 0
        self._son_gecis_zamani = 0.0
        self._puan_fn = puan_fn
        self._olay_fn = olay_fn or (lambda mesaj: None)

        # Cooldown durumu
        self._cooldown_bitis = 0.0          # bu zamana kadar yeni tetiklemeler yoksayilir
        self._cooldown_tuketilmedi = False  # yeni baslayan cooldown, henuz sweep tarafindan "alinmadi"

    def saymaya_basla(self, puan=0):
        """Yeni bir atis pozisyonuna girerken cagrilir. Sayaci ve cooldown
        durumunu sifirlar, bu pozisyonun bolge puanini kaydeder - her
        basarili gecis bu kadar puan katacak."""
        with self._lock:
            self._sayma_aktif = True
            self._gecis_sayaci = 0
            self._aktif_puan = puan
            self._cooldown_bitis = 0.0
            self._cooldown_tuketilmedi = False

    def saymayi_durdur(self):
        """Pozisyon bitip robot bir sonrakine hareket ederken cagrilir -
        bu sirada olabilecek gecisler artik sayilmaz/puanlanmaz (orn.
        gecis sirasinda robot potanin onunden gecerken yanlis tetiklenme
        olmasin diye)."""
        with self._lock:
            self._sayma_aktif = False

    def gecis_bildir(self, sensor_no=None):
        """
        Arduino'dan /skor istegi geldiginde (Flask route icinden) cagrilir.
        Sayim aktifse VE cooldown aktif DEGILSE sayaci arttirir ve PUANI
        ANINDA ekler (2. gecis beklenmeden - boylece sweep erken kesilse
        bile kazanilan puan kaybolmaz), ardindan COOLDOWN_SURESI'lik bir
        cooldown baslatir.

        Cooldown aktifken gelen tetiklemeler (muhtemelen topun potadan
        sekip geri cikmasi) SESSIZCE YOKSAYILIR - ayni atis 2 kez sayilmaz.

        Donus: True (sayildi) / False (sayim aktif degildi YA DA cooldown
        aktifti - orn. pozisyonlar arasi gecis sirasinda gelen bir
        tetikleme, ya da topun sekmesi)
        """
        with self._lock:
            if not self._sayma_aktif:
                return False

            simdi = time.time()
            if simdi < self._cooldown_bitis:
                kalan = self._cooldown_bitis - simdi
                cooldown_aktifti = True
            else:
                cooldown_aktifti = False

            if cooldown_aktifti:
                sensor_str = sensor_no
                kalan_str = f"{kalan:.2f}"
            else:
                self._gecis_sayaci += 1
                sayac = self._gecis_sayaci
                puan = self._aktif_puan
                self._son_gecis_zamani = simdi
                self._cooldown_bitis = simdi + self.COOLDOWN_SURESI
                self._cooldown_tuketilmedi = True

        if cooldown_aktifti:
            self._olay_fn(f"(Cooldown aktif, {kalan_str}s kaldi - sensor {sensor_str} "
                          f"tetiklemesi sayilmadi, muhtemelen topun sekmesi)")
            return False

        self._olay_fn(f"TOP CEMBERDEN GECTI! (sensor {sensor_no}) - "
                      f"+{puan} puan (bu pozisyonda {sayac}. basarili gecis) - "
                      f"{self.COOLDOWN_SURESI}s cooldown basladi.")
        if self._puan_fn is not None and puan:
            self._puan_fn(puan)
        return True

    def cooldown_suresini_tuket(self):
        """
        Sweep bekleme dongusu (girdi_bekle) tarafindan HER kontrol turunde
        cagrilmasi beklenir. Eger bir onceki gecis_bildir() cagrisi YENI
        bir cooldown baslattiysa (ve bu henuz "tuketilmediyse"), o
        cooldown suresini (COOLDOWN_SURESI) DONER ve bayragi sifirlar -
        cagiran kod, sweep bekleme suresinin BITIS ZAMANINI bu kadar
        UZATARAK cooldown'un 10 saniyelik pencereden CALMAMASINI saglar.

        Ayni cooldown icin sadece BIR KEZ (ilk cagrida) sure doner - takip
        eden cagrilarda 0.0 doner, boylece ayni cooldown birden fazla kez
        sureyi uzatmaz.

        Donus: eklenecek sure (saniye, float) - cooldown yoksa 0.0
        """
        with self._lock:
            if self._cooldown_tuketilmedi:
                self._cooldown_tuketilmedi = False
                return self.COOLDOWN_SURESI
            return 0.0

    def gecis_sayisi(self):
        with self._lock:
            return self._gecis_sayaci

    def iki_gecis_oldu_mu(self):
        with self._lock:
            return self._gecis_sayaci >= 2
