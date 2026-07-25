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
COOLDOWN_SURESI (1s) boyunca YENI tetiklemeler YOKSAYILIR - top sekip
geri cikarsa bu ikinci tetikleme sayilmaz.

GUNCELLEME (IKINCI ATIS ICIN SINIRLI BEKLEME - kural degisikligi): Onceki
surumde bir pozisyon, 2. basarili gecis gelene kadar (sinirsiz sure,
sweep_bekleme dolana kadar) beklerdi. YARISMA KURALINA gore, bir noktadan
BASARILI bir atis sonrasi o noktadan sadece 1 ATIS DAHA yapilabilir -
BASARILI ya da BASARISIZ farketmeksizin. Break-beam sensorleri sadece
BASARILI gecisleri algilayabildigi icin (bir ISKA'yi dogrudan olcecek bir
sensorumuz yok), bu kural SUREYE dayanarak uygulaniyor: ilk basarili
gecisten SONRA en fazla IKINCI_ATIS_MAKS_BEKLEME_SN (5.0s) beklenir, bu
sure icinde 2. bir basarili gecis gelirse (ya da gelmese bile sure dolunca)
pozisyon OTOMATIK olarak bitirilir - boylece "sinirsiz deneme" yerine
kurala uygun sekilde "en fazla 1 ek deneme" hakki taniniyor.

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
    COOLDOWN_SURESI = 1.0

    # YENI: Bir pozisyonda ILK basarili gecis kaydedildikten sonra, bu
    # pozisyonun en fazla ne kadar daha bekleyebilecegi (saniye). Bu
    # sure dolunca - 2. bir basarili gecis gelse de gelmese de - pozisyon
    # otomatik olarak bitirilir (bkz. pozisyon_bitmeli_mi()). Yarisma
    # kuraline gore basarili bir atistan sonra sadece 1 ek deneme hakki
    # oldugu icin, bu sure o tek ek denemeye yetecek kadar kisa tutuluyor.
    IKINCI_ATIS_MAKS_BEKLEME_SN = 5.0

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

        # YENI: ilk basarili gecisin zamani - IKINCI_ATIS_MAKS_BEKLEME_SN
        # bu zamandan itibaren sayilir.
        self._ilk_basari_zamani = None

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
            self._ilk_basari_zamani = None

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
                if self._ilk_basari_zamani is None:
                    # YENI: bu pozisyondaki ILK basarili gecis - ikinci
                    # (son) deneme icin en fazla IKINCI_ATIS_MAKS_BEKLEME_SN
                    # kadar bir sayac baslatiliyor.
                    self._ilk_basari_zamani = simdi

        if cooldown_aktifti:
            self._olay_fn(f"(Cooldown aktif, {kalan_str}s kaldi - sensor {sensor_str} "
                          f"tetiklemesi sayilmadi, muhtemelen topun sekmesi)")
            return False

        if sayac == 1:
            self._olay_fn(f"TOP CEMBERDEN GECTI! (sensor {sensor_no}) - "
                          f"+{puan} puan (bu pozisyonda {sayac}. basarili gecis) - "
                          f"{self.COOLDOWN_SURESI}s cooldown basladi. Kural geregi "
                          f"bu pozisyondan en fazla {self.IKINCI_ATIS_MAKS_BEKLEME_SN}s "
                          f"daha (1 ek deneme icin) beklenecek.")
        else:
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
        """
        ESKI davranis (geriye donuk uyumluluk icin korunuyor) - SADECE
        2 basarili gecis olunca True doner, ilk basaridan sonraki 5s
        siniri BURADA UYGULANMAZ. Yeni kodda bunun yerine
        pozisyon_bitmeli_mi() kullanilmali.
        """
        with self._lock:
            return self._gecis_sayaci >= 2

    def pozisyon_bitmeli_mi(self):
        """
        YENI: Pozisyonun sweep'inin bitirilmesi gerekip gerekmedigini
        soyler. Iki durumda True doner:
          1) 2 basarili gecis olduysa (eskisi gibi, 2. gecis de basariliysa
             ANINDA biter).
          2) EN AZ 1 basarili gecis olduysa VE ilk basaridan bu yana
             IKINCI_ATIS_MAKS_BEKLEME_SN (5.0s) gectiyse - 2. deneme
             BASARISIZ olsa bile (ya da hic denenmese bile) kural geregi
             pozisyon biter.

        Sweep dongusu (girdi_bekle) bu metodu HER kontrol turunde cagirir.
        """
        with self._lock:
            if self._gecis_sayaci >= 2:
                return True
            if self._gecis_sayaci >= 1 and self._ilk_basari_zamani is not None:
                if time.time() - self._ilk_basari_zamani >= self.IKINCI_ATIS_MAKS_BEKLEME_SN:
                    return True
            return False
