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

Kullanim:
    skor_dinleyici = SkorDinleyici(puan_fn=..., olay_fn=...)

    # Bir atis pozisyonuna girerken:
    skor_dinleyici.saymaya_basla(puan=3)

    # Flask route icinde (Arduino'dan istek geldiginde):
    skor_dinleyici.gecis_bildir(sensor_no)

    # Sweep dongusu icinde:
    if skor_dinleyici.iki_gecis_oldu_mu():
        ...  # pozisyonu bitir

    # Pozisyon bitince (hareket/gecis sirasinda yanlissik sayilmasin diye):
    skor_dinleyici.saymayi_durdur()
"""

import threading
import time


class SkorDinleyici:
    def __init__(self, puan_fn=None, olay_fn=None):
        self._lock = threading.Lock()
        self._sayma_aktif = False
        self._gecis_sayaci = 0
        self._aktif_puan = 0
        self._son_gecis_zamani = 0.0
        self._puan_fn = puan_fn
        self._olay_fn = olay_fn or (lambda mesaj: None)

    def saymaya_basla(self, puan=0):
        """Yeni bir atis pozisyonuna girerken cagrilir. Sayaci sifirlar ve
        bu pozisyonun bolge puanini kaydeder - her basarili gecis bu kadar
        puan katacak."""
        with self._lock:
            self._sayma_aktif = True
            self._gecis_sayaci = 0
            self._aktif_puan = puan

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
        Sayim aktifse sayaci arttirir ve PUANI ANINDA ekler (2. gecis
        beklenmeden - boylece sweep erken kesilse bile kazanilan puan
        kaybolmaz).

        Donus: True (sayildi) / False (sayim aktif degildi, yoksayildi -
        orn. pozisyonlar arasi gecis sirasinda gelen bir tetikleme)
        """
        with self._lock:
            if not self._sayma_aktif:
                return False
            self._gecis_sayaci += 1
            sayac = self._gecis_sayaci
            puan = self._aktif_puan
            self._son_gecis_zamani = time.time()

        self._olay_fn(f"TOP CEMBERDEN GECTI! (sensor {sensor_no}) - "
                      f"+{puan} puan (bu pozisyonda {sayac}. basarili gecis)")
        if self._puan_fn is not None and puan:
            self._puan_fn(puan)
        return True

    def gecis_sayisi(self):
        with self._lock:
            return self._gecis_sayaci

    def iki_gecis_oldu_mu(self):
        with self._lock:
            return self._gecis_sayaci >= 2
