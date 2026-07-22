"""
ELE495 - ESC Hiz Kontrolcusu
GUI'nin, robot her atis pozisyonuna vardiginda ESC hizini SORMASINI ve
surec boyunca bu hizi CANLI olarak degistirebilmesini saglayan paylasilan
durum nesnesi.

Iki farkli thread bu nesneyi kullanir:
  1) Flask'in istek isleyen thread'i - GUI'den yeni bir hiz degeri
     geldiginde hiz_ayarla() cagrilir.
  2) Navigasyon test kodunun calistigi arka plan thread'i - yeni bir
     pozisyona varinca ilk_deger_bekle() ile GUI'den ilk degerin
     gelmesini bekler, sweep bekleme dongusu icinde de get() ile
     surekli guncel degeri okuyup ESC'ye uygular.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(skor_dinleyici.py, donus_kapali_dongu.py, robot_bridge.py ve
ozel_navigasyon_testi_esc_sweep_2.py ile ayni yerde olmali)

Kullanim:
    esc_hiz_kontrolcusu = EscHizKontrolcusu(olay_fn=...)

    # Arka plan thread'inde, yeni bir pozisyona varinca:
    ilk_hiz = esc_hiz_kontrolcusu.ilk_deger_bekle("1. atis", dur_bayragi=dur_bayragi)

    # Flask route icinde (GUI'den yeni deger geldiginde):
    esc_hiz_kontrolcusu.hiz_ayarla(deger)

    # Sweep bekleme dongusu icinde (canli guncelleme icin):
    guncel_hiz = esc_hiz_kontrolcusu.get()
"""

import threading


class EscHizKontrolcusu:
    def __init__(self, olay_fn=None):
        self._lock = threading.Lock()
        self._hiz = 0.0
        self._ilk_deger_bekleniyor = False
        self._ilk_deger_event = threading.Event()
        self._ilk_deger = None
        self._aktif_etiket = None
        self._olay_fn = olay_fn or (lambda mesaj: None)

    def get(self):
        """Su anki (en son GUI'den gelen ya da varsayilan) ESC hizini dondurur."""
        with self._lock:
            return self._hiz

    def hiz_ayarla(self, deger):
        """
        GUI'den (Flask route uzerinden) yeni bir ESC hizi geldiginde cagrilir.
        Eger o an bir pozisyonun ILK degeri bekleniyorsa (ilk_deger_bekle
        cagirilmis ve henuz donmemisse), bu deger o bekleyisi de tamamlar.
        """
        with self._lock:
            self._hiz = deger
            if self._ilk_deger_bekleniyor:
                self._ilk_deger = deger
                self._ilk_deger_bekleniyor = False
                self._ilk_deger_event.set()
        self._olay_fn(f"ESC hizi %{deger:.1f} olarak ayarlandi.")

    def ilk_deger_bekle(self, etiket, dur_bayragi=None):
        """
        Yeni bir atis pozisyonuna varildiginda cagrilir - GUI'den bu
        pozisyon icin ilk ESC hizi degeri gelene kadar BLOKE olur (bu
        bekleme sirasinda ESC 0'da/kapali kalir - cagiran kod bunu
        garanti eder).

        dur_bayragi verilirse (Acil Durdur/Ctrl+C), set edildiginde
        bekleme aninda iptal edilip None doner.

        Donus: GUI'den gelen ilk hiz degeri (float) / None (durdurma
        sinyali geldiyse)
        """
        with self._lock:
            self._aktif_etiket = etiket
            self._ilk_deger_bekleniyor = True
            self._ilk_deger = None
        self._ilk_deger_event.clear()
        self._olay_fn(f"{etiket}: ESC hizi bekleniyor (GUI'den gir)...")

        while True:
            if dur_bayragi is not None and dur_bayragi.is_set():
                with self._lock:
                    self._ilk_deger_bekleniyor = False
                    self._aktif_etiket = None
                return None
            if self._ilk_deger_event.wait(timeout=0.2):
                break

        with self._lock:
            deger = self._ilk_deger
            self._ilk_deger_bekleniyor = False
            self._aktif_etiket = None
        return deger

    def durum(self):
        """GUI'nin durumu gostermesi icin (bekleniyor mu, hangi pozisyon, guncel hiz)."""
        with self._lock:
            return {
                "hiz": self._hiz,
                "ilk_deger_bekleniyor": self._ilk_deger_bekleniyor,
                "aktif_etiket": self._aktif_etiket,
            }
