"""ELE495 - Basketbol Robot Kontrol Paneli (Flask GUI).

Gercek donanim surumu. Ozel navigasyon + otomatik sweep atis dongusunu
arka planda calistirir. Interaktif ESC hizi gerekmez; pozisyonlarda rota
dosyasindaki sabit ESC hizlari kullanilir.
"""

from flask import Flask, request

try:
    from .api_helpers import skor_bildirimi_cevabi
    from .controller_common import (
        BaseAtisTestiKontrolcusu,
        ortak_route_kayitlarini_ekle,
        raspberry_pi_yollarini_hazirla,
        sigint_yakalayicisini_kur,
    )
except ImportError:
    from api_helpers import skor_bildirimi_cevabi
    from controller_common import (
        BaseAtisTestiKontrolcusu,
        ortak_route_kayitlarini_ekle,
        raspberry_pi_yollarini_hazirla,
        sigint_yakalayicisini_kur,
    )


raspberry_pi_yollarini_hazirla()
from skor_dinleyici import SkorDinleyici


app = Flask(__name__)


class AtisTestiKontrolcusu(BaseAtisTestiKontrolcusu):
    """Normal sweep test controller using fixed ESC speeds from the route file."""

    test_basladi_mesaji = "Ozel navigasyon testi (sweep) basladi."

    def __init__(self):
        super().__init__(SkorDinleyici)


kontrolcu = AtisTestiKontrolcusu()
ortak_route_kayitlarini_ekle(app, kontrolcu, skor_bildirimi_cevabi, request)


if __name__ == "__main__":
    try:
        sigint_yakalayicisini_kur(kontrolcu, "app.py")
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        kontrolcu.close()
