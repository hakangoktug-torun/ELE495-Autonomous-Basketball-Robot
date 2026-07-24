"""ELE495 - Basketbol Robot Kontrol Paneli (Flask GUI) - Interaktif ESC Surumu.

Normal Flask uygulamasiyla ayni sweep rotasini kullanir. Tek fark, robot
atis pozisyonuna geldiginde ESC hizinin GUI uzerinden girilebilmesidir.
"""

import math

from flask import Flask, jsonify, request

try:
    from .api_helpers import skor_bildirimi_cevabi
    from .controller_common import (
        BaseAtisTestiKontrolcusu,
        json_durum,
        ortak_route_kayitlarini_ekle,
        raspberry_pi_yollarini_hazirla,
        sigint_yakalayicisini_kur,
    )
except ImportError:
    from api_helpers import skor_bildirimi_cevabi
    from controller_common import (
        BaseAtisTestiKontrolcusu,
        json_durum,
        ortak_route_kayitlarini_ekle,
        raspberry_pi_yollarini_hazirla,
        sigint_yakalayicisini_kur,
    )


raspberry_pi_yollarini_hazirla()
from esc_hiz_kontrolcusu import EscHizKontrolcusu
from skor_dinleyici import SkorDinleyici


app = Flask(__name__)
MIN_ESC_HIZI = 0.0
MAKS_ESC_HIZI = 100.0


class AtisTestiKontrolcusu(BaseAtisTestiKontrolcusu):
    """Sweep test controller with GUI-driven ESC speed updates."""

    test_basladi_mesaji = "Ozel navigasyon testi (interaktif ESC) basladi."

    def __init__(self):
        super().__init__(SkorDinleyici)
        self.esc_hiz_kontrolcusu = EscHizKontrolcusu(olay_fn=self._gecmise_ekle)

    def rota_calisma_argumanlari(self):
        return {"esc_hiz_kontrolcusu": self.esc_hiz_kontrolcusu}

    def ek_status_alanlari(self):
        return {"esc": self.esc_hiz_kontrolcusu.durum()}


kontrolcu = AtisTestiKontrolcusu()
ortak_route_kayitlarini_ekle(app, kontrolcu, skor_bildirimi_cevabi, request)


@app.post("/api/esc-hiz")
def esc_hiz_ayarla():
    try:
        hiz = _esc_hizi_oku(request.get_json(silent=True) or {})
    except ValueError as exc:
        return jsonify({"hata": str(exc)}), 400

    kontrolcu.esc_hiz_kontrolcusu.hiz_ayarla(hiz)
    return json_durum(kontrolcu)


def _esc_hizi_oku(payload):
    try:
        hiz = float(payload.get("hiz"))
    except (TypeError, ValueError) as exc:
        raise ValueError("Gecerli bir 'hiz' sayisi gonder.") from exc

    if not math.isfinite(hiz):
        raise ValueError("ESC hizi sonlu bir sayi olmali.")
    if not MIN_ESC_HIZI <= hiz <= MAKS_ESC_HIZI:
        raise ValueError("ESC hizi 0 ile 100 arasinda olmali.")
    return hiz


if __name__ == "__main__":
    try:
        sigint_yakalayicisini_kur(kontrolcu, "app_esc_interaktif.py")
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        kontrolcu.close()
