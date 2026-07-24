from flask import jsonify


GECERLI_SKOR_SENSORLERI = {"1", "2", "3", "4"}


def skor_bildirimi_cevabi(kontrolcu, sensor_no):
    if sensor_no not in GECERLI_SKOR_SENSORLERI:
        return jsonify({
            "ok": False,
            "sayildi": False,
            "hata": "Gecerli sensor numarasi bekleniyor: 1, 2, 3 veya 4.",
            "sensor": sensor_no,
        }), 400

    sayildi = kontrolcu.skor_dinleyici.gecis_bildir(sensor_no)
    return jsonify({
        "ok": True,
        "sayildi": sayildi,
        "sensor": sensor_no,
        "durum": kontrolcu.status_dict(),
    })
