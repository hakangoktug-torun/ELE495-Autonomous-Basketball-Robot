"""
ELE495 - Kucuk aci, SOL yon karsilastirma testi
nokta_a_test.py'deki 17.6 derece SAG donusu takildi (heading neredeyse hic
degismedi). Bu script AYNI aciyi SOL yonde dener - eger bu calisirsa,
sorunun yon-spesifik (muhtemelen mekanik asimetri) oldugu dogrulanir.

Kullanim:
    python3 kucuk_aci_sol_test.py
"""

from donus_kapali_dongu import donus_yap

if __name__ == "__main__":
    donus_yap(17.6, yon="sol")
