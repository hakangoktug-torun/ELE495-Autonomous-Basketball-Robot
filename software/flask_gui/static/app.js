const ASAMA_METINLERI = {
    beklemede: "Test baslamadi",
    baslatiliyor: "Test baslatiliyor...",
    atis: "Atis pozisyonunda - sweep yapiliyor",
    hareket: "Sonraki pozisyona hareket ediliyor",
    tamamlandi: "Test tamamlandi",
    durduruldu: "Durduruldu",
    hata: "Hata olustu",
};

const SAHA_BEKLEME_NOKTASI = {x: 12, y: 10};
const VARSAYILAN_SAHA_GENISLIK_CM = 120;
const VARSAYILAN_SAHA_UZUNLUK_CM = 80;
const VARSAYILAN_KALAN_SURE_SN = 300;
const DURUM_YENILEME_MS = 700;

const elements = {
    baglantiDurumu: document.getElementById("baglanti-durumu"),
    baglantiHata: document.getElementById("baglanti-hata"),
    baslatButton: document.getElementById("baslat-btn"),
    bolgeRozeti: document.getElementById("bolge-rozeti"),
    connection: document.getElementById("connection"),
    emergencyButton: document.getElementById("emergency"),
    escBilgisi: document.getElementById("esc-bilgisi"),
    escGonderButton: document.getElementById("esc-gonder-btn"),
    escGuncelDeger: document.getElementById("esc-guncel-deger"),
    escHizInput: document.getElementById("esc-hiz-input"),
    escPanel: document.getElementById("esc-panel"),
    gecisSayisi: document.getElementById("gecis-sayisi"),
    kalanSure: document.getElementById("kalan-sure"),
    modeBadge: document.getElementById("mode-badge"),
    modeText: document.getElementById("mode-text"),
    pozisyonAsama: document.getElementById("pozisyon-asama"),
    pozisyonEtiket: document.getElementById("pozisyon-etiket"),
    pozisyonPanel: document.getElementById("pozisyon-panel"),
    pozisyonSayac: document.getElementById("pozisyon-sayac"),
    robotMarker: document.getElementById("robot-marker"),
    sahaAsama: document.getElementById("saha-asama"),
    sahaHeading: document.getElementById("saha-heading"),
    sahaKonum: document.getElementById("saha-konum"),
    score: document.getElementById("score"),
    sureHucresi: document.getElementById("sure-hucresi"),
    taskHistory: document.getElementById("task-history"),
    testDurumu: document.getElementById("test-durumu"),
};

async function postJson(url, body = {}) {
    try {
        const response = await fetch(url, {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(body),
        });
        const payload = await response.json();

        if (!response.ok && payload.hata) {
            elements.baglantiHata.textContent = payload.hata;
            return;
        }

        renderStatus(payload);
    } catch (error) {
        setConnection(false);
        elements.baglantiHata.textContent = "Sunucudan gecerli cevap alinamadi.";
    }
}

async function refreshStatus() {
    try {
        const response = await fetch("/api/durum");
        renderStatus(await response.json());
        setConnection(true);
    } catch (error) {
        setConnection(false);
    }
}

function renderStatus(status) {
    elements.score.textContent = status.score;
    elements.baglantiDurumu.textContent = status.baglanti_hazir ? "hazir" : "bekleniyor";
    elements.testDurumu.textContent = status.baslatiliyor
        ? "baslatiliyor"
        : (status.calisiyor ? "calisiyor" : "beklemede");
    elements.gecisSayisi.textContent = status.gecis_sayisi != null ? status.gecis_sayisi : 0;
    elements.baglantiHata.textContent = status.baglanti_hata_mesaji || "";

    updateSureGostergesi(status);
    updateModeBadge(status);
    updateEscPanel(status.esc);
    updatePozisyonPanel(status.aktif_pozisyon);
    updateCourtPanel(status.aktif_pozisyon, status.telemetri);
    updateHistory(status.gecmis || []);
}

function formatSure(saniye) {
    const toplamSaniye = Math.max(0, Math.round(saniye));
    const dakika = Math.floor(toplamSaniye / 60);
    const kalanSaniye = toplamSaniye % 60;
    return `${dakika}:${String(kalanSaniye).padStart(2, "0")}`;
}

function numberOrNull(value) {
    if (value == null || value === "") {
        return null;
    }

    const numberValue = Number(value);
    return Number.isFinite(numberValue) ? numberValue : null;
}

function updateSureGostergesi(status) {
    const kalanSn = status.kalan_sure_sn != null
        ? status.kalan_sure_sn
        : VARSAYILAN_KALAN_SURE_SN;

    elements.kalanSure.textContent = formatSure(kalanSn);
    elements.sureHucresi.classList.remove("sure-uyari", "sure-kritik");

    if (kalanSn <= 30) {
        elements.sureHucresi.classList.add("sure-kritik");
    } else if (kalanSn <= 60) {
        elements.sureHucresi.classList.add("sure-uyari");
    }
}

function updatePozisyonPanel(pozisyon) {
    if (!pozisyon) {
        elements.pozisyonSayac.textContent = "-";
        elements.pozisyonEtiket.textContent = "Test baslamadi";
        elements.pozisyonAsama.textContent = "";
        elements.bolgeRozeti.hidden = true;
        elements.pozisyonPanel.className = "pozisyon-panel";
        return;
    }

    elements.pozisyonSayac.textContent = pozisyon.pozisyon_no
        ? `${pozisyon.pozisyon_no} / ${pozisyon.toplam_pozisyon || "?"}`
        : "-";
    elements.pozisyonEtiket.textContent = pozisyon.etiket || "-";
    elements.pozisyonAsama.textContent =
        ASAMA_METINLERI[pozisyon.asama] || pozisyon.asama || "";

    updateBolgeRozeti(pozisyon);
    updatePozisyonPanelClass(pozisyon.asama);
}

function updateBolgeRozeti(pozisyon) {
    if (!pozisyon.bolge) {
        elements.bolgeRozeti.hidden = true;
        return;
    }

    const puanMetni = pozisyon.puan != null ? ` (${pozisyon.puan} puan)` : "";
    elements.bolgeRozeti.hidden = false;
    elements.bolgeRozeti.textContent = pozisyon.bolge.split(" (")[0] + puanMetni;
    elements.bolgeRozeti.className = "bolge-rozeti";

    if (pozisyon.bolge.startsWith("KIRMIZI")) {
        elements.bolgeRozeti.classList.add("bolge-kirmizi");
    } else if (pozisyon.bolge.startsWith("YESIL")) {
        elements.bolgeRozeti.classList.add("bolge-yesil");
    } else {
        elements.bolgeRozeti.classList.add("bolge-bilinmiyor");
    }
}

function updatePozisyonPanelClass(asama) {
    elements.pozisyonPanel.className = "pozisyon-panel";

    if (asama === "atis") {
        elements.pozisyonPanel.classList.add("pozisyon-aktif");
    } else if (asama === "hareket") {
        elements.pozisyonPanel.classList.add("pozisyon-hareket");
    } else if (asama === "tamamlandi") {
        elements.pozisyonPanel.classList.add("pozisyon-tamamlandi");
    } else if (asama === "durduruldu" || asama === "hata") {
        elements.pozisyonPanel.classList.add("pozisyon-durduruldu");
    }
}

function updateCourtPanel(pozisyon, telemetri) {
    const sahaKonumu = getSahaKonumu(pozisyon);
    const heading = telemetri ? numberOrNull(telemetri.heading) : null;

    elements.robotMarker.style.left = `${sahaKonumu.yuzdeX}%`;
    elements.robotMarker.style.top = `${sahaKonumu.yuzdeY}%`;
    elements.robotMarker.style.transform = `translate(-50%, -50%) rotate(${heading || 0}deg)`;
    elements.robotMarker.classList.toggle("robot-active", Boolean(pozisyon && pozisyon.asama === "atis"));
    elements.robotMarker.classList.toggle("robot-moving", Boolean(pozisyon && pozisyon.asama === "hareket"));
    elements.robotMarker.classList.toggle(
        "robot-error",
        Boolean(pozisyon && (pozisyon.asama === "hata" || pozisyon.asama === "durduruldu")),
    );

    updateCourtText(pozisyon, sahaKonumu, heading);
}

function getSahaKonumu(pozisyon) {
    const sahaGenislik = pozisyon ? numberOrNull(pozisyon.saha_genislik_cm) : null;
    const sahaUzunluk = pozisyon ? numberOrNull(pozisyon.saha_uzunluk_cm) : null;
    const xCm = pozisyon ? numberOrNull(pozisyon.saha_x_cm) : null;
    const yCm = pozisyon ? numberOrNull(pozisyon.saha_y_cm) : null;

    if (xCm === null || yCm === null) {
        return {
            cmX: null,
            cmY: null,
            yuzdeX: SAHA_BEKLEME_NOKTASI.x,
            yuzdeY: SAHA_BEKLEME_NOKTASI.y,
        };
    }

    return {
        cmX: xCm,
        cmY: yCm,
        yuzdeX: (xCm / (sahaGenislik || VARSAYILAN_SAHA_GENISLIK_CM)) * 100,
        yuzdeY: (yCm / (sahaUzunluk || VARSAYILAN_SAHA_UZUNLUK_CM)) * 100,
    };
}

function updateCourtText(pozisyon, sahaKonumu, heading) {
    const etiket = pozisyon && pozisyon.etiket ? pozisyon.etiket : "baslangic";
    const asamaMetni = pozisyon
        ? (ASAMA_METINLERI[pozisyon.asama] || pozisyon.asama || "beklemede")
        : "beklemede";
    const headingText = heading !== null ? `${heading.toFixed(1)} derece` : "-";
    const koordinatText = sahaKonumu.cmX !== null && sahaKonumu.cmY !== null
        ? ` (${sahaKonumu.cmX.toFixed(1)}, ${sahaKonumu.cmY.toFixed(1)} cm)`
        : "";

    elements.sahaKonum.textContent = `Konum: ${etiket}${koordinatText}`;
    elements.sahaAsama.textContent = `Durum: ${asamaMetni}`;
    elements.sahaHeading.textContent = `Aci: ${headingText}`;
}

function updateEscPanel(esc) {
    if (!esc) {
        elements.escPanel.hidden = true;
        return;
    }

    elements.escPanel.hidden = false;

    if (esc.ilk_deger_bekleniyor) {
        elements.escBilgisi.textContent =
            `${esc.aktif_etiket || "Pozisyon"} - ESC hizi bekleniyor, lutfen deger girip gonder.`;
        elements.escPanel.classList.add("esc-bekleniyor");
    } else {
        elements.escBilgisi.textContent =
            "Sweep calisiyor - istedigin an yeni bir deger gonderip hizi degistirebilirsin.";
        elements.escPanel.classList.remove("esc-bekleniyor");
    }

    elements.escGuncelDeger.textContent = `Su anki ESC hizi: %${Number(esc.hiz).toFixed(1)}`;
}

function updateHistory(items) {
    elements.taskHistory.innerHTML = "";

    items.forEach((item) => {
        const row = document.createElement("li");
        const parts = item.split(" - ");
        const time = document.createElement("span");
        const message = document.createElement("span");

        time.className = "history-time";
        time.textContent = `[${parts[0]}]`;
        message.textContent = parts.slice(1).join(" - ") || item;
        row.append(time, message);
        elements.taskHistory.appendChild(row);
    });
}

function setConnection(isConnected) {
    elements.connection.textContent = isConnected ? "bagli" : "baglanti yok";
    elements.connection.className = isConnected
        ? "connection connected"
        : "connection disconnected";
}

function updateModeBadge(status) {
    elements.modeBadge.className = "mode-badge";

    if (status.baglanti_hata_mesaji) {
        elements.modeBadge.classList.add("mode-danger");
        elements.modeText.textContent = "HATA";
    } else if (status.baslatiliyor) {
        elements.modeBadge.classList.add("mode-waiting");
        elements.modeText.textContent = "BASLATILIYOR";
    } else if (status.esc && status.esc.ilk_deger_bekleniyor) {
        elements.modeBadge.classList.add("mode-waiting");
        elements.modeText.textContent = "ESC HIZI BEKLENIYOR";
    } else if (status.calisiyor) {
        elements.modeBadge.classList.add("mode-auto");
        elements.modeText.textContent = "CALISIYOR";
    } else if (status.baglanti_hazir) {
        elements.modeBadge.classList.add("mode-ready");
        elements.modeText.textContent = "HAZIR";
    } else {
        elements.modeText.textContent = "HAZIR DEGIL";
    }
}

elements.baslatButton.addEventListener("click", () => {
    postJson("/api/atis-testi/baslat");
});

elements.escGonderButton.addEventListener("click", () => {
    postJson("/api/esc-hiz", {hiz: Number(elements.escHizInput.value)});
});

elements.emergencyButton.addEventListener("click", () => {
    postJson("/api/emergency-stop");
});

refreshStatus();
setInterval(refreshStatus, DURUM_YENILEME_MS);
