#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, json, time, random, math
from collections import defaultdict

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QIcon
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QListWidget, QListWidgetItem,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QLineEdit, QTextEdit,
    QGroupBox, QCheckBox, QMessageBox, QInputDialog, QAction, QMenu, QFormLayout, 
    QSpinBox, QDoubleSpinBox, QGridLayout, QSlider, QDial, QTabWidget, QScrollArea
)
# Qt WebEngine may be unavailable on some systems; try import and provide
# a lightweight fallback so the app doesn't crash and can still run.
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWebChannel import QWebChannel
    WEBENGINE_AVAILABLE = True
    from PyQt5.QtCore import QObject, pyqtSlot
except Exception:
    WEBENGINE_AVAILABLE = False
    from PyQt5.QtCore import QObject, pyqtSlot

    # Fallback simple viewer that writes the HTML to a temp file and opens
    # it in the default web browser. Also exposes minimal methods used by
    # the rest of this module (setHtml, page().setWebChannel, runJavaScript).
    class QWebEngineView(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self._html = ""
            lay = QVBoxLayout(self)
            self._lbl = QLabel("Qt WebEngine bulunamadı. Harita önizlemesi devre dışı.")
            self._btn = QPushButton("Haritayı harici tarayıcıda aç")
            self._btn.clicked.connect(self.open_map_external)
            lay.addWidget(self._lbl)
            lay.addWidget(self._btn)

        def setHtml(self, html):
            self._html = html or ""

        def page(self):
            return self

        # stub for QWebChannel registration path
        def setWebChannel(self, channel):
            self._channel = channel

        # compatibility: called as self.map.page().setWebChannel(channel)
        def runJavaScript(self, *args, **kwargs):
            # No-op in fallback
            return None

        def open_map_external(self):
            import tempfile, webbrowser, os
            try:
                tf = tempfile.NamedTemporaryFile(delete=False, suffix='.html', mode='w', encoding='utf-8')
                tf.write(self._html)
                tf.close()
                webbrowser.open('file://' + os.path.abspath(tf.name))
            except Exception as e:
                QMessageBox.warning(self, 'Hata', f'Harita açılamadı: {e}')

    class QWebChannel:
        def __init__(self):
            pass

        def registerObject(self, name, obj):
            # no-op fallback
            return None

import serial
from serial.tools import list_ports

DARK_QSS = """
QMainWindow { background:#0e1116; color:#e6edf3; }
QGroupBox { border:1px solid #3a6ff0; border-radius:12px; margin-top:16px; color:#70a1ff; font-weight:700; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 8px; }
QLabel { color:#d0d7de; }
QListWidget { background:#0b0f14; color:#e6edf3; border:1px solid #202938; border-radius:10px; }
QTextEdit, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background:#0b0f14; color:#e6edf3; border:1px solid #202938; border-radius:10px; padding:8px; }
QPushButton { background:#345fff; color:#ffffff; border:none; padding:10px 14px; border-radius:10px; font-weight:700; }
QPushButton:hover { background:#4a72ff; }
QPushButton:disabled { background:#2a3550; color:#8b949e; }
QPushButton#headingBtn { background:#ff6b35; }
QPushButton#headingBtn:hover { background:#ff8555; }
QPushButton#headingBtn:checked { background:#00c896; }
QCheckBox { color:#e6edf3; }
QMenu { background:#0b0f14; color:#e6edf3; border:1px solid #202938; }
QMenu::item:selected { background:#345fff; }
QTabWidget::pane { border:1px solid #202938; background:#0b0f14; }
QTabBar::tab { background:#161b22; color:#8b949e; padding:8px 16px; }
QTabBar::tab:selected { background:#345fff; color:#ffffff; }
QSlider::groove:horizontal { background:#202938; height:6px; border-radius:3px; }
QSlider::handle:horizontal { background:#345fff; width:16px; height:16px; border-radius:8px; margin:-5px 0; }
QSlider::handle:horizontal:hover { background:#4a72ff; }
QDial { background:#161b22; }
/* Otonom Görev Dizisi için stiller */
QScrollArea {
    background-color: #0b0f14;
    border: 1px solid #202938;
    border-radius: 10px;
}

QScrollArea QWidget {
    background-color: #0b0f14;
}

QGroupBox {
    background-color: #0b0f14;
    color: #70a1ff;
    font-weight: 700;
    border: 1px solid #3a6ff0;
    border-radius: 12px;
    margin-top: 16px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 8px;
    background-color: #0b0f14;
}

QTextEdit#seq_preview {
    background: #0b0f14;
    color: #e6edf3;
    border: 1px solid #202938;
    border-radius: 10px;
    padding: 8px;
}
"""

# (ENHANCED_LEAFLET_HTML unchanged to keep map behavior identical)
ENHANCED_LEAFLET_HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="initial-scale=1, width=device-width"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
<style> 
    html, body, #map { height:100%; margin:0; } 
    .lbl { background:rgba(255,255,255,0.9); padding:2px 6px; border-radius:8px; border:1px solid #ddd; font-weight:700; color:#111; } 
    .rot { transform-origin:center; }
    .leaflet-control-layers { background: rgba(0,0,0,0.7); color: white; }
    .leaflet-control-layers-toggle { background-color: #333; }
    .leaflet-draw-toolbar a { background-color: #333; }
    
    /* Formasyon yönü okları için stil */
    .formation-arrow {
        stroke: #ff6b35;
        stroke-width: 3;
        fill: none;
        opacity: 0.8;
    }
    .formation-arrow-head {
        fill: #ff6b35;
        stroke: #ff6b35;
        stroke-width: 2;
    }
    .heading-label {
        background: rgba(255,107,53,0.9);
        color: white;
        padding: 4px 8px;
        border-radius: 12px;
        font-weight: bold;
        font-size: 14px;
    }
    .formation-preview {
        stroke: #00c896;
        stroke-width: 2;
        fill: rgba(0,200,150,0.2);
        stroke-dasharray: 5,5;
    }
    
    /* Kontrol paneli */
    .heading-control {
        position: absolute;
        top: 10px;
        left: 60px;
        z-index: 1000;
        background: rgba(0,0,0,0.8);
        padding: 10px;
        border-radius: 8px;
        color: white;
    }
    .heading-control button {
        background: #ff6b35;
        color: white;
        border: none;
        padding: 8px 16px;
        border-radius: 6px;
        cursor: pointer;
        margin: 2px;
    }
    .heading-control button:hover {
        background: #ff8555;
    }
    .heading-control button.active {
        background: #00c896;
    }
    .heading-value {
        display: inline-block;
        background: #345fff;
        padding: 4px 8px;
        border-radius: 4px;
        margin-left: 8px;
        min-width: 60px;
        text-align: center;
    }
</style>
</head>
<body>
<div id="map"></div>

<!-- Heading kontrol paneli -->
<div class="heading-control">
    <button id="drawHeadingBtn" onclick="toggleHeadingMode()">🧭 Formasyon Yönü Çiz</button>
    <button onclick="clearHeading()">🗑️ Temizle</button>
    <span class="heading-value" id="headingValue">0°</span>
</div>

<script>
var map = L.map('map', { zoomControl: true }).setView([39.0, 35.0], 6);

// Katmanlar
var osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 21, 
    attribution: '© OpenStreetMap'
}).addTo(map);

var esriLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 21,
    attribution: '© Esri'
});

var baseLayers = {
    "OpenStreetMap": osmLayer,
    "Uydu Görüntüsü": esriLayer
};

// Waypoint'leri saklamak için
var waypointsLayer = L.featureGroup().addTo(map);
var waypoints = [];
var routeLine = L.polyline([], {color: '#3498db', weight: 3}).addTo(map);

// Formasyon yönü için katman
var headingLayer = L.featureGroup().addTo(map);
var formationHeading = 0;
var headingArrow = null;
var isDrawingHeading = false;
var startPoint = null;

// Çizim araçları
var drawOptions = {
    position: 'topright',
    draw: {
        polygon: false,
        rectangle: false,
        circle: false,
        circlemarker: false,
        marker: {
            icon: L.divIcon({
                className: 'waypoint-marker',
                html: '<div style="background-color:#e74c3c;width:12px;height:12px;border-radius:50%;border:2px solid white;"></div>',
                iconSize: [16, 16],
                iconAnchor: [8, 8]
            })
        },
        polyline: false
    },
    edit: {
        featureGroup: waypointsLayer,
        remove: true
    }
};

// Draw kontrolünü başlat
map.addControl(new L.Control.Draw(drawOptions));

// Formasyon yönü çizme modu
function toggleHeadingMode() {
    isDrawingHeading = !isDrawingHeading;
    var btn = document.getElementById('drawHeadingBtn');
    
    if (isDrawingHeading) {
        // Diğer çizim araçlarını devre dışı bırak
        Object.keys(drawOptions.draw).forEach(function(type) {
            if (map.drawControl) {
                map.drawControl.disable();
            }
        });
        btn.classList.add('active');
    } else {
        // Çizim araçlarını tekrar etkinleştir
        if (map.drawControl) {
            map.drawControl.enable();
        }
        btn.classList.remove('active');
    }
    
    if (isDrawingHeading) {
        btn.classList.add('active');
        btn.textContent = '✓ Çizim Aktif';
        map.getContainer().style.cursor = 'crosshair';
        
        // Diğer çizim araçlarını devre dışı bırak
        drawControl.disable();
    } else {
        btn.classList.remove('active');
        btn.textContent = '🧭 Formasyon Yönü Çiz';
        map.getContainer().style.cursor = '';
        
        // Çizim araçlarını tekrar etkinleştir
        drawControl.enable();
    }
}

// Heading temizle
function clearHeading() {
    if (headingArrow) {
        headingLayer.removeLayer(headingArrow);
        headingArrow = null;
    }
    formationHeading = 0;
    updateHeadingDisplay(0);
    
    // Python'a bildir
    if (typeof qt !== 'undefined' && qt.updateHeading) {
        qt.updateHeading(0);
    }
}

// Heading değerini güncelle
function updateHeadingDisplay(deg) {
    document.getElementById('headingValue').textContent = Math.round(deg) + '°';
}

// Ok çizme fonksiyonu
function drawArrow(start, end) {
    // Önceki oku temizle
    if (headingArrow) {
        headingLayer.removeLayer(headingArrow);
    }
    
    // Heading hesapla
    var dx = end.lng - start.lng;
    var dy = end.lat - start.lat;
    var heading = Math.atan2(dx, dy) * 180 / Math.PI;
    if (heading < 0) heading += 360;
    
    formationHeading = heading;
    updateHeadingDisplay(heading);
    
    // Ok çiz
    var arrowPoints = [start, end];
    headingArrow = L.polyline(arrowPoints, {
        color: '#ff6b35',
        weight: 4,
        opacity: 0.8
    }).addTo(headingLayer);
    
    // Ok başı ekle
    var arrowHead = L.polylineDecorator(headingArrow, {
        patterns: [
            {
                offset: '100%',
                repeat: 0,
                symbol: L.Symbol.arrowHead({
                    pixelSize: 20,
                    polygon: true,
                    pathOptions: {
                        fillOpacity: 1,
                        weight: 0,
                        color: '#ff6b35'
                    }
                })
            }
        ]
    }).addTo(headingLayer);
    
    // Heading etiketi ekle
    var midPoint = L.latLng(
        (start.lat + end.lat) / 2,
        (start.lng + end.lng) / 2
    );
    
    L.marker(midPoint, {
        icon: L.divIcon({
            className: 'heading-label',
            html: '<div>' + Math.round(heading) + '°</div>',
            iconSize: [60, 30],
            iconAnchor: [30, 15]
        })
    }).addTo(headingLayer);
    
    // Python'a bildir
    if (typeof qt !== 'undefined' && qt.updateHeading) {
        qt.updateHeading(heading);
    }
}

// Harita tıklama olayları
map.on('mousedown', function(e) {
    if (isDrawingHeading) {
        startPoint = e.latlng;
    }
});

map.on('mouseup', function(e) {
    if (isDrawingHeading && startPoint) {
        var endPoint = e.latlng;
        
        // Minimum mesafe kontrolü
        var distance = startPoint.distanceTo(endPoint);
        if (distance > 100) { // 100 metre minimum
            drawArrow(startPoint, endPoint);
        }
        
        startPoint = null;
        toggleHeadingMode(); // Çizimi kapat
    }
});

// Fare hareketi ile önizleme
map.on('mousemove', function(e) {
    if (isDrawingHeading && startPoint) {
        // Geçici ok çiz (önizleme)
        // Bu özellik performans için basitleştirildi
    }
});

// Waypoint numaralandırma
function updateWaypointNumbers() {
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var num = waypoints.findIndex(wp => wp.lat === layer.getLatLng().lat && wp.lng === layer.getLatLng().lng) + 1;
            if (layer.getPopup()) {
                layer.setPopupContent("WP" + num);
            } else {
                layer.bindPopup("WP" + num).openPopup();
            }
        }
    });
    
    // Rota çizgisini güncelle
    var latlngs = waypoints.map(wp => [wp.lat, wp.lng]);
    routeLine.setLatLngs(latlngs);
}

// Çizim olayları
map.on(L.Draw.Event.CREATED, function (e) {
    var type = e.layerType,
        layer = e.layer;
    
    if (type === 'marker') {
        var latlng = layer.getLatLng();
        waypoints.push({lat: latlng.lat, lng: latlng.lng});
        waypointsLayer.addLayer(layer);
        updateWaypointNumbers();
    }
});

map.on(L.Draw.Event.EDITED, function (e) {
    waypoints = [];
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var latlng = layer.getLatLng();
            waypoints.push({lat: latlng.lat, lng: latlng.lng});
        }
    });
    updateWaypointNumbers();
});

map.on(L.Draw.Event.DELETED, function (e) {
    waypoints = [];
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var latlng = layer.getLatLng();
            waypoints.push({lat: latlng.lat, lng: latlng.lng});
        }
    });
    updateWaypointNumbers();
});

// Katman kontrolü
L.control.layers(baseLayers, {
    "Waypoints": waypointsLayer,
    "Rota": routeLine,
    "Formasyon Yönü": headingLayer
}, {position: 'topright'}).addTo(map);

// Drone fonksiyonları
var drones = {};

function makeIcon(isActive){
    return L.divIcon({
        className: 'drone-icon',
        html: '<svg class="rot" width="26" height="26" viewBox="0 0 100 100">' +
              '<polygon points="50,5 65,45 95,50 65,55 50,95 35,55 5,50 35,45" ' +
              'fill="' + (isActive ? '#00ff66' : '#ff4444') + '" stroke="#111" stroke-width="3"/>' +
              '</svg>',
        iconSize: [26,26],
        iconAnchor: [13,13]
    });
}

function addOrUpdateDrone(id, lat, lon, infoText, isActive, yawDeg) {
    var icon = makeIcon(isActive);
    if (!(id in drones)) {
        var m = L.marker([lat, lon], {icon: icon}).addTo(map);
        var lbl = L.marker([lat, lon], {
            icon: L.divIcon({className:'lbl', html: id, iconSize:[60,22], iconAnchor:[30, 30]})
        }).addTo(map);
        drones[id] = {marker:m, label:lbl, lastUpdate: Date.now(), yaw: yawDeg || 0};
    } else {
        drones[id].marker.setLatLng([lat, lon]);
        drones[id].label.setLatLng([lat, lon]);
        drones[id].marker.setIcon(icon);
        drones[id].lastUpdate = Date.now();
        drones[id].yaw = yawDeg || drones[id].yaw || 0;
    }
    if (infoText) { drones[id].marker.bindPopup(infoText); }
    var el = drones[id].marker.getElement();
    if (el){
        var svg = el.querySelector('.rot');
        if (svg){ svg.style.transform = 'rotate(' + (yawDeg||0) + 'deg)'; }
    }
}

function removeDrone(id) {
    if (drones[id]) { 
        map.removeLayer(drones[id].marker); 
        map.removeLayer(drones[id].label); 
        delete drones[id]; 
    }
}

// Waypoint'leri döndüren fonksiyon
function getWaypoints() {
    return JSON.stringify(waypoints.slice(0, 3));
}

// Heading değerini döndür
function getHeading() {
    return formationHeading;
}

// Heading'i manuel olarak ayarla
function setHeading(deg) {
    formationHeading = deg;
    updateHeadingDisplay(deg);
    // TODO: Oku yeniden çiz
}

// Tüm waypoint'leri temizle
function clearWaypoints() {
    waypoints = [];
    waypointsLayer.clearLayers();
    routeLine.setLatLngs([]);
}

// Leaflet.PolylineDecorator eklentisi basit versiyonu
L.PolylineDecorator = L.FeatureGroup.extend({
    initialize: function(paths, options) {
        L.FeatureGroup.prototype.initialize.call(this);
        this._paths = paths;
        this._patterns = options.patterns || [];
        this._drawPatterns();
    },
    
    _drawPatterns: function() {
        var paths = this._paths;
        if (paths._latlngs) {
            var latlngs = paths._latlngs;
            if (latlngs.length >= 2) {
                var last = latlngs[latlngs.length - 1];
                var prev = latlngs[latlngs.length - 2];
                
                // Ok başı üçgeni oluştur
                var angle = Math.atan2(last.lng - prev.lng, last.lat - prev.lat);
                var size = 0.0002;
                
                var arrowHead = [
                    [last.lat, last.lng],
                    [last.lat - size * Math.cos(angle - 2.5), last.lng - size * Math.sin(angle - 2.5)],
                    [last.lat - size * Math.cos(angle + 2.5), last.lng - size * Math.sin(angle + 2.5)]
                ];
                
                L.polygon(arrowHead, {
                    color: '#ff6b35',
                    fillColor: '#ff6b35',
                    fillOpacity: 1,
                    weight: 0
                }).addTo(this);
            }
        }
    }
});

L.polylineDecorator = function(paths, options) {
    return new L.PolylineDecorator(paths, options);
};

L.Symbol = {};
L.Symbol.arrowHead = function(options) {
    return {options: options};
};
</script>
</body>
</html>
"""
class MapBridge(QObject):
    """Harita ile Python arasında köprü"""
    headingChanged = pyqtSignal(float)
    
    @pyqtSlot(float)
    def updateHeading(self, heading):
        self.headingChanged.emit(heading)

class EnhancedGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLACKHEAD")
        self.resize(1600, 1000)  # Pencereyi biraz daha büyüttük
        self.setStyleSheet(DARK_QSS)

        self.vehicles = {}
        self.ser = None
        self.last_update = defaultdict(float)
        self.map_waypoints = []
        self.formation_heading = 0.0  # Formasyon yönü

        # Sıralı formasyon görev yönetimi
        self.seq_def_widgets = []  # her satır: dict of widgets
        self.sequence = []
        self.seq_index = -1
        self.seq_running = False
        self.seq_timer = QTimer(self)

        # Mission state management
        self.mission_running = False

        # Harita köprüsü
        self.map_bridge = MapBridge()
        self.map_bridge.headingChanged.connect(self.on_heading_changed)

        # Build UI
        self._build_ui()

        # Zamanlayıcılar
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.read_serial)
        self.rx_timer.start(100)

        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self.demo_tick)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.check_vehicle_status)
        self.status_timer.start(1000)

    def _build_ui(self):
        root = QSplitter(Qt.Horizontal)

        # Sol Panel - Araçlar
        left_box = QGroupBox("🚁 Araçlar")
        left_lay = QVBoxLayout()
        
        hl = QHBoxLayout()
        self.btn_add = QPushButton("➕ Drone Ekle")
        self.btn_add.clicked.connect(self.add_vehicle)
        self.btn_remove = QPushButton("➖ Drone Çıkar")
        self.btn_remove.clicked.connect(self.remove_vehicle)
        self.btn_remove.setEnabled(False)
        hl.addWidget(self.btn_add)
        hl.addWidget(self.btn_remove)

        left_lay.addLayout(hl)
        
        # Vehicle list widget
        self.list_vehicles = QListWidget()
        self.list_vehicles.currentItemChanged.connect(self.on_vehicle_select)
        self.list_vehicles.setContextMenuPolicy(Qt.CustomContextMenu)
        self.list_vehicles.customContextMenuRequested.connect(self.show_context_menu)
        left_lay.addWidget(self.list_vehicles)
        
        left_box.setLayout(left_lay)

        # Orta Panel - Harita
        center_box = QGroupBox("🗺️ Harita & Formasyon Yönü")
        center_lay = QVBoxLayout()
        
        self.map = QWebEngineView()
        self.map.setHtml(ENHANCED_LEAFLET_HTML)
        # Web channel kurulumu
        channel = QWebChannel()
        channel.registerObject('qt', self.map_bridge)
        self.map.page().setWebChannel(channel)
        
        center_lay.addWidget(self.map)
        center_box.setLayout(center_lay)
        center_box.setMaximumWidth(850)

        # Sağ Panel - Kontroller
        right_col = QWidget()
        rc = QVBoxLayout(right_col)

        # Tab widget
        tabs = QTabWidget()
        
        # Tab 1: Bağlantı & Görev
        tab1 = QWidget()
        tab1_lay = QVBoxLayout(tab1)
        
        # Serial bağlantı
        ser = QGroupBox("📡 ESP32 Mesh Bağlantı")
        slay = QVBoxLayout()
        
        row1 = QHBoxLayout()
        self.cmb_port = QComboBox()
        self.btn_refresh = QPushButton("🔄 Portları Tara")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        row1.addWidget(self.cmb_port)
        row1.addWidget(self.btn_refresh)
        
        row2 = QHBoxLayout()
        self.cmb_baud = QComboBox()
        self.cmb_baud.addItems(["9600","115200","57600","38400","19200"])
        self.cmb_baud.setCurrentText("115200")
        self.btn_connect = QPushButton("🔌 Bağlan")
        self.btn_connect.clicked.connect(self.toggle_serial)
        row2.addWidget(self.cmb_baud)
        row2.addWidget(self.btn_connect)
        
        self.chk_demo = QCheckBox("Demo Modu")
        self.chk_demo.stateChanged.connect(self.toggle_demo)
        slay.addWidget(self.chk_demo)
        
        slay.addLayout(row1)
        slay.addLayout(row2)
        ser.setLayout(slay)
        
        # Görev kontrol
        mission = QGroupBox("🎯 Görev Kontrolü")
        form = QFormLayout()
        
        self.sp_alt = QDoubleSpinBox()
        self.sp_alt.setRange(1, 200)
        self.sp_alt.setValue(10.0)
        self.sp_alt.setSuffix(" m")
        
        self.sp_yaw = QDoubleSpinBox()
        self.sp_yaw.setDecimals(1)
        self.sp_yaw.setRange(-180, 180)
        self.sp_yaw.setValue(0.0)
        self.sp_yaw.setSuffix(" °")
        
        self.sp_gs = QDoubleSpinBox()
        self.sp_gs.setDecimals(1)
        self.sp_gs.setRange(0.5, 20)
        self.sp_gs.setValue(3.0)
        self.sp_gs.setSuffix(" m/s")
        
        form.addRow(QLabel("🔺 Kalkış İrtifası:"), self.sp_alt)
        form.addRow(QLabel("🧭 Lider Yaw:"), self.sp_yaw)
        form.addRow(QLabel("💨 Yer Hızı:"), self.sp_gs)
        
        rowb = QHBoxLayout()
        self.btn_takeoff = QPushButton("🛫 ARM + TAKEOFF")
        self.btn_takeoff.clicked.connect(self.on_takeoff)
        self.btn_land = QPushButton("🛬 LAND")
        self.btn_land.clicked.connect(self.on_land)
        rowb.addWidget(self.btn_takeoff)
        rowb.addWidget(self.btn_land)
        
        mission.setLayout(form)
        mission.layout().addRow(rowb)
        
        tab1_lay.addWidget(ser)
        tab1_lay.addWidget(mission)
        tab1_lay.addStretch()



                # Tab 2'yi tamamen değiştiriyoruz - Satır 430'dan itibaren
        # Tab 2: Otonom Görev Dizisi
        tab2 = QWidget()
        tab2_lay = QVBoxLayout(tab2)

        seq_group = QGroupBox("🚁 Otonom Görev Dizisi")
        seq_layout = QVBoxLayout()

        # Görev önizleme
        self.seq_preview = QTextEdit()
        self.seq_preview.setObjectName("seq_preview")  # ensure QSS targets this widget
        self.seq_preview.setReadOnly(True)
        self.seq_preview.setMaximumHeight(150)
        self.seq_preview.setPlaceholderText("Görev adımları burada görünecek...")
        seq_layout.addWidget(self.seq_preview)

        # Görev adımları için scrollable alan
        scroll = QScrollArea()
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        # expose scroll widget/layout so add/remove handlers can modify it later
        self.mission_scroll_widget = scroll_widget
        self.mission_scroll_layout = scroll_layout

        self.mission_steps = []

        # İlk 5 adım için varsayılan satırlar
        for i in range(5):
            step_widget = self.create_mission_step_widget(i)
            self.mission_steps.append(step_widget)
            scroll_layout.addWidget(step_widget)

        scroll.setWidget(scroll_widget)
        scroll.setWidgetResizable(True)
        scroll.setMaximumHeight(400)
        seq_layout.addWidget(scroll)

        # Adım ekleme/çıkarma butonları
        step_buttons = QHBoxLayout()
        self.btn_add_step = QPushButton("➕ Adım Ekle")
        self.btn_add_step.clicked.connect(self.add_mission_step)
        self.btn_remove_step = QPushButton("➖ Son Adımı Sil")
        self.btn_remove_step.clicked.connect(self.remove_mission_step)
        step_buttons.addWidget(self.btn_add_step)
        step_buttons.addWidget(self.btn_remove_step)
        seq_layout.addLayout(step_buttons)

        # Görev başlatma butonu
        self.btn_execute_mission = QPushButton("🚀 GÖREVİ GÖNDER VE BAŞLAT")
        self.btn_execute_mission.clicked.connect(self.execute_autonomous_mission)
        self.btn_execute_mission.setStyleSheet("background: #00c896; font-size: 14px; padding: 15px;")
        seq_layout.addWidget(self.btn_execute_mission)

        seq_group.setLayout(seq_layout)
        tab2_lay.addWidget(seq_group)
        tab2_lay.addStretch()
                
                


        
        tabs.addTab(tab1, "🎮 Bağlantı")
        tabs.addTab(tab2, "📋 Görev Dizisi")
        
        # Tab 3: Navigasyon
        tab3 = QWidget()
        tab3_lay = QVBoxLayout(tab3)
        
        nav = QGroupBox("🗺️ Navigasyon (3 Waypoint)")
        gl = QGridLayout()
        
        self.wp = []
        for i in range(3):
            lat = QLineEdit()
            lon = QLineEdit()
            alt = QDoubleSpinBox()
            alt.setDecimals(1)
            alt.setRange(1, 400)
            alt.setValue(10.0)
            alt.setSuffix(" m")
            self.wp.append((lat, lon, alt))
            
            gl.addWidget(QLabel(f"📍 WP{i+1} Lat:"), i, 0)
            gl.addWidget(lat, i, 1)
            gl.addWidget(QLabel("Lon:"), i, 2)
            gl.addWidget(lon, i, 3)
            gl.addWidget(QLabel("Alt:"), i, 4)
            gl.addWidget(alt, i, 5)
        
        self.btn_read_wp = QPushButton("📥 Haritadan WP Oku")
        self.btn_read_wp.clicked.connect(self.read_waypoints_from_map)
        gl.addWidget(self.btn_read_wp, 3, 0, 1, 3)
        
        self.btn_nav = QPushButton("🚀 NAV GÖNDER")
        self.btn_nav.clicked.connect(self.on_nav_send)
        gl.addWidget(self.btn_nav, 3, 3, 1, 3)
        
        # --- Yeni: Navigasyon Formasyon Kontrolü ---
        nav_form_group = QGroupBox("🚁 Navigasyon Formasyonu")
        nav_form_layout = QFormLayout()
        
        # Formasyon tipi
        self.nav_form_type = QComboBox()
        self.nav_form_type.addItems(["V", "LINE", "ARROW"])
        nav_form_layout.addRow(QLabel("Formasyon Tipi:"), self.nav_form_type)
        
        # Formasyon parametreleri
        self.nav_form_spacing = QDoubleSpinBox()
        self.nav_form_spacing.setRange(1, 50)
        self.nav_form_spacing.setValue(8.0)
        self.nav_form_spacing.setSuffix(" m")
        nav_form_layout.addRow(QLabel("Mesafe:"), self.nav_form_spacing)
        
        self.nav_form_alt = QDoubleSpinBox()
        self.nav_form_alt.setRange(1, 400)
        self.nav_form_alt.setValue(10.0)
        self.nav_form_alt.setSuffix(" m")
        nav_form_layout.addRow(QLabel("Yükseklik:"), self.nav_form_alt)
        
        self.nav_form_gs = QDoubleSpinBox()
        self.nav_form_gs.setRange(0.5, 20)
        self.nav_form_gs.setValue(3.0)
        self.nav_form_gs.setSuffix(" m/s")
        nav_form_layout.addRow(QLabel("Hız:"), self.nav_form_gs)
        
        self.nav_form_duration = QDoubleSpinBox()
        self.nav_form_duration.setRange(1, 3600)
        self.nav_form_duration.setValue(30.0)
        self.nav_form_duration.setSuffix(" s")
        nav_form_layout.addRow(QLabel("Süre:"), self.nav_form_duration)
        
        self.nav_form_heading = QDoubleSpinBox()
        self.nav_form_heading.setRange(0, 360)
        self.nav_form_heading.setValue(0.0)
        self.nav_form_heading.setSuffix(" °")
        nav_form_layout.addRow(QLabel("Yön:"), self.nav_form_heading)
        
        self.nav_form_angle = QDoubleSpinBox()
        self.nav_form_angle.setRange(10, 120)
        self.nav_form_angle.setValue(60.0)
        self.nav_form_angle.setSuffix(" °")
        nav_form_layout.addRow(QLabel("Açı:"), self.nav_form_angle)
        
        # Formasyon gönder butonu
        self.btn_nav_formation = QPushButton("🚀 NAV + FORMASYON GÖNDER")
        self.btn_nav_formation.clicked.connect(self.on_nav_formation_send)
        nav_form_layout.addRow(self.btn_nav_formation)
        
        nav_form_group.setLayout(nav_form_layout)
        gl.addWidget(nav_form_group, 4, 0, 1, 6)
        
        nav.setLayout(gl)
        tab3_lay.addWidget(nav)
        tab3_lay.addStretch()
        
        tabs.addTab(tab3, "🗺️ Navigasyon")
        
        # Log - Büyük log ekranı
        msg_box = QGroupBox("📝 Log / Mesh Mesajları")
        mlay = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMinimumHeight(300)  # Log ekranını büyüttük
        mlay.addWidget(self.txt_log)
        msg_box.setLayout(mlay)

        rc.addWidget(tabs)
        rc.addWidget(msg_box)

        # Ana düzen
        root.addWidget(left_box)
        root.addWidget(center_box)
        root.addWidget(right_col)
        root.setSizes([250, 750, 600])
        
        self.setCentralWidget(root)
        self.refresh_ports()
        self.update_sequence_preview()

    def create_mission_step_widget(self, index):
        """Tek bir görev adımı için widget oluştur"""
    
        group = QGroupBox(f"Adım {index + 1}")
        layout = QGridLayout()
        
        # Adım tipi
        type_combo = QComboBox()
        type_combo.addItems(["TAKEOFF", "MOVE", "FORMATION", "WAIT", "LAND"])
        type_combo.currentTextChanged.connect(lambda: self.update_mission_preview())
        
        # Yön (direction) kontrolü - SADECE MOVE için
        direction_label = QLabel("Yön:")
        direction_combo = QComboBox()
        direction_combo.addItems(["GERİ", "İLERİ", "SOL", "SAĞ"])
        direction_combo.currentTextChanged.connect(lambda: self.update_mission_preview())
        
        # Heading (yön) kontrolü - FORMATION için
        heading_label = QLabel("Yaw:")
        heading_spin = QDoubleSpinBox()
        heading_spin.setRange(0, 360)
        heading_spin.setValue(0)
        heading_spin.setSuffix(" °")
        heading_spin.valueChanged.connect(lambda: self.update_mission_preview())
        
        # Mesafe
        distance_label = QLabel("Mesafe:")
        distance_spin = QDoubleSpinBox()
        distance_spin.setRange(0, 500)
        distance_spin.setValue(10)
        distance_spin.setSuffix(" m")
        distance_spin.valueChanged.connect(lambda: self.update_mission_preview())
        
        # Formasyon tipi
        formation_label = QLabel("Formasyon:")
        formation_combo = QComboBox()
        formation_combo.addItems(["V", "LINE", "ARROW", "TRIANGLE", "DIAMOND"])
        formation_combo.currentTextChanged.connect(lambda: self.update_mission_preview())
        
        # Formasyon parametreleri
        spacing_label = QLabel("Aralık:")
        spacing_spin = QDoubleSpinBox()
        spacing_spin.setRange(1, 50)
        spacing_spin.setValue(8)
        spacing_spin.setSuffix(" m")
        spacing_spin.valueChanged.connect(lambda: self.update_mission_preview())
        
        angle_label = QLabel("Açı:")
        angle_spin = QDoubleSpinBox()
        angle_spin.setRange(10, 120)
        angle_spin.setValue(60)
        angle_spin.setSuffix(" °")
        angle_spin.valueChanged.connect(lambda: self.update_mission_preview())
        
        # İrtifa
        alt_label = QLabel("İrtifa:")
        alt_spin = QDoubleSpinBox()
        alt_spin.setRange(1, 200)
        alt_spin.setValue(10)
        alt_spin.setSuffix(" m")
        alt_spin.valueChanged.connect(lambda: self.update_mission_preview())
        
        # Hız
        speed_label = QLabel("Hız:")
        speed_spin = QDoubleSpinBox()
        speed_spin.setRange(0.5, 20)
        speed_spin.setValue(3)
        speed_spin.setSuffix(" m/s")
        speed_spin.valueChanged.connect(lambda: self.update_mission_preview())

        # Bekleme süresi
        wait_label = QLabel("Bekleme:")
        wait_spin = QDoubleSpinBox()
        wait_spin.setRange(0, 300)
        wait_spin.setValue(5)
        wait_spin.setSuffix(" s")
        wait_spin.valueChanged.connect(lambda: self.update_mission_preview())

        # Formasyon yönü (heading) - görünür MOVE veya FORMATION için (ayrı bir kontrol)
        formation_heading_label = QLabel("Formasyon Yönü:")
        formation_heading_spin = QDoubleSpinBox()
        formation_heading_spin.setRange(0, 360)
        formation_heading_spin.setValue(0)
        formation_heading_spin.setSuffix(" °")
        formation_heading_spin.setToolTip("Formasyon yönü - 0° Kuzey, 90° Doğu, 180° Güney, 270° Batı")
        formation_heading_spin.valueChanged.connect(lambda: self.update_mission_preview())

        # Layout'a ekle
        layout.addWidget(QLabel("Tip:"), 0, 0)
        layout.addWidget(type_combo, 0, 1, 1, 3)

        layout.addWidget(direction_label, 1, 0)
        layout.addWidget(direction_combo, 1, 1)
        layout.addWidget(heading_label, 1, 2)
        layout.addWidget(heading_spin, 1, 3)

        layout.addWidget(distance_label, 2, 0)
        layout.addWidget(distance_spin, 2, 1)
        layout.addWidget(formation_label, 2, 2)
        layout.addWidget(formation_combo, 2, 3)
        # place formation heading to the right of formation controls
        layout.addWidget(formation_heading_label, 2, 4)
        layout.addWidget(formation_heading_spin, 2, 5)

        layout.addWidget(spacing_label, 3, 0)
        layout.addWidget(spacing_spin, 3, 1)
        layout.addWidget(angle_label, 3, 2)
        layout.addWidget(angle_spin, 3, 3)

        layout.addWidget(alt_label, 4, 0)
        layout.addWidget(alt_spin, 4, 1)
        layout.addWidget(speed_label, 4, 2)
        layout.addWidget(speed_spin, 4, 3)

        layout.addWidget(wait_label, 5, 0)
        layout.addWidget(wait_spin, 5, 1)

        group.setLayout(layout)

        # Widget referanslarını sakla
        group.widgets = {
            'type': type_combo,
            'direction': direction_combo,
            # 'heading' is the yaw control used for FORMATION
            'heading': heading_spin,
            # 'formation_heading' is the explicit formation heading for MOVE/FORMATION
            'formation_heading': formation_heading_spin,
            'distance': distance_spin,
            'formation': formation_combo,
            'spacing': spacing_spin,
            'angle': angle_spin,
            'altitude': alt_spin,
            'speed': speed_spin,
            'wait': wait_spin
        }

        # Tip değiştiğinde kontrolleri güncelle
        def update_controls():
            step_type = type_combo.currentText()
            is_move = (step_type == "MOVE")
            is_formation = (step_type == "FORMATION")
            is_takeoff = (step_type == "TAKEOFF")
            is_land = (step_type == "LAND")
            is_wait = (step_type == "WAIT")

            # Görünürlük ayarları
            direction_label.setVisible(is_move)
            direction_combo.setVisible(is_move)
            heading_label.setVisible(is_formation)
            heading_spin.setVisible(is_formation)
            # show the explicit formation heading for MOVE or FORMATION
            formation_heading_label.setVisible(is_move or is_formation)
            formation_heading_spin.setVisible(is_move or is_formation)

            distance_label.setVisible(is_move)
            distance_spin.setVisible(is_move)
            formation_label.setVisible(is_formation or is_move)
            formation_combo.setVisible(is_formation or is_move)
            spacing_label.setVisible(is_formation or is_move)
            spacing_spin.setVisible(is_formation or is_move)
            angle_label.setVisible(is_formation or is_move)
            angle_spin.setVisible(is_formation or is_move)
            alt_label.setVisible(is_takeoff or is_move or is_formation)
            alt_spin.setVisible(is_takeoff or is_move or is_formation)
            speed_label.setVisible(is_move or is_formation)
            speed_spin.setVisible(is_move or is_formation)
            wait_label.setVisible(step_type not in ["TAKEOFF", "LAND"])
            wait_spin.setVisible(step_type not in ["TAKEOFF", "LAND"])

        type_combo.currentTextChanged.connect(update_controls)
        update_controls()  # İlk durumu ayarla

        return group
    
    def execute_autonomous_mission(self):
        """Tüm görevi tek seferde gönder (toggle davranışı: çalışıyorsa durdur)."""

        # Eğer zaten çalışıyorsa, kullanıcının durdurma isteği
        if self.mission_running:
            self.log("[GÖREV] Kullanıcı görevi durdurdu.")
            self.stop_autonomous_mission()
            return

        # Görev listesi oluştur
        mission = {"type": "autonomous_mission", "steps": []}

        for i, step_widget in enumerate(self.mission_steps):
            widgets = step_widget.widgets
            step_type = widgets['type'].currentText()
            if step_type == "TAKEOFF":
                mission["steps"].append({"type": "TAKEOFF", "altitude": widgets['altitude'].value(), "wait": 5})
            elif step_type == "MOVE":
                direction = widgets['direction'].currentText()
                distance = widgets['distance'].value()
                # prefer explicit per-step formation_heading if present, otherwise fall back to heading
                if widgets.get('formation_heading') is not None:
                    formation_heading = widgets['formation_heading'].value()
                else:
                    formation_heading = widgets['heading'].value() if widgets.get('heading') is not None else 0

                # base movement vector (north, east)
                base_moves = {"GERİ": (1, 0), "İLERİ": (-1, 0), "SOL": (0, 1), "SAĞ": (0, -1)}
                base_north, base_east = base_moves.get(direction, (0, 0))

                # rotate base vector by formation heading
                heading_rad = math.radians(formation_heading)
                cos_h = math.cos(heading_rad)
                sin_h = math.sin(heading_rad)
                move_north = distance * (base_north * cos_h - base_east * sin_h)
                move_east = distance * (base_north * sin_h + base_east * cos_h)

                # clean tiny floating-rounding residues
                if abs(move_north) < 1e-10:
                    move_north = 0.0
                if abs(move_east) < 1e-10:
                    move_east = 0.0

                mission["steps"].append({
                    "type": "MOVE",
                    "north": move_north,
                    "east": move_east,
                    "altitude": widgets['altitude'].value(),
                    "speed": widgets['speed'].value(),
                    "formation": widgets['formation'].currentText(),
                    "spacing": widgets['spacing'].value(),
                    "angle": widgets['angle'].value(),
                    "formation_heading": formation_heading,
                    "wait": widgets['wait'].value()
                })
            elif step_type == "FORMATION":
                # prefer explicit per-step formation_heading if present (user-facing control)
                if widgets.get('formation_heading') is not None:
                    heading_val = widgets['formation_heading'].value()
                else:
                    heading_val = widgets['heading'].value()
                mission["steps"].append({
                    "type": "FORMATION",
                    "formation": widgets['formation'].currentText(),
                    "spacing": widgets['spacing'].value(),
                    "angle": widgets['angle'].value(),
                    "heading": heading_val,
                    "altitude": widgets['altitude'].value(),
                    "wait": widgets['wait'].value()
                })
            elif step_type == "WAIT":
                mission["steps"].append({"type": "WAIT", "duration": widgets['wait'].value()})
            elif step_type == "LAND":
                mission["steps"].append({"type": "LAND"})

        # Görevi gönder
        if self.send_json(mission):
            self.log("[GÖREV] Otonom görev gönderildi!")
            self.log(f"[GÖREV] {len(mission['steps'])} adım içeriyor")
            # Buton metnini değiştir ve durdurma handler'ını ayarla
            try:
                self.btn_execute_mission.setText("⏹️ GÖREVİ DURDUR")
                self.btn_execute_mission.setStyleSheet("background: #ff4757; font-size: 14px; padding: 15px;")
                self.btn_execute_mission.clicked.disconnect()
                self.btn_execute_mission.clicked.connect(self.stop_autonomous_mission)
            except Exception:
                pass
            self.mission_running = True

    def stop_autonomous_mission(self):
        """Stop a running autonomous mission: send LAND and reset UI/button state."""
        self.log("[GÖREV] Görev durduruluyor, LAND gönderiliyor.")
        try:
            self.send_json({"type": "LAND"})
        except Exception:
            pass
        self.mission_running = False
        try:
            self.btn_execute_mission.setText("▶️ GÖREVİ BAŞLAT")
            self.btn_execute_mission.setStyleSheet("")
            self.btn_execute_mission.clicked.disconnect()
            self.btn_execute_mission.clicked.connect(self.execute_autonomous_mission)
        except Exception:
            pass

    def update_step_controls(self, step_type, index):
        """Adım tipine göre kontrolleri güncelle"""
        w = self.seq_def_widgets[index]
        
        # Tüm kontrolleri önce etkinleştir
        for control in [w['spacing'], w['alt'], w['gs'], w['time'], w['heading'], w['angle']]:
            control.setEnabled(True)
        
        # Tip'e göre kontrolleri ayarla
        if step_type == "(Yok)":
            for control in [w['spacing'], w['alt'], w['gs'], w['time'], w['heading'], w['angle']]:
                control.setEnabled(False)
        elif step_type == "ARM+TAKEOFF":
            w['spacing'].setEnabled(False)
            w['angle'].setEnabled(False)
        elif step_type == "LAND":
            for control in [w['spacing'], w['angle'], w['heading']]:
                control.setEnabled(False)
        
        self.update_sequence_preview()

    def on_heading_changed(self, heading):
        """Haritadan heading değişimi geldiğinde"""
        self.formation_heading = heading
        self.log(f"[Harita] Formasyon yönü: {heading:.1f}°")

    def update_sequence_preview(self):
        lines = []
        for i, w in enumerate(self.seq_def_widgets):
            t = w['type'].currentText()
            if t == '(Yok)':
                lines.append(f"{i+1}. (Yok)")
                continue
            lines.append(f"{i+1}. {t} – dist:{w['spacing'].value():.1f}m alt:{w['alt'].value():.1f}m gs:{w['gs'].value():.1f}m/s dur:{w['time'].value():.1f}s head:{w['heading'].value():.1f}° angle:{w['angle'].value():.1f}°")
        self.seq_preview.setText("\n".join(lines))

    def update_mission_preview(self):
        """Render a human-readable preview of the mission steps into the preview text box."""
        lines = []
        for i, step_widget in enumerate(self.mission_steps):
            widgets = getattr(step_widget, 'widgets', {})
            t = widgets.get('type').currentText() if widgets.get('type') is not None else '(?)'
            if t == 'TAKEOFF':
                lines.append(f"{i+1}. TAKEOFF alt:{widgets['altitude'].value():.1f}m wait:{widgets['wait'].value():.1f}s")
            elif t == 'MOVE':
                lines.append(f"{i+1}. MOVE dir:{widgets['direction'].currentText()} dist:{widgets['distance'].value():.1f}m alt:{widgets['altitude'].value():.1f}m speed:{widgets['speed'].value():.1f}m/s")
            elif t == 'FORMATION':
                lines.append(f"{i+1}. FORMATION {widgets['formation'].currentText()} spacing:{widgets['spacing'].value():.1f}m angle:{widgets['angle'].value():.1f}° alt:{widgets['altitude'].value():.1f}m")
            elif t == 'WAIT':
                lines.append(f"{i+1}. WAIT {widgets['wait'].value():.1f}s")
            elif t == 'LAND':
                lines.append(f"{i+1}. LAND")
            else:
                lines.append(f"{i+1}. {t}")
        try:
            self.seq_preview.setText("\n".join(lines))
        except Exception:
            pass

    def add_mission_step(self):
        """Add a new mission step widget to the scroll area and internal list."""
        idx = len(self.mission_steps)
        step_widget = self.create_mission_step_widget(idx)
        self.mission_steps.append(step_widget)
        if hasattr(self, 'mission_scroll_layout') and self.mission_scroll_layout is not None:
            self.mission_scroll_layout.addWidget(step_widget)
        # Update titles (numbers) for clarity
        for i, w in enumerate(self.mission_steps):
            try:
                w.setTitle(f"Adım {i+1}")
            except Exception:
                pass
        self.update_mission_preview()

    def remove_mission_step(self):
        """Remove the last mission step widget if present."""
        if not self.mission_steps:
            return
        widget = self.mission_steps.pop()
        if hasattr(self, 'mission_scroll_layout') and self.mission_scroll_layout is not None:
            try:
                self.mission_scroll_layout.removeWidget(widget)
            except Exception:
                pass
        try:
            widget.setParent(None)
        except Exception:
            pass
        # Re-number remaining steps
        for i, w in enumerate(self.mission_steps):
            try:
                w.setTitle(f"Adım {i+1}")
            except Exception:
                pass
        self.update_mission_preview()

    def add_vehicle(self):
        vid, ok = QInputDialog.getText(self, "Yeni Drone", "Drone ID (örn: 0,1,2):")
        if ok and vid:
            vid = str(vid).strip()
            if vid in self.vehicles:
                QMessageBox.warning(self, "Uyarı", "Bu ID zaten mevcut.")
                return
            self.vehicles[vid] = {'id': vid, 'status':'offline', 'last_update':0}
            self.update_vehicle_list()
            self.map.page().runJavaScript(f"addOrUpdateDrone({json.dumps(vid)}, 39.0, 35.0, 'Yeni', false, 0);")

    def remove_vehicle(self):
        item = self.list_vehicles.currentItem()
        if not item: return
        vid = item.text()
        self.map.page().runJavaScript(f"removeDrone({json.dumps(vid)});")
        if vid in self.vehicles: del self.vehicles[vid]
        row = self.list_vehicles.row(item)
        self.list_vehicles.takeItem(row)

    def update_vehicle_list(self):
        self.list_vehicles.clear()
        for vid in sorted(self.vehicles.keys(), key=lambda x: str(x)):
            item = QListWidgetItem(vid)
            status = self.vehicles[vid].get('status','offline')
            if status == 'online': item.setForeground(QColor(0,255,170))
            elif status == 'warning': item.setForeground(QColor(255,200,0))
            else: item.setForeground(QColor(255,120,120))
            self.list_vehicles.addItem(item)

    def on_vehicle_select(self, cur, prev):
        self.btn_remove.setEnabled(bool(cur))

    def show_context_menu(self, position):
        item = self.list_vehicles.itemAt(position)
        if not item: return
        vid = item.text()
        menu = QMenu()
        focus = QAction("Haritada Odaklan", self)
        focus.triggered.connect(lambda: self.focus_on_vehicle(vid))
        info = QAction("Drone Bilgisi", self)
        info.triggered.connect(lambda: self.show_vehicle_info(vid))
        menu.addAction(focus)
        menu.addAction(info)
        menu.exec_(self.list_vehicles.viewport().mapToGlobal(position))

    def focus_on_vehicle(self, vid):
        v = self.vehicles.get(vid, {})
        lat, lon = v.get("lat"), v.get("lon")
        if lat is not None and lon is not None:
            self.map.page().runJavaScript(f"map.setView([{float(lat)}, {float(lon)}], 15);")

    def show_vehicle_info(self, vid):
        v = self.vehicles.get(vid, {})
        info = f"Drone ID: {vid}\nDurum: {v.get('status','bilinmiyor')}\n"
        info += f"Son Güncelleme: {time.strftime('%H:%M:%S', time.localtime(v.get('last_update',0)))}\n"
        QMessageBox.information(self, "Drone Bilgisi", info)

    def check_vehicle_status(self):
        now = time.time()
        for vid, data in self.vehicles.items():
            last = data.get('last_update', 0)
            if now - last > 10: data['status'] = 'offline'
            elif now - last > 5: data['status'] = 'warning'
            else: data['status'] = 'online'
        self.update_vehicle_list()

    def refresh_ports(self):
        self.cmb_port.clear()
        items = [p.device for p in list_ports.comports()]
        if not items: items = ["(Port yok)"]
        self.cmb_port.addItems(items)

    def toggle_serial(self):
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except: pass
            self.ser = None
            self.btn_connect.setText("🔌 Bağlan")
            self.log("[Seri] Kapandı.")
            return
        
        port = self.cmb_port.currentText().strip()
        if "(Port yok)" in port or not port:
            QMessageBox.warning(self, "Uyarı", "Kullanılabilir bir seri port seçin.")
            return
        
        baud = int(self.cmb_baud.currentText())
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
            self.btn_connect.setText("🔌 Kes")
            self.log(f"[Seri] Bağlandı: {port} @{baud}")
        except Exception as e:
            self.ser = None
            QMessageBox.critical(self, "Bağlantı Hatası", str(e))

    def _write_line(self, s: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((s + "\n").encode())
                self.log(f"> {s}")
                return True
            except Exception as e:
                self.log(f"[Gönderim hatası] {e}")
                return False
        elif self.chk_demo.isChecked():
            self.log(f"[Demo] Gönderildi: {s}")
            return True
        else:
            self.log("[Uyarı] Seri bağlı değil!")
            return False

    def send_json(self, payload: dict):
        try:
            line = json.dumps(payload, ensure_ascii=False)
        except Exception as e:
            self.log(f"[JSON hata] {e}")
            return False
        ok = self._write_line(line)
        if ok:
            self.log("[Kontrol] Komut gönderildi.")
        return ok

    def read_waypoints_from_map(self):
        """Haritadaki waypoint'leri oku"""
        self.map.page().runJavaScript("getWaypoints()", self._on_waypoints_received)

    def _on_waypoints_received(self, result):
        """Haritadan waypoint'ler alındığında"""
        try:
            waypoints = json.loads(result)
            self.map_waypoints = waypoints
            
            for i, (lat, lon, alt) in enumerate(self.wp):
                if i < len(waypoints):
                    wp = waypoints[i]
                    lat.setText(str(wp['lat']))
                    lon.setText(str(wp['lng']))
            
            self.log(f"[Harita] {len(waypoints)} waypoint alındı")
            
            if len(waypoints) > 3:
                QMessageBox.warning(self, "Uyarı", f"Haritada {len(waypoints)} waypoint bulundu. Sadece ilk 3'ü kullanılacak.")
                
        except Exception as e:
            self.log(f"[Hata] Waypoint okuma hatası: {e}")

    def on_takeoff(self):
        payload = {
            "type": "ui_takeoff",
            "alt": float(self.sp_alt.value()),
            "yaw_deg": float(self.sp_yaw.value()),
            "gs": float(self.sp_gs.value())
        }
        self.log("[Kontrol] ARM+TAKEOFF basıldı.")
        self.send_json(payload)

    def on_land(self):
        self.log("[Kontrol] LAND basıldı.")
        self.send_json({"type": "ui_land"})

    def on_nav_send(self):
        # Waypoint'leri al
        if len(self.map_waypoints) > 0:
            wps = []
            for i, wp in enumerate(self.map_waypoints[:3]):
                alt = self.wp[i][2].value() if i < len(self.wp) else 10.0
                wps.append({"lat": wp['lat'], "lon": wp['lng'], "alt": float(alt)})
        else:
            wps = []
            for i, (lat, lon, alt) in enumerate(self.wp):
                lt = lat.text().strip()
                ln = lon.text().strip()
                if lt and ln:
                    try:
                        wps.append({"lat": float(lt), "lon": float(ln), "alt": float(alt.value())})
                    except ValueError:
                        QMessageBox.warning(self, "WP Hatası", f"WP{i+1} lat/lon sayısal olmalı.")
                        return
            if not wps:
                QMessageBox.warning(self, "WP Hatası", "En az bir WP girin.")
                return

        self.log(f"[Kontrol] NAV GÖNDER - {len(wps)} waypoint")
        self.send_json({
            "type": "ui_nav",
            "wps": wps,
            "gs": float(self.sp_gs.value())
        })

    def on_nav_formation_send(self):
        """Navigasyon + Formasyon komutu gönder"""
        # Önce waypoint'leri al
        if len(self.map_waypoints) > 0:
            wps = []
            for i, wp in enumerate(self.map_waypoints[:3]):
                alt = self.wp[i][2].value() if i < len(self.wp) else 10.0
                wps.append({"lat": wp['lat'], "lon": wp['lng'], "alt": float(alt)})
        else:
            wps = []
            for i, (lat, lon, alt) in enumerate(self.wp):
                lt = lat.text().strip()
                ln = lon.text().strip()
                if lt and ln:
                    try:
                        wps.append({"lat": float(lt), "lon": float(ln), "alt": float(alt.value())})
                    except ValueError:
                        QMessageBox.warning(self, "WP Hatası", f"WP{i+1} lat/lon sayısal olmalı.")
                        return
            if not wps:
                QMessageBox.warning(self, "WP Hatası", "En az bir WP girin.")
                return

        # Formasyon parametrelerini al
        form_type = self.nav_form_type.currentText()
        spacing = float(self.nav_form_spacing.value())
        alt = float(self.nav_form_alt.value())
        gs = float(self.nav_form_gs.value())
        duration = float(self.nav_form_duration.value())
        heading = float(self.nav_form_heading.value())
        angle = float(self.nav_form_angle.value())

        self.log(f"[Kontrol] NAV + FORMASYON GÖNDER - {len(wps)} waypoint, {form_type} formasyonu")
        
        payload = {
            "type": "ui_nav_formation",
            "wps": wps,
            "gs": gs,
            "formation": {
                "name": form_type,
                "spacing_m": spacing,
                "heading_deg": heading,
                "alt": alt,
                "yaw_deg": heading,
                "duration_s": duration
            }
        }
        
        if form_type in ["V", "ARROW"]:
            payload["formation"]["angle_deg"] = angle

        self.send_json(payload)

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                data = self.ser.read(4096)
                if data:
                    for line in data.decode(errors="ignore").splitlines():
                        line = line.strip()
                        if not line: continue
                        self.log(f"< {line}")
                        self.process_line(line)
            except Exception as e:
                self.log(f"[Seri okuma hatası] {e}")

    def process_line(self, line: str):
        try:
            obj = json.loads(line)
            t = obj.get("type")
            if t == "state":
                did = str(obj.get("id", ""))
                if not did: return
                lat = obj.get("lat")
                lon = obj.get("lon")
                if lat is None or lon is None: return
                yaw = obj.get("yaw", 0.0)
                
                if did not in self.vehicles:
                    self.vehicles[did] = {'id': did}
                v = self.vehicles[did]
                v.update({"lat": lat, "lon": lon, "last_update": time.time()})
                
                is_active = (time.time() - v.get('last_update', 0)) < 5
                infoText = ""
                js = f"addOrUpdateDrone({json.dumps(did)}, {float(lat)}, {float(lon)}, {json.dumps(infoText)}, {str(is_active).lower()}, {float(yaw) if yaw is not None else 0});"
                self.map.page().runJavaScript(js)
                return
        except json.JSONDecodeError:
            pass

    def toggle_demo(self, state):
        if state == Qt.Checked:
            for i in range(3):
                did = str(i)
                if did not in self.vehicles:
                    self.vehicles[did] = {'id': did, 'last_update': time.time()}
            self.update_vehicle_list()
            self.demo_timer.start(800)
        else:
            self.demo_timer.stop()

    def demo_tick(self):
        for i in range(3):
            did = str(i)
            if did not in self.vehicles: continue
            base_lat, base_lon = 41.01 + i*0.005, 29.0 + i*0.005
            lat = base_lat + 0.001 * random.uniform(-1, 1)
            lon = base_lon + 0.001 * random.uniform(-1, 1)
            yaw = random.uniform(-180, 180)
            obj = {"type": "state", "id": did, "lat": lat, "lon": lon, "yaw": yaw}
            try:
                self.process_line(json.dumps(obj))
            except Exception:
                pass

    def log(self, text: str):
        self.txt_log.append(text)
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    # --- Sıralı görev fonksiyonları ---
    def build_sequence_from_ui(self):
        seq = []
        for w in self.seq_def_widgets:
            step_type = w['type'].currentText()
            if step_type == "(Yok)":
                continue
                
            entry = {
                'type': step_type,
                'spacing_m': float(w['spacing'].value()),
                'alt': float(w['alt'].value()),
                'gs': float(w['gs'].value()),
                'duration_s': float(w['time'].value()),
                'heading_deg': float(w['heading'].value()),
                'angle_deg': float(w['angle'].value())
            }
            seq.append(entry)
        return seq

    def start_sequence(self):
        if self.seq_running:
            QMessageBox.warning(self, "Uyarı", "Zaten bir görev çalışıyor.")
            return
        self.sequence = self.build_sequence_from_ui()
        if not self.sequence:
            QMessageBox.warning(self, "Uyarı", "Görev dizisi boş. En az bir adım seçin.")
            return
        
        # Disable controls to avoid mid-change
        self.btn_seq_start.setEnabled(False)
        self.btn_seq_abort.setEnabled(True)
        self.btn_takeoff.setEnabled(False)
        self.btn_land.setEnabled(False)
        self.seq_running = True
        self.seq_index = -1

        self.log("[Sıralı] Görev başlatılıyor...")
        self.next_sequence_step()

    def next_sequence_step(self):
        if not self.seq_running:
            return
        self.seq_index += 1
        
        # if finished
        if self.seq_index >= len(self.sequence):
            self.log("[Sıralı] Tüm adımlar tamamlandı.")
            self.seq_running = False
            self.btn_seq_start.setEnabled(True)
            self.btn_seq_abort.setEnabled(False)
            self.btn_takeoff.setEnabled(True)
            self.btn_land.setEnabled(True)
            return
            
        step = self.sequence[self.seq_index]
        step_type = step['type']

        if step_type == "ARM+TAKEOFF":
            payload = {
                "type": "ui_takeoff",
                "alt": step['alt'],
                "yaw_deg": step['heading_deg'],
                "gs": step['gs']
            }
            self.log(f"[Sıralı] ARM+TAKEOFF -> alt={step['alt']}m, yaw={step['heading_deg']}°, hız={step['gs']}m/s")
            self.send_json(payload)
            # TAKEOFF'tan sonra bir sonraki adıma geçmek için bekleme süresi
            QTimer.singleShot(int(step['duration_s'] * 1000), self.next_sequence_step)

        elif step_type == "LAND":
            payload = {"type": "ui_land"}
            self.log("[Sıralı] LAND")
            self.send_json(payload)
            # LAND'dan sonra görev biter
            self.seq_running = False
            self.btn_seq_start.setEnabled(True)
            self.btn_seq_abort.setEnabled(False)
            self.btn_takeoff.setEnabled(True)
            self.btn_land.setEnabled(True)

        else:
            # Formasyon adımları
            payload = {
                "type": "ui_formation",
                "name": step_type,
                "spacing_m": step['spacing_m'],
                "heading_deg": step['heading_deg'],
                "alt": step['alt'],
                "yaw_deg": step['heading_deg'],
                "gs": step['gs'],
                "dwell": step['duration_s']  # Dwell süresini gönderiyoruz ama leader kontrol edecek
            }
            if step_type in ["V", "ARROW"]:
                payload["angle_deg"] = step['angle_deg']

            self.log(f"[Sıralı] {step_type} Formasyon -> alt={step['alt']}m, hız={step['gs']}m/s, süre={step['duration_s']}s")
            self.send_json(payload)
            
            # Formasyon tamamlanma süresini bekle (leader kontrol ediyor)
            # Burada maksimum bekleme süresi hesaplayalım
            formation_timeout = max(30, step['duration_s'] + 60)  # En az 30s, duration + 60s buffer
            QTimer.singleShot(int(formation_timeout * 1000), self.next_sequence_step)

    def stop_sequence(self):
        if not self.seq_running:
            return
        self.log("[Sıralı] Görev iptal edildi. LAND gönderiliyor.")
        self.send_json({"type":"ui_land"})
        self.seq_running = False
        self.btn_seq_start.setEnabled(True)
        self.btn_seq_abort.setEnabled(False)
        self.btn_takeoff.setEnabled(True)
        self.btn_land.setEnabled(True)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    w = EnhancedGCS()
    w.show()
    sys.exit(app.exec_())#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, json, time, random, math
from collections import defaultdict

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QIcon
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QListWidget, QListWidgetItem,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QLineEdit, QTextEdit,
    QGroupBox, QCheckBox, QMessageBox, QInputDialog, QAction, QMenu, QFormLayout, 
    QSpinBox, QDoubleSpinBox, QGridLayout, QSlider, QDial, QTabWidget
)
# Qt WebEngine may be unavailable on some systems; try import and provide
# a lightweight fallback so the app doesn't crash and can still run.
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWebChannel import QWebChannel
    WEBENGINE_AVAILABLE = True
    from PyQt5.QtCore import QObject, pyqtSlot
except Exception:
    WEBENGINE_AVAILABLE = False
    from PyQt5.QtCore import QObject, pyqtSlot

    # Fallback simple viewer that writes the HTML to a temp file and opens
    # it in the default web browser. Also exposes minimal methods used by
    # the rest of this module (setHtml, page().setWebChannel, runJavaScript).
    class QWebEngineView(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self._html = ""
            lay = QVBoxLayout(self)
            self._lbl = QLabel("Qt WebEngine bulunamadı. Harita önizlemesi devre dışı.")
            self._btn = QPushButton("Haritayı harici tarayıcıda aç")
            self._btn.clicked.connect(self.open_map_external)
            lay.addWidget(self._lbl)
            lay.addWidget(self._btn)

        def setHtml(self, html):
            self._html = html or ""

        def page(self):
            return self

        # stub for QWebChannel registration path
        def setWebChannel(self, channel):
            self._channel = channel

        # compatibility: called as self.map.page().setWebChannel(channel)
        def runJavaScript(self, *args, **kwargs):
            # No-op in fallback
            return None

        def open_map_external(self):
            import tempfile, webbrowser, os
            try:
                tf = tempfile.NamedTemporaryFile(delete=False, suffix='.html', mode='w', encoding='utf-8')
                tf.write(self._html)
                tf.close()
                webbrowser.open('file://' + os.path.abspath(tf.name))
            except Exception as e:
                QMessageBox.warning(self, 'Hata', f'Harita açılamadı: {e}')

    class QWebChannel:
        def __init__(self):
            pass

        def registerObject(self, name, obj):
            # no-op fallback
            return None

import serial
from serial.tools import list_ports

DARK_QSS = """
QMainWindow { background:#0e1116; color:#e6edf3; }
QGroupBox { border:1px solid #3a6ff0; border-radius:12px; margin-top:16px; color:#70a1ff; font-weight:700; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 8px; }
QLabel { color:#d0d7de; }
QListWidget { background:#0b0f14; color:#e6edf3; border:1px solid #202938; border-radius:10px; }
QTextEdit, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background:#0b0f14; color:#e6edf3; border:1px solid #202938; border-radius:10px; padding:8px; }
QPushButton { background:#345fff; color:#ffffff; border:none; padding:10px 14px; border-radius:10px; font-weight:700; }
QPushButton:hover { background:#4a72ff; }
QPushButton:disabled { background:#2a3550; color:#8b949e; }
QPushButton#headingBtn { background:#ff6b35; }
QPushButton#headingBtn:hover { background:#ff8555; }
QPushButton#headingBtn:checked { background:#00c896; }
QCheckBox { color:#e6edf3; }
QMenu { background:#0b0f14; color:#e6edf3; border:1px solid #202938; }
QMenu::item:selected { background:#345fff; }
QTabWidget::pane { border:1px solid #202938; background:#0b0f14; }
QTabBar::tab { background:#161b22; color:#8b949e; padding:8px 16px; }
QTabBar::tab:selected { background:#345fff; color:#ffffff; }
QSlider::groove:horizontal { background:#202938; height:6px; border-radius:3px; }
QSlider::handle:horizontal { background:#345fff; width:16px; height:16px; border-radius:8px; margin:-5px 0; }
QSlider::handle:horizontal:hover { background:#4a72ff; }
QDial { background:#161b22; }
"""

# (ENHANCED_LEAFLET_HTML unchanged to keep map behavior identical)
ENHANCED_LEAFLET_HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="initial-scale=1, width=device-width"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
<style> 
    html, body, #map { height:100%; margin:0; } 
    .lbl { background:rgba(255,255,255,0.9); padding:2px 6px; border-radius:8px; border:1px solid #ddd; font-weight:700; color:#111; } 
    .rot { transform-origin:center; }
    .leaflet-control-layers { background: rgba(0,0,0,0.7); color: white; }
    .leaflet-control-layers-toggle { background-color: #333; }
    .leaflet-draw-toolbar a { background-color: #333; }
    
    /* Formasyon yönü okları için stil */
    .formation-arrow {
        stroke: #ff6b35;
        stroke-width: 3;
        fill: none;
        opacity: 0.8;
    }
    .formation-arrow-head {
        fill: #ff6b35;
        stroke: #ff6b35;
        stroke-width: 2;
    }
    .heading-label {
        background: rgba(255,107,53,0.9);
        color: white;
        padding: 4px 8px;
        border-radius: 12px;
        font-weight: bold;
        font-size: 14px;
    }
    .formation-preview {
        stroke: #00c896;
        stroke-width: 2;
        fill: rgba(0,200,150,0.2);
        stroke-dasharray: 5,5;
    }
    
    /* Kontrol paneli */
    .heading-control {
        position: absolute;
        top: 10px;
        left: 60px;
        z-index: 1000;
        background: rgba(0,0,0,0.8);
        padding: 10px;
        border-radius: 8px;
        color: white;
    }
    .heading-control button {
        background: #ff6b35;
        color: white;
        border: none;
        padding: 8px 16px;
        border-radius: 6px;
        cursor: pointer;
        margin: 2px;
    }
    .heading-control button:hover {
        background: #ff8555;
    }
    .heading-control button.active {
        background: #00c896;
    }
    .heading-value {
        display: inline-block;
        background: #345fff;
        padding: 4px 8px;
        border-radius: 4px;
        margin-left: 8px;
        min-width: 60px;
        text-align: center;
    }
</style>
</head>
<body>
<div id="map"></div>

<!-- Heading kontrol paneli -->
<div class="heading-control">
    <button id="drawHeadingBtn" onclick="toggleHeadingMode()">🧭 Formasyon Yönü Çiz</button>
    <button onclick="clearHeading()">🗑️ Temizle</button>
    <span class="heading-value" id="headingValue">0°</span>
</div>

<script>
var map = L.map('map', { zoomControl: true }).setView([39.0, 35.0], 6);

// Katmanlar
var osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 21, 
    attribution: '© OpenStreetMap'
}).addTo(map);

var esriLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 21,
    attribution: '© Esri'
});

var baseLayers = {
    "OpenStreetMap": osmLayer,
    "Uydu Görüntüsü": esriLayer
};

// Waypoint'leri saklamak için
var waypointsLayer = L.featureGroup().addTo(map);
var waypoints = [];
var routeLine = L.polyline([], {color: '#3498db', weight: 3}).addTo(map);

// Formasyon yönü için katman
var headingLayer = L.featureGroup().addTo(map);
var formationHeading = 0;
var headingArrow = null;
var isDrawingHeading = false;
var startPoint = null;

// Çizim araçları
var drawOptions = {
    position: 'topright',
    draw: {
        polygon: false,
        rectangle: false,
        circle: false,
        circlemarker: false,
        marker: {
            icon: L.divIcon({
                className: 'waypoint-marker',
                html: '<div style="background-color:#e74c3c;width:12px;height:12px;border-radius:50%;border:2px solid white;"></div>',
                iconSize: [16, 16],
                iconAnchor: [8, 8]
            })
        },
        polyline: false
    },
    edit: {
        featureGroup: waypointsLayer,
        remove: true
    }
};

// Draw kontrolünü başlat
map.addControl(new L.Control.Draw(drawOptions));

// Formasyon yönü çizme modu
function toggleHeadingMode() {
    isDrawingHeading = !isDrawingHeading;
    var btn = document.getElementById('drawHeadingBtn');
    
    if (isDrawingHeading) {
        // Diğer çizim araçlarını devre dışı bırak
        Object.keys(drawOptions.draw).forEach(function(type) {
            if (map.drawControl) {
                map.drawControl.disable();
            }
        });
        btn.classList.add('active');
    } else {
        // Çizim araçlarını tekrar etkinleştir
        if (map.drawControl) {
            map.drawControl.enable();
        }
        btn.classList.remove('active');
    }
    
    if (isDrawingHeading) {
        btn.classList.add('active');
        btn.textContent = '✓ Çizim Aktif';
        map.getContainer().style.cursor = 'crosshair';
        
        // Diğer çizim araçlarını devre dışı bırak
        drawControl.disable();
    } else {
        btn.classList.remove('active');
        btn.textContent = '🧭 Formasyon Yönü Çiz';
        map.getContainer().style.cursor = '';
        
        // Çizim araçlarını tekrar etkinleştir
        drawControl.enable();
    }
}

// Heading temizle
function clearHeading() {
    if (headingArrow) {
        headingLayer.removeLayer(headingArrow);
        headingArrow = null;
    }
    formationHeading = 0;
    updateHeadingDisplay(0);
    
    // Python'a bildir
    if (typeof qt !== 'undefined' && qt.updateHeading) {
        qt.updateHeading(0);
    }
}

// Heading değerini güncelle
function updateHeadingDisplay(deg) {
    document.getElementById('headingValue').textContent = Math.round(deg) + '°';
}

// Ok çizme fonksiyonu
function drawArrow(start, end) {
    // Önceki oku temizle
    if (headingArrow) {
        headingLayer.removeLayer(headingArrow);
    }
    
    // Heading hesapla
    var dx = end.lng - start.lng;
    var dy = end.lat - start.lat;
    var heading = Math.atan2(dx, dy) * 180 / Math.PI;
    if (heading < 0) heading += 360;
    
    formationHeading = heading;
    updateHeadingDisplay(heading);
    
    // Ok çiz
    var arrowPoints = [start, end];
    headingArrow = L.polyline(arrowPoints, {
        color: '#ff6b35',
        weight: 4,
        opacity: 0.8
    }).addTo(headingLayer);
    
    // Ok başı ekle
    var arrowHead = L.polylineDecorator(headingArrow, {
        patterns: [
            {
                offset: '100%',
                repeat: 0,
                symbol: L.Symbol.arrowHead({
                    pixelSize: 20,
                    polygon: true,
                    pathOptions: {
                        fillOpacity: 1,
                        weight: 0,
                        color: '#ff6b35'
                    }
                })
            }
        ]
    }).addTo(headingLayer);
    
    // Heading etiketi ekle
    var midPoint = L.latLng(
        (start.lat + end.lat) / 2,
        (start.lng + end.lng) / 2
    );
    
    L.marker(midPoint, {
        icon: L.divIcon({
            className: 'heading-label',
            html: '<div>' + Math.round(heading) + '°</div>',
            iconSize: [60, 30],
            iconAnchor: [30, 15]
        })
    }).addTo(headingLayer);
    
    // Python'a bildir
    if (typeof qt !== 'undefined' && qt.updateHeading) {
        qt.updateHeading(heading);
    }
}

// Harita tıklama olayları
map.on('mousedown', function(e) {
    if (isDrawingHeading) {
        startPoint = e.latlng;
    }
});

map.on('mouseup', function(e) {
    if (isDrawingHeading && startPoint) {
        var endPoint = e.latlng;
        
        // Minimum mesafe kontrolü
        var distance = startPoint.distanceTo(endPoint);
        if (distance > 100) { // 100 metre minimum
            drawArrow(startPoint, endPoint);
        }
        
        startPoint = null;
        toggleHeadingMode(); // Çizimi kapat
    }
});

// Fare hareketi ile önizleme
map.on('mousemove', function(e) {
    if (isDrawingHeading && startPoint) {
        // Geçici ok çiz (önizleme)
        // Bu özellik performans için basitleştirildi
    }
});

// Waypoint numaralandırma
function updateWaypointNumbers() {
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var num = waypoints.findIndex(wp => wp.lat === layer.getLatLng().lat && wp.lng === layer.getLatLng().lng) + 1;
            if (layer.getPopup()) {
                layer.setPopupContent("WP" + num);
            } else {
                layer.bindPopup("WP" + num).openPopup();
            }
        }
    });
    
    // Rota çizgisini güncelle
    var latlngs = waypoints.map(wp => [wp.lat, wp.lng]);
    routeLine.setLatLngs(latlngs);
}

// Çizim olayları
map.on(L.Draw.Event.CREATED, function (e) {
    var type = e.layerType,
        layer = e.layer;
    
    if (type === 'marker') {
        var latlng = layer.getLatLng();
        waypoints.push({lat: latlng.lat, lng: latlng.lng});
        waypointsLayer.addLayer(layer);
        updateWaypointNumbers();
    }
});

map.on(L.Draw.Event.EDITED, function (e) {
    waypoints = [];
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var latlng = layer.getLatLng();
            waypoints.push({lat: latlng.lat, lng: latlng.lng});
        }
    });
    updateWaypointNumbers();
});

map.on(L.Draw.Event.DELETED, function (e) {
    waypoints = [];
    waypointsLayer.eachLayer(function(layer) {
        if (layer instanceof L.Marker) {
            var latlng = layer.getLatLng();
            waypoints.push({lat: latlng.lat, lng: latlng.lng});
        }
    });
    updateWaypointNumbers();
});

// Katman kontrolü
L.control.layers(baseLayers, {
    "Waypoints": waypointsLayer,
    "Rota": routeLine,
    "Formasyon Yönü": headingLayer
}, {position: 'topright'}).addTo(map);

// Drone fonksiyonları
var drones = {};

function makeIcon(isActive){
    return L.divIcon({
        className: 'drone-icon',
        html: '<svg class="rot" width="26" height="26" viewBox="0 0 100 100">' +
              '<polygon points="50,5 65,45 95,50 65,55 50,95 35,55 5,50 35,45" ' +
              'fill="' + (isActive ? '#00ff66' : '#ff4444') + '" stroke="#111" stroke-width="3"/>' +
              '</svg>',
        iconSize: [26,26],
        iconAnchor: [13,13]
    });
}

function addOrUpdateDrone(id, lat, lon, infoText, isActive, yawDeg) {
    var icon = makeIcon(isActive);
    if (!(id in drones)) {
        var m = L.marker([lat, lon], {icon: icon}).addTo(map);
        var lbl = L.marker([lat, lon], {
            icon: L.divIcon({className:'lbl', html: id, iconSize:[60,22], iconAnchor:[30, 30]})
        }).addTo(map);
        drones[id] = {marker:m, label:lbl, lastUpdate: Date.now(), yaw: yawDeg || 0};
    } else {
        drones[id].marker.setLatLng([lat, lon]);
        drones[id].label.setLatLng([lat, lon]);
        drones[id].marker.setIcon(icon);
        drones[id].lastUpdate = Date.now();
        drones[id].yaw = yawDeg || drones[id].yaw || 0;
    }
    if (infoText) { drones[id].marker.bindPopup(infoText); }
    var el = drones[id].marker.getElement();
    if (el){
        var svg = el.querySelector('.rot');
        if (svg){ svg.style.transform = 'rotate(' + (yawDeg||0) + 'deg)'; }
    }
}

function removeDrone(id) {
    if (drones[id]) { 
        map.removeLayer(drones[id].marker); 
        map.removeLayer(drones[id].label); 
        delete drones[id]; 
    }
}

// Waypoint'leri döndüren fonksiyon
function getWaypoints() {
    return JSON.stringify(waypoints.slice(0, 3));
}

// Heading değerini döndür
function getHeading() {
    return formationHeading;
}

// Heading'i manuel olarak ayarla
function setHeading(deg) {
    formationHeading = deg;
    updateHeadingDisplay(deg);
    // TODO: Oku yeniden çiz
}

// Tüm waypoint'leri temizle
function clearWaypoints() {
    waypoints = [];
    waypointsLayer.clearLayers();
    routeLine.setLatLngs([]);
}

// Leaflet.PolylineDecorator eklentisi basit versiyonu
L.PolylineDecorator = L.FeatureGroup.extend({
    initialize: function(paths, options) {
        L.FeatureGroup.prototype.initialize.call(this);
        this._paths = paths;
        this._patterns = options.patterns || [];
        this._drawPatterns();
    },
    
    _drawPatterns: function() {
        var paths = this._paths;
        if (paths._latlngs) {
            var latlngs = paths._latlngs;
            if (latlngs.length >= 2) {
                var last = latlngs[latlngs.length - 1];
                var prev = latlngs[latlngs.length - 2];
                
                // Ok başı üçgeni oluştur
                var angle = Math.atan2(last.lng - prev.lng, last.lat - prev.lat);
                var size = 0.0002;
                
                var arrowHead = [
                    [last.lat, last.lng],
                    [last.lat - size * Math.cos(angle - 2.5), last.lng - size * Math.sin(angle - 2.5)],
                    [last.lat - size * Math.cos(angle + 2.5), last.lng - size * Math.sin(angle + 2.5)]
                ];
                
                L.polygon(arrowHead, {
                    color: '#ff6b35',
                    fillColor: '#ff6b35',
                    fillOpacity: 1,
                    weight: 0
                }).addTo(this);
            }
        }
    }
});

L.polylineDecorator = function(paths, options) {
    return new L.PolylineDecorator(paths, options);
};

L.Symbol = {};
L.Symbol.arrowHead = function(options) {
    return {options: options};
};
</script>
</body>
</html>
"""
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    w = EnhancedGCS()
    w.show()
    sys.exit(app.exec_())