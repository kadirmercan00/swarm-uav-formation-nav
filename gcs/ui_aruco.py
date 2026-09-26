#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Planner Style Dual-drone lawnmower mission planner
- Her drone için iki waypoint ile kare alan belirleme
- PyQt5 UI (dark theme)
- QtWebEngine + Leaflet + QWebChannel map
- Waypoint'leri haritada işaretleme ve düzenleme
- Parameters panel (IDs, spacing, speed, altitude, direction)
- Sends per-drone single_navigation JSON over Serial (ESP Mesh) or Demo log
"""
import json, math, os, sys, time, random
from dataclasses import dataclass
from typing import List, Tuple

# ---------------- Qt setup ----------------
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QGroupBox, QListWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit, QDoubleSpinBox,
    QSpinBox, QComboBox, QTextEdit, QMessageBox, QCheckBox, QFormLayout,
    QTabWidget
)

# WebEngine may be missing on some systems (CI), provide safe fallback
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWebChannel import QWebChannel
    WEB_OK = True
except Exception:
    WEB_OK = False
    from PyQt5.QtWidgets import QVBoxLayout
    class QWebEngineView(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            l = QVBoxLayout(self)
            l.addWidget(QLabel("QtWebEngine bulunamadı. Harita önizlemesi devre dışı."))
    class QWebChannel:
        def registerObject(self, *a, **k): pass

# Serial is optional – if missing we still run in Demo
try:
    import serial
    from serial.tools import list_ports
    SERIAL_OK = True
except Exception:
    SERIAL_OK = False

# Add these imports
import threading
from collections import deque

# Add this class near the top (before Main class)
class SerialReader(QObject):
    data_received = pyqtSignal(str)
    
    def __init__(self, serial_conn):
        super().__init__()
        self.serial = serial_conn
        self.running = True
        self.buffer = ""
        
    def run(self):
        while self.running:
            if self.serial and self.serial.is_open:
                try:
                    # Read all available data or wait for data
                    if self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                        if data:
                            self.buffer += data
                            while '\n' in self.buffer:
                                line, self.buffer = self.buffer.split('\n', 1)
                                self.data_received.emit(line.strip())
                    else:
                        time.sleep(0.01)  # Small sleep to prevent CPU hogging
                except Exception as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

DARK_QSS = """
QMainWindow { background:#0e1116; color:#e6edf3; }
QGroupBox { border:1px solid #30363d; border-radius:12px; margin-top:12px; padding-top:8px; }
QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 6px; color:#a5d6ff; }
QLabel { color:#c9d1d9; }
QLineEdit,QTextEdit,QComboBox,QDoubleSpinBox,QSpinBox { background:#0b0f14; color:#e6edf3; border:1px solid #202938; border-radius:10px; padding:6px; }
QPushButton { background:#2f81f7; color:white; border:none; padding:10px 14px; border-radius:10px; font-weight:700; }
QPushButton:hover { background:#418cff; }
QPushButton:disabled { background:#263341; color:#8b949e; }
QListWidget { background:#0b0f14; border:1px solid #202938; border-radius:10px; }
QTabWidget::pane { border:1px solid #30363d; border-radius:12px; background:#0e1116; }
QTabBar::tab { background:#0b0f14; color:#c9d1d9; padding:8px 16px; border-top-left-radius:6px; border-top-right-radius:6px; }
QTabBar::tab:selected { background:#2f81f7; color:white; }
"""

LEAFLET_HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
html, body, #map { height:100%; margin:0; background:#0e1116;}
.leaflet-control { filter: invert(0); }
.leaflet-bar a, .leaflet-control-layers { background:#11161d; color:#fff; border-color:#202938; }
.legend { position:absolute; bottom:10px; left:10px; background:rgba(0,0,0,0.7); color:#fff; padding:8px 10px; border-radius:8px; font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial; }
.badge { padding:2px 6px; border-radius:6px; font-weight:700; }
.a { background:#00e5a8; color:#001; } .b { background:#ffd166; color:#001; }
.param { position:absolute; top:10px; left:10px; background:rgba(0,0,0,0.7); color:#fff; padding:8px 10px; border-radius:8px; }
.waypoint-marker { border-radius:50%; }
.drone-marker { border-radius:50%; }
</style>
</head>
<body>
<div id="map"></div>
<div class="legend">Önizleme – <span class="badge a">Drone A</span>  <span class="badge b">Drone B</span></div>
<script>
var map = L.map('map', {attributionControl:true}).setView([39.0,35.0],6);

// Base layers
var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:21, attribution:'© OpenStreetMap'});
var esri = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {maxZoom:21, attribution:'© Esri'});
esri.addTo(map);
L.control.layers({"OSM":osm,"Uydu":esri},{}).addTo(map);

// Waypoint markers
var waypointsA = [], waypointsB = [];
var markersA = [], markersB = [];
var areaA = null, areaB = null;
var stripesA = null, stripesB = null;

// Add these variables at the top with other global variables
// Use string keys '0' and '1' for drone IDs
var droneMarkers = {'0': null, '1': null};

var lastParams = { spacing:10, direction:'NS', angle:0 };
var currentDrone = 'A'; // Varsayılan drone

// Marker icons
var iconA = L.divIcon({className: 'waypoint-marker', html: '<div style="background:#00e5a8; width:16px; height:16px; border:2px solid white; border-radius:50%;"></div>', iconSize: [20, 20]});
var iconB = L.divIcon({className: 'waypoint-marker', html: '<div style="background:#ffd166; width:16px; height:16px; border:2px solid white; border-radius:50%;"></div>', iconSize: [20, 20]});

// Add this function to update drone positions
function updateDronePosition(drone, lat, lng, alt, yaw) {
    // drone is expected to be '0' or '1'
    var color = drone === '0' ? '#00e5a8' : '#ffd166';
    var icon = L.divIcon({
        className: 'drone-marker',
        html: `<div style="background:${color}; width:20px; height:20px; border:2px solid white; border-radius:50%;"></div>`,
        iconSize: [24, 24]
    });
    
    if (droneMarkers[drone]) {
        droneMarkers[drone].setLatLng([lat, lng]);
        droneMarkers[drone].setPopupContent(`Drone ${drone}<br>Position: ${lat.toFixed(6)}, ${lng.toFixed(6)}<br>Altitude: ${alt}m`);
    } else {
        droneMarkers[drone] = L.marker([lat, lng], {icon: icon})
            .addTo(map)
            .bindPopup(`Drone ${drone}<br>Position: ${lat.toFixed(6)}, ${lng.toFixed(6)}<br>Altitude: ${alt}m`);
    }
}

function clearDronePositions() {
    if (droneMarkers['0']) { map.removeLayer(droneMarkers['0']); droneMarkers['0'] = null; }
    if (droneMarkers['1']) { map.removeLayer(droneMarkers['1']); droneMarkers['1'] = null; }
}

function addWaypoint(latLng, drone) {
    var icon = drone === 'A' ? iconA : iconB;
    var marker = L.marker(latLng, {icon: icon, draggable: true});
    marker.addTo(map);
    
    marker.on('dragend', function(e) {
        var marker = e.target;
        var position = marker.getLatLng();
        if (drone === 'A') {
            var index = markersA.indexOf(marker);
            if (index !== -1) {
                waypointsA[index] = position;
            }
        } else {
            var index = markersB.indexOf(marker);
            if (index !== -1) {
                waypointsB[index] = position;
            }
        }
        updateArea(drone);
        rebuild();
    });
    
    if (drone === 'A') {
        waypointsA.push(latLng);
        markersA.push(marker);
        if (waypointsA.length > 2) {
            map.removeLayer(markersA.shift());
            waypointsA.shift();
        }
    } else {
        waypointsB.push(latLng);
        markersB.push(marker);
        if (waypointsB.length > 2) {
            map.removeLayer(markersB.shift());
            waypointsB.shift();
        }
    }
    
    updateArea(drone);
    rebuild();
}

function updateArea(drone) {
    var waypoints = drone === 'A' ? waypointsA : waypointsB;
    var area = drone === 'A' ? areaA : areaB;
    
    if (area) {
        map.removeLayer(area);
        if (drone === 'A') areaA = null;
        else areaB = null;
    }
    
    if (waypoints.length === 2) {
        var bounds = L.latLngBounds(waypoints);
        var areaColor = drone === 'A' ? '#00e5a8' : '#ffd166';
        area = L.rectangle(bounds, {color: areaColor, weight: 2, fillOpacity: 0.1});
        area.addTo(map);
        
        if (drone === 'A') areaA = area;
        else areaB = area;
    }
}

function generateLawnmower(bounds, spacing, direction, angleDeg){
    if (!bounds) return [];
    var c = bounds.getCenter();
    var sw = bounds.getSouthWest(), ne = bounds.getNorthEast();
    // Half sizes in meters
    var wh = latLngToMeters(L.latLng(c.lat, ne.lng), c)[0];
    var hh = latLngToMeters(L.latLng(ne.lat, c.lng), c)[1];
    var W = Math.abs(wh), H = Math.abs(hh);
    var ang = angleDeg * Math.PI/180.0;
    var cosA = Math.cos(ang), sinA = Math.sin(ang);

    function toWorld(x,y){ // local rotated -> latlng
        var xr = x*cosA - y*sinA;
        var yr = x*sinA + y*cosA;
        return metersToLatLng([xr,yr], c);
    }

    var paths=[];
    var passes = (direction=='NS') ? Math.ceil((2*W)/spacing) : Math.ceil((2*H)/spacing);
    var xs=[], ys=[];
    if (direction=='NS'){
        for (var i=0;i<passes;i++){
            var x = -W + i*spacing;
            if (x>W) x = W;
            xs.push(x);
        }
        // assign alternating from edges towards center
        var turnTop=[-H,H], turnBottom=[H,-H];
        for (var k=0;k<xs.length;k++){
            var x=xs[k];
            var seq = (k%2==0)? turnTop: turnBottom; // start bottom->top alternating for lawnmower turns
            var p0 = toWorld(x, seq[0]);
            var p1 = toWorld(x, seq[1]);
            paths.push([p0,p1]);
        }
    } else { // EW passes
        for (var i=0;i<passes;i++){
            var y = -H + i*spacing;
            if (y>H) y = H;
            ys.push(y);
        }
        var turnLeft=[-W,W], turnRight=[W,-W];
        for (var k=0;k<ys.length;k++){
            var y=ys[k];
            var seq = (k%2==0)? turnLeft: turnRight;
            var p0 = toWorld(seq[0], y);
            var p1 = toWorld(seq[1], y);
            paths.push([p0,p1]);
        }
    }
    return paths;
}

function drawPreview(dataA, dataB){
    if (stripesA){ map.removeLayer(stripesA); stripesA=null; }
    if (stripesB){ map.removeLayer(stripesB); stripesB=null; }
    
    if (dataA && dataA.length > 0) {
        var la=[];
        dataA.forEach(seg=>{ la.push(seg[0]); la.push(seg[1]); });
        stripesA = L.polyline(la,{color:'#00e5a8',weight:3}).addTo(map);
    }
    
    if (dataB && dataB.length > 0) {
        var lb=[];
        dataB.forEach(seg=>{ lb.push(seg[0]); lb.push(seg[1]); });
        stripesB = L.polyline(lb,{color:'#ffd166',weight:3}).addTo(map);
    }
}

function rebuild(){
    var spacing = lastParams.spacing;
    var dir = lastParams.direction;
    var ang = lastParams.angle;
    
    var resA = null, resB = null;
    
    if (waypointsA.length === 2) {
        resA = generateLawnmower(L.latLngBounds(waypointsA), spacing, dir, ang);
    }
    
    if (waypointsB.length === 2) {
        resB = generateLawnmower(L.latLngBounds(waypointsB), spacing, dir, ang);
    }
    
    drawPreview(resA, resB);
}

// Harita tıklama olayı
map.on('click', function(e) {
    addWaypoint(e.latlng, currentDrone);
});

// Yardımcı fonksiyonlar
function latLngToMeters(latlng, origin){
    var R = 6378137.0;
    var dLat = (latlng.lat - origin.lat) * Math.PI/180.0;
    var dLon = (latlng.lng - origin.lng) * Math.PI/180.0;
    var x = R * dLon * Math.cos(origin.lat * Math.PI/180.0);
    var y = R * dLat;
    return [x,y];
}
function metersToLatLng(xy, origin){
    var R = 6378137.0;
    var x = xy[0], y = xy[1];
    var dLat = y / R;
    var dLon = x / (R * Math.cos(origin.lat * Math.PI/180.0));
    return L.latLng(origin.lat + dLat*180/Math.PI, origin.lng + dLon*180/Math.PI);
}

// QWebChannel bridge
var qt = null;
function setParams(p){ lastParams = p; rebuild(); }
function clearWaypoints(drone) {
    if (drone === 'A') {
        markersA.forEach(marker => map.removeLayer(marker));
        waypointsA = [];
        markersA = [];
        if (areaA) { map.removeLayer(areaA); areaA = null; }
    } else {
        markersB.forEach(marker => map.removeLayer(marker));
        waypointsB = [];
        markersB = [];
        if (areaB) { map.removeLayer(areaB); areaB = null; }
    }
    rebuild();
}
function getWaypoints(){
    function toList(lines){
        var arr=[];
        for (var i=0;i<lines.length;i++){
            arr.push([lines[i][0].lat, lines[i][0].lng]);
            arr.push([lines[i][1].lat, lines[i][1].lng]);
        }
        return arr;
    }
    
    var spacing = lastParams.spacing;
    var dir = lastParams.direction;
    var ang = lastParams.angle;
    
    var pathsA = [], pathsB = [];
    
    if (waypointsA.length === 2) {
        pathsA = generateLawnmower(L.latLngBounds(waypointsA), spacing, dir, ang);
    }
    
    if (waypointsB.length === 2) {
        pathsB = generateLawnmower(L.latLngBounds(waypointsB), spacing, dir, ang);
    }
    
    return JSON.stringify({A:toList(pathsA), B:toList(pathsB)});
}

// Python'dan çağrılacak fonksiyonlar
function setCurrentDrone(drone) {
    currentDrone = drone;
}
function getWaypointsForDrone(drone) {
    if (drone === 'A') {
        return waypointsA.map(wp => [wp.lat, wp.lng]);
    } else {
        return waypointsB.map(wp => [wp.lat, wp.lng]);
    }
}
</script>
</body>
</html>
"""

# ---------------- Main Window ----------------
class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mission Planner Style Dual Lawnmower GCS")
        self.resize(1500, 900)
        self.setStyleSheet(DARK_QSS)

        self.serial = None
        self.demoMode = True  # default demo on unless serial connected
        self.currentDrone = "0"  # Default to Drone 0

        root = QSplitter(Qt.Horizontal, self)

        # Left: map
        left = QGroupBox("Harita")
        llay = QVBoxLayout(left)
        self.web = QWebEngineView()
        if WEB_OK:
            self.web.setHtml(LEAFLET_HTML)
        llay.addWidget(self.web)

        # Right: controls
        right = QGroupBox("Kontrol Paneli")
        rlay = QVBoxLayout(right)

        # Drone selection tabs
        self.tabs = QTabWidget()
        self.tabA = QWidget()
        self.tabB = QWidget()
        self.tabs.addTab(self.tabA, "Drone 0")
        self.tabs.addTab(self.tabB, "Drone 1")

        # Drone 0 tab
        tabAlay = QVBoxLayout(self.tabA)
        gidA = QGroupBox("Drone 0 Kimliği")
        gA = QFormLayout(gidA)
        self.ed_id_a = QLineEdit("0")
        gA.addRow("Drone 0 ID:", self.ed_id_a)
        # State ID spinbox for mapping serial 'id' to drone
        self.state_id_a = QSpinBox()
        self.state_id_a.setRange(0, 255)
        self.state_id_a.setValue(0)
        gA.addRow("State ID:", self.state_id_a)
        tabAlay.addWidget(gidA)

        # Waypoint controls for 0
        gwA = QGroupBox("Drone 0 Waypoint'ler")
        gwAlay = QVBoxLayout(gwA)
        self.wp_list_a = QListWidget()
        self.btn_clear_a = QPushButton("Waypoint'leri Temizle")
        self.btn_clear_a.clicked.connect(lambda: self.clear_waypoints("0"))
        gwAlay.addWidget(QLabel("Waypoint'ler (en fazla 2):"))
        gwAlay.addWidget(self.wp_list_a)
        gwAlay.addWidget(self.btn_clear_a)
        tabAlay.addWidget(gwA)

        # Drone 1 tab
        tabBlay = QVBoxLayout(self.tabB)
        gidB = QGroupBox("Drone 1 Kimliği")
        gB = QFormLayout(gidB)
        self.ed_id_b = QLineEdit("1")
        gB.addRow("Drone 1 ID:", self.ed_id_b)
        # State ID spinbox for mapping serial 'id' to drone
        self.state_id_b = QSpinBox()
        self.state_id_b.setRange(0, 255)
        self.state_id_b.setValue(1)
        gB.addRow("State ID:", self.state_id_b)
        tabBlay.addWidget(gidB)

        # Waypoint controls for 1
        gwB = QGroupBox("Drone 1 Waypoint'ler")
        gwBlay = QVBoxLayout(gwB)
        self.wp_list_b = QListWidget()
        self.btn_clear_b = QPushButton("Waypoint'leri Temizle")
        self.btn_clear_b.clicked.connect(lambda: self.clear_waypoints("1"))
        gwBlay.addWidget(QLabel("Waypoint'ler (en fazla 2):"))
        gwBlay.addWidget(self.wp_list_b)
        gwBlay.addWidget(self.btn_clear_b)
        tabBlay.addWidget(gwB)

        rlay.addWidget(self.tabs)

        # Mission params
        gp = QGroupBox("Tarama Parametreleri")
        pf = QFormLayout(gp)
        self.cmb_dir = QComboBox(); self.cmb_dir.addItems(["NS (Kuzey↔Güney)", "EW (Doğu↔Batı)"])
        self.sp_spacing = QDoubleSpinBox(); self.sp_spacing.setRange(1,200); self.sp_spacing.setValue(20); self.sp_spacing.setSuffix(" m")
        self.sp_angle = QDoubleSpinBox(); self.sp_angle.setRange(0,359); self.sp_angle.setValue(0); self.sp_angle.setSuffix(" °")
        self.sp_alt = QDoubleSpinBox(); self.sp_alt.setRange(1,1000); self.sp_alt.setValue(30); self.sp_alt.setSuffix(" m")
        self.sp_speed = QDoubleSpinBox(); self.sp_speed.setRange(0.5,50); self.sp_speed.setValue(6); self.sp_speed.setSuffix(" m/s")
        pf.addRow("Tarama yönü:", self.cmb_dir)
        pf.addRow("Şerit aralığı:", self.sp_spacing)
        pf.addRow("Açı (Alan Döndür):", self.sp_angle)
        pf.addRow("İrtifa:", self.sp_alt)
        pf.addRow("Hız:", self.sp_speed)
        rlay.addWidget(gp)

        # Buttons
        hb = QHBoxLayout()
        self.btn_preview = QPushButton("🔁 Önizlemeyi Güncelle")
        self.btn_send = QPushButton("📤 Görevi Gönder (ESP Mesh)")
        hb.addWidget(self.btn_preview); hb.addWidget(self.btn_send)
        rlay.addLayout(hb)

        # Additional control buttons
        hb_control = QHBoxLayout()
        self.btn_takeoff = QPushButton("🛫 Kalkış")
        self.btn_land = QPushButton("🛬 İniş")
        self.btn_emergency_land = QPushButton("🚨 Acil İniş")
        self.btn_abort = QPushButton("❌ Görevi İptal Et")
        hb_control.addWidget(self.btn_takeoff)
        hb_control.addWidget(self.btn_land)
        hb_control.addWidget(self.btn_emergency_land)
        hb_control.addWidget(self.btn_abort)
        rlay.addLayout(hb_control)

        # Log
        self.logView = QTextEdit(); self.logView.setReadOnly(True); self.logView.setMinimumHeight(220)
        rlay.addWidget(self.logView)

        # Serial (minimal)
        gs = QGroupBox("ESP Mesh Seri")
        sf = QFormLayout(gs)
        self.cmb_port = QComboBox()
        if SERIAL_OK:
            self.refresh_ports()
        self.cmb_baud = QComboBox(); self.cmb_baud.addItems(["115200","57600","38400","19200","9600"]); self.cmb_baud.setCurrentText("115200")
        self.btn_serial = QPushButton("🔌 Bağlan")
        self.btn_serial.clicked.connect(self.toggle_serial)
        self.btn_refresh = QPushButton("🔄 Portları Yenile")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        sf.addRow("Port:", self.cmb_port)
        sf.addRow("Baud:", self.cmb_baud)
        hbox = QHBoxLayout()
        hbox.addWidget(self.btn_serial)
        hbox.addWidget(self.btn_refresh)
        sf.addRow("", hbox)
        rlay.addWidget(gs)

        # Assemble
        root.addWidget(left); root.addWidget(right); root.setSizes([900, 600])
        self.setCentralWidget(root)

        # Wire
        self.btn_preview.clicked.connect(self.update_preview)
        self.btn_send.clicked.connect(self.send_mission)
        self.tabs.currentChanged.connect(self.tab_changed)
        
        # Connect new control buttons
        self.btn_takeoff.clicked.connect(self.send_takeoff)
        self.btn_land.clicked.connect(self.send_land)
        self.btn_emergency_land.clicked.connect(self.send_emergency_land)
        self.btn_abort.clicked.connect(self.send_abort_mission)

        # Update waypoint lists initially
        QTimer.singleShot(1000, self.update_waypoint_lists)

        # Demo timer to print alive
        self.t = QTimer(self); self.t.timeout.connect(lambda: None); self.t.start(2000)

        # Add these after setting up serial
        self.serial_reader = None
        self.drone_positions = {'0': None, '1': None}

        # Start serial reading thread
        self.start_serial_reading()

    def refresh_ports(self):
        if SERIAL_OK:
            self.cmb_port.clear()
            ports = list_ports.comports()
            for port in ports:
                self.cmb_port.addItem(port.device, port)

    def start_serial_reading(self):
        if self.serial and self.serial.is_open:
            if self.serial_reader is None:
                self.serial_reader = SerialReader(self.serial)
                self.serial_reader.data_received.connect(self.handle_serial_data)
                self.reader_thread = threading.Thread(target=self.serial_reader.run, daemon=True)
                self.reader_thread.start()
                self.log("Seri port okuma başlatıldı")

    def handle_serial_data(self, data):
        try:
            self.log(f"← {data}")  # Gelen veriyi logla
            if data.startswith('{') and data.endswith('}'):
                state_data = json.loads(data)
                if state_data.get('type') == 'state':
                    drone_id = state_data.get('id')
                    # Map numeric IDs to drone letters using UI settings
                    if drone_id == self.state_id_a.value():
                        drone = '0'
                    elif drone_id == self.state_id_b.value():
                        drone = '1'
                    else:
                        return  # Unknown drone ID
                    
                    lat = state_data.get('lat')
                    lon = state_data.get('lon')
                    alt = state_data.get('alt')
                    yaw = state_data.get('yaw', 0)
                    
                    if lat is not None and lon is not None:
                        self.drone_positions[drone] = (lat, lon, alt, yaw)
                        self.update_drone_position(drone, lat, lon, alt, yaw)
        except Exception as e:
            self.log(f"State data error: {e}")

    def update_drone_position(self, drone, lat, lon, alt, yaw):
        # Update drone position on map
        js_code = f"updateDronePosition('{drone}', {lat}, {lon}, {alt}, {yaw});"
        self.js(js_code)

    def send_takeoff(self):
        """Send takeoff command for selected drone"""
        drone_id = self.ed_id_a.text().strip() if self.tabs.currentIndex() == 0 else self.ed_id_b.text().strip()
        alt = float(self.sp_alt.value())
        
        # Get drone's current position if available, otherwise use first waypoint
        lat, lon = 0.0, 0.0
        if drone_id in self.drone_positions and self.drone_positions[drone_id]:
            lat, lon, current_alt, yaw = self.drone_positions[drone_id]
        
        takeoff_cmd = {
            "type": "ui_takeoff",
            "lat": lat,
            "lon": lon,
            "alt": alt
        }
        
        if self._write_line(json.dumps(takeoff_cmd)):
            self.log(f"Drone {drone_id} kalkış komutu gönderildi: {alt}m")
        else:
            self.error("Kalkış komutu gönderilemedi")

    def send_land(self):
        """Send land command for selected drone"""
        drone_id = self.ed_id_a.text().strip() if self.tabs.currentIndex() == 0 else self.ed_id_b.text().strip()
        
        land_cmd = {
            "type": "ui_land"
        }
        
        if self._write_line(json.dumps(land_cmd)):
            self.log(f"Drone {drone_id} iniş komutu gönderildi")
        else:
            self.error("İniş komutu gönderilemedi")

    def send_emergency_land(self):
        """Send emergency land command for selected drone"""
        drone_id = self.ed_id_a.text().strip() if self.tabs.currentIndex() == 0 else self.ed_id_b.text().strip()
        
        emergency_land_cmd = {
            "type": "emergency_land"
        }
        
        if self._write_line(json.dumps(emergency_land_cmd)):
            self.log(f"Drone {drone_id} acil iniş komutu gönderildi")
        else:
            self.error("Acil iniş komutu gönderilemedi")

    def send_abort_mission(self):
        """Send abort mission command for selected drone"""
        drone_id = self.ed_id_a.text().strip() if self.tabs.currentIndex() == 0 else self.ed_id_b.text().strip()
        
        abort_cmd = {
            "type": "abort_mission"
        }
        
        if self._write_line(json.dumps(abort_cmd)):
            self.log(f"Drone {drone_id} görev iptal komutu gönderildi")
        else:
            self.error("Görev iptal komutu gönderilemedi")

    def tab_changed(self, index):
        self.currentDrone = "A" if index == 0 else "B"
        self.js(f"setCurrentDrone('{self.currentDrone}');")
        self.log(f"Şimdi Drone {self.currentDrone} için waypoint ekleniyor")
        self.update_waypoint_list(self.currentDrone)

    def clear_waypoints(self, drone):
        self.js(f"clearWaypoints('{drone}');")
        self.update_waypoint_list(drone)
        self.log(f"Drone {drone} waypoint'leri temizlendi")

    def update_waypoint_list(self, drone):
        list_widget = self.wp_list_a if drone == "A" else self.wp_list_b
        list_widget.clear()
        
        # JavaScript'ten waypoint'leri al
        self.js(f"getWaypointsForDrone('{drone}')", 
                lambda wps: self._update_wp_list_callback(wps, list_widget) if wps else None)

    def _update_wp_list_callback(self, waypoints, list_widget):
        if not waypoints:
            return
            
        for i, wp in enumerate(waypoints):
            list_widget.addItem(f"{i+1}: {wp[0]:.6f}, {wp[1]:.6f}")

    def update_waypoint_lists(self):
        self.update_waypoint_list("A")
        self.update_waypoint_list("B")

    def js(self, code, cb=None):
        if not WEB_OK: 
            return
        if cb: 
            self.web.page().runJavaScript(code, cb)
        else: 
            self.web.page().runJavaScript(code)

    def update_preview(self):
        direction = "NS" if self.cmb_dir.currentIndex()==0 else "EW"
        params = {
            "spacing": float(self.sp_spacing.value()),
            "direction": direction,
            "angle": float(self.sp_angle.value())
        }
        self.js(f"setParams({json.dumps(params)});")
        self.log("Önizleme güncellendi.")

    def send_mission(self):
        # pull waypoint list from JS
        self.js("getWaypoints();", self._on_wp_json)

    def _on_wp_json(self, s):
        try:
            obj = json.loads(s)
        except Exception:
            self.error("Waypoint üretilemedi. Önce her iki drone için ikişer waypoint seçin.")
            return
            
        A = obj.get("A",[]); B = obj.get("B",[])
        
        # Check if we have enough waypoints
        if len(A) < 4:
            self.error("Drone A için yeterli waypoint yok. En az 2 waypoint gerekli.")
            return
        if len(B) < 4:
            self.error("Drone B için yeterli waypoint yok. En az 2 waypoint gerekli.")
            return

        # Build payloads in the receiver's expected format
        try:
            drone_id_a = int(self.ed_id_a.text().strip())
        except Exception:
            drone_id_a = 0
        try:
            drone_id_b = int(self.ed_id_b.text().strip())
        except Exception:
            drone_id_b = 1

        payloadA = {
            "type": "ui_takeoff",
            "type": "ui_scan",
            "drone_id": drone_id_a,
            "waypoints": A,
            "waypoint_type": "latlon",
            "alt": float(self.sp_alt.value()),
            "speed": float(self.sp_speed.value())
        }

        payloadB = {
            "type": "ui_takeoff",
            "type": "ui_scan",
            "drone_id": drone_id_b,
            "waypoints": B,
            "waypoint_type": "latlon",
            "alt": float(self.sp_alt.value()),
            "speed": float(self.sp_speed.value())
        }

        # Send waypoints
        okA = self._write_line(json.dumps(payloadA, ensure_ascii=False))
        okB = self._write_line(json.dumps(payloadB, ensure_ascii=False))
        
        if okA and okB:
            self.log("✅ İki görev gönderildi.")
        else:
            self.error("Gönderimde sorun oldu.")

    # --- Serial helpers ---
    def toggle_serial(self):
        if not SERIAL_OK:
            self.error("pyserial mevcut değil. Demo modunda devam ediliyor.")
            return
            
        if self.serial and self.serial.is_open:
            # Stop serial reader
            if self.serial_reader:
                self.serial_reader.running = False
                self.reader_thread.join(timeout=0.1)
                self.serial_reader = None
                
            try:
                self.serial.close()
            except Exception:
                pass
                
            self.serial = None
            self.btn_serial.setText("🔌 Bağlan")
            self.demoMode = True
            self.log("Seri bağlantı kapatıldı. Demo açık.")
            # Clear drone positions
            self.drone_positions = {'0': None, '1': None}
            self.js("clearDronePositions();")
            return
            
        port = self.cmb_port.currentText().strip()
        if not port:
            self.error("Port seçin.")
            return
            
        baud = int(self.cmb_baud.currentText())
        try:
            self.serial = serial.Serial(port, baudrate=baud, timeout=1)
            self.btn_serial.setText("🔌 Kes")
            self.demoMode = False
            self.log(f"Seri bağlandı: {port} @{baud}")
            # Start serial reader
            self.start_serial_reading()
        except Exception as e:
            self.serial = None
            self.error(f"Seri bağlanamadı: {e}")

    def _write_line(self, s: str)->bool:
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((s+"\n").encode('utf-8'))
                self.log("→ "+s)
                return True
            except Exception as e:
                self.error(f"Gönderim hatası: {e}")
                return False
        else:
            # Demo: just log
            self.log("⟶ [DEMO] "+s)
            return True

    # --- logging ---
    def log(self, msg): 
        self.logView.append(msg)
        self.logView.verticalScrollBar().setValue(self.logView.verticalScrollBar().maximum())
    def error(self, msg): 
        self.logView.append(f"<span style='color:#ff8a8a'>[Hata] {msg}</span>")
        self.logView.verticalScrollBar().setValue(self.logView.verticalScrollBar().maximum())

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Main()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()