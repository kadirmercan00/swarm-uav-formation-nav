#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone 1 – FOLLOWER with ArUco Detection
- ArUco marker tespiti eklendi
- Tespit durumunda hedef ortalama ve koordinat gönderme
"""
import time, math, threading, json
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from esp_mesh import ESPMesh
import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

DRONE_ID = 1
CONN_STR = "udp:127.0.0.1:14551"
ESP_PORT = "/dev/ttyUSB1"
M_PER_DEG = 1.113195e5

# ArUco parametreleri
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
PARAMS = cv2.aruco.DetectorParameters()
TARGET_ERROR_THRESHOLD = 500  # Hedefleme başlama eşiği
CENTERED_ERROR_THRESHOLD = 300  # Hedef ortalandı eşiği

def wrap360(a):
    """Wrap angle to [0, 360) degrees"""
    a = float(a)
    return a % 360.0

def get_location_metres(origin, dN, dE, alt):
    """North-East ofsetten GPS konumu hesapla"""
    R = 6378137.0
    dLat = dN / R
    dLon = dE / (R * math.cos(math.pi * origin.lat / 180.0))
    return LocationGlobalRelative(origin.lat + (dLat * 180.0 / math.pi),
                                  origin.lon + (dLon * 180.0 / math.pi), alt)

def ne_from_latlon(origin, lat, lon):
    """Lat/lon'dan North-East ofset hesapla"""
    dN = (lat - origin.lat) * M_PER_DEG
    dE = (lon - origin.lon) * M_PER_DEG * math.cos(math.radians(origin.lat))
    return np.array([float(dN), float(dE)], dtype=float)

def arm_and_takeoff(vehicle, target_alt):
    """Drone'u arm et ve belirtilen irtifaya kaldır"""
    print(f"[UAV{DRONE_ID}] Arm edilebilirlik kontrol ediliyor...")
    while not vehicle.is_armable:
        print(f"[UAV{DRONE_ID}] Arm edilebilirlik bekleniyor...")
        time.sleep(0.5)
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED": 
        time.sleep(0.2)
    
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(0.2)
    
    print(f"[UAV{DRONE_ID}] TAKEOFF -> {target_alt} m")
    vehicle.simple_takeoff(target_alt)
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt and alt >= target_alt * 0.95: 
            break
        print(f"[UAV{DRONE_ID}] İrtifa: {alt:.1f}m / {target_alt}m")
        time.sleep(0.4)
    
    print(f"[UAV{DRONE_ID}] Hedef irtifa tamam")

def ensure_guided_and_alt(v, target_alt):
    """Ensure vehicle is in GUIDED mode and at (or climbing to) target_alt."""
    if v.mode.name != "GUIDED":
        v.mode = VehicleMode("GUIDED")
        while v.mode.name != "GUIDED":
            time.sleep(0.2)
    if (not getattr(v, 'armed', False)) or ((getattr(v.location, 'global_relative_frame', None) and (v.location.global_relative_frame.alt or 0)) < target_alt * 0.9):
        arm_and_takeoff(v, target_alt)

def goto_NE(vehicle, origin, dN, dE, alt, gs=3.0):
    """Belirtilen North-East konumuna git"""
    vehicle.groundspeed = gs
    tgt = get_location_metres(origin, dN, dE, alt)
    vehicle.simple_goto(tgt)

def wait_to_reach(vehicle, origin, target_ne, tol=2.0, timeout=120):
    """Hedefe ulaşana kadar bekle"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        if not (loc and loc.lat and loc.lon):
            time.sleep(0.3)
            continue
        curr = ne_from_latlon(origin, loc.lat, loc.lon)
        dist = np.linalg.norm(curr - target_ne)
        if dist < tol: 
            print(f"[UAV{DRONE_ID}] Hedefe ulaşıldı (hata: {dist:.1f}m)")
            return True
        time.sleep(0.4)
    print(f"[UAV{DRONE_ID}] Hedefe ulaşma zaman aşımı!")
    return False

def state_broadcast_loop(vehicle, mesh, running_flag):
    """Durum yayınlama döngüsü"""
    while running_flag[0]:
        try:
            loc = vehicle.location.global_relative_frame
            if loc and loc.lat and loc.lon:
                yaw = vehicle.attitude.yaw
                mesh.broadcast_state(loc.lat, loc.lon, loc.alt or 0.0, math.degrees(yaw))
            time.sleep(0.5)
        except Exception as e:
            print(f"[UAV{DRONE_ID}] State broadcast hatası: {e}")
            time.sleep(0.5)

def aruco_detection_loop(vehicle, mesh, origin, detection_active, stop_event):
    """ArUco tespit döngüsü"""
    if not detection_active[0]:
        return
    
    picam2 = None
    try:
        # Kamera başlat
        picam2 = Picamera2()
        picam2.start()
        print(f"[UAV{DRONE_ID}] ArUco tespit sistemi aktif!")
        
        centering_active = False
        error_x, error_y = 0, 0
        
        while detection_active[0] and not stop_event.is_set():
            try:
                # Frame yakala
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                h, w = frame.shape[:2]
                cam_cx, cam_cy = w // 2, h // 2
                
                # ArUco tespit
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)
                
                if ids is not None and len(corners) > 0:
                    # İlk marker'ı al
                    pts = corners[0][0]
                    mx = int(np.mean(pts[:, 0]))
                    my = int(np.mean(pts[:, 1]))
                    
                    error_x = mx - cam_cx
                    error_y = my - cam_cy
                    total_error = abs(error_x) + abs(error_y)
                    
                    print(f"[UAV{DRONE_ID}] ArUco tespit! Error X:{error_x}, Y:{error_y}, Total:{total_error}")
                    
                    # Hedefleme kontrolü
                    if total_error < TARGET_ERROR_THRESHOLD and not centering_active:
                        centering_active = True
                        print(f"[UAV{DRONE_ID}] HEDEF TESPİT! Ortalama başlıyor...")
                        
                        # Loiter moduna geç (havada dur)
                        vehicle.mode = VehicleMode("LOITER")
                        
                        # Diğer drone'lara tespit sinyali gönder
                        mesh.send({
                            "type": "detection",
                            "from": DRONE_ID,
                            "status": "centering",
                            "error_x": error_x,
                            "error_y": error_y
                        })
                    
                    # Hedef ortalama kontrolü
                    if centering_active:
                        # Basit P kontrolör ile hedefi ortala
                        kp = 0.001  # Proporsiyon katsayısı
                        
                        # Velocity komutları hesapla (North-East)
                        vel_north = -error_y * kp  # Y ekseni ters
                        vel_east = error_x * kp
                        vel_down = 0  # Sabit irtifa
                        
                        # Velocity komut gönder
                        vehicle.send_mavlink(
                            vehicle.message_factory.set_position_target_local_ned_encode(
                                0, 0, 0, 
                                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                0b0000111111000111,  # Velocity mask
                                0, 0, 0,  # Position (ignored)
                                vel_north, vel_east, vel_down,  # Velocity
                                0, 0, 0,  # Acceleration (ignored)
                                0, 0  # Yaw, yaw_rate (ignored)
                            )
                        )
                        
                        # Hedef ortalandı mı?
                        if total_error < CENTERED_ERROR_THRESHOLD:
                            print(f"[UAV{DRONE_ID}] HEDEF ORTALANDI! Konum kaydediliyor...")
                            
                            # Mevcut konumu al
                            current_loc = vehicle.location.global_relative_frame
                            
                            # Drone2'ye tespit konumunu gönder
                            mesh.send({
                                "type": "tespit_hedef_ortalandi",
                                "from": DRONE_ID,
                                "lat": current_loc.lat,
                                "lon": current_loc.lon,
                                "alt": current_loc.alt,
                                "error_x": error_x,
                                "error_y": error_y
                            })
                            
                            print(f"[UAV{DRONE_ID}] Tespit konumu gönderildi! 5m kuzeye gidip iniş yapılıyor...")
                            
                            # 5 metre kuzeye git
                            goto_NE(vehicle, origin, 5.0, 0.0, current_loc.alt, gs=2.0)
                            time.sleep(5)  # Hareket için bekle
                            
                            # İniş yap
                            print(f"[UAV{DRONE_ID}] İNİŞ BAŞLIYOR!")
                            vehicle.mode = VehicleMode("LAND")
                            
                            # Tespit tamamlandı, döngüden çık
                            detection_active[0] = False
                            break
                
                else:
                    # ArUco görünmüyor
                    if centering_active:
                        print(f"[UAV{DRONE_ID}] ArUco kaybedildi, aranıyor...")
                        centering_active = False
                
                time.sleep(0.1)  # 10 FPS
                
            except Exception as e:
                print(f"[UAV{DRONE_ID}] ArUco işlem hatası: {e}")
                time.sleep(0.1)
        
    except Exception as e:
        print(f"[UAV{DRONE_ID}] ArUco sistem hatası: {e}")
    finally:
        if picam2:
            picam2.stop()
            cv2.destroyAllWindows()
        print(f"[UAV{DRONE_ID}] ArUco tespit sistemi kapatıldı")

def execute_scan_pattern(vehicle, origin, waypoints, alt, speed, mesh, stop_event, detection_active):
    """Tarama paternini çalıştır - ArUco tespit entegreli"""
    print(f"[UAV{DRONE_ID}] Tarama görevi başlıyor, {len(waypoints)} nokta...")
    
    # ArUco tespit thread'ini başlat
    detection_active[0] = True
    aruco_thread = threading.Thread(
        target=aruco_detection_loop,
        args=(vehicle, mesh, origin, detection_active, stop_event),
        daemon=True
    )
    aruco_thread.start()
    
    for idx, wp in enumerate(waypoints):
        if stop_event.is_set() or not detection_active[0]:
            print(f"[UAV{DRONE_ID}] Tarama durduruldu!")
            return "DETECTED"
        
        dN, dE = wp[0], wp[1]
        print(f"[UAV{DRONE_ID}] Tarama noktası {idx+1}/{len(waypoints)}: N={dN:.1f}, E={dE:.1f}")
        
        goto_NE(vehicle, origin, dN, dE, alt, gs=speed)
        
        # Hedefe ulaşana kadar bekle
        target_ne = np.array([dN, dE])
        t0 = time.time()
        timeout = 120
        
        while time.time() - t0 < timeout:
            if stop_event.is_set() or not detection_active[0]:
                print(f"[UAV{DRONE_ID}] Tarama durduruldu!")
                return "DETECTED"
            
            loc = vehicle.location.global_relative_frame
            if loc and loc.lat and loc.lon:
                curr = ne_from_latlon(origin, loc.lat, loc.lon)
                dist = np.linalg.norm(curr - target_ne)
                if dist < 2.0:
                    break
            time.sleep(0.3)
    
    detection_active[0] = False
    print(f"[UAV{DRONE_ID}] Tarama tamamlandı!")
    return "COMPLETED"

def main():
    mesh = ESPMesh(port=ESP_PORT, drone_id=DRONE_ID)
    if not mesh.connect():
        print(f"[UAV{DRONE_ID}] ESP32 mesh bağlantısı başarısız!")
        return

    v1 = connect(CONN_STR, baud=57600, wait_ready=True)
    origin = None
    running = [True]
    scan_stop_event = threading.Event()
    scan_thread = None
    detection_active = [False]  # ArUco tespit aktif mi?
    
    # State broadcast thread'i başlat
    threading.Thread(target=state_broadcast_loop, args=(v1, mesh, running), daemon=True).start()

    print(f"[UAV{DRONE_ID}] Komutları bekliyor...")

    while True:
        pkt = mesh.receive(timeout=0.05)
        if not pkt: 
            continue
        
        data = pkt.get('data', {})
        t = data.get('type')

        if t == 'takeoff':
            # Leader'dan gelen takeoff komutu
            leader_lat = float(data.get('lat'))
            leader_lon = float(data.get('lon'))
            alt = float(data.get('alt', 10.0))
            
            print(f"[UAV{DRONE_ID}] Leader kalkış yaptı, takip ediliyor...")
            
            # Origin'i leader'ın konumu yap
            origin = LocationGlobalRelative(leader_lat, leader_lon, 0)
            
            # Kalkış yap
            arm_and_takeoff(v1, alt)
            
            mesh.send({
                "type": "ack",
                "for": "takeoff",
                "status": "airborne",
                "drone_id": DRONE_ID
            })

        elif t == 'scan':
            # Tarama komutu
            target_drone = data.get('target_drone')
            
            if target_drone == DRONE_ID:
                waypoints = data.get('waypoints', [])
                alt = float(data.get('alt', 10.0))
                speed = float(data.get('speed', 2.0))
                
                # Uçuşu ve irtifayı garanti et
                ensure_guided_and_alt(v1, alt)
                
                # Origin yoksa mevcut konumu al
                if origin is None:
                    origin = v1.location.global_relative_frame
                
                print(f"[UAV{DRONE_ID}] Tarama görevi alındı, {len(waypoints)} nokta")
                
                mesh.send({
                    "type": "ack",
                    "for": "scan",
                    "status": "starting",
                    "drone_id": DRONE_ID,
                    "waypoint_count": len(waypoints)
                })
                
                # Önceki tarama varsa durdur
                if scan_thread and scan_thread.is_alive():
                    scan_stop_event.set()
                    detection_active[0] = False
                    scan_thread.join(timeout=2)
                
                # Yeni tarama başlat
                scan_stop_event.clear()
                scan_thread = threading.Thread(
                    target=lambda: execute_scan_pattern(v1, origin, waypoints, alt, speed, mesh, scan_stop_event, detection_active),
                    daemon=True
                )
                scan_thread.start()

        elif t == 'detection' or t == 'detection_broadcast':
            # Başka bir drone tespit yaptı
            from_drone = data.get('from')
            if from_drone != DRONE_ID:
                print(f"[UAV{DRONE_ID}] Drone{from_drone} tespit yaptı! Tarama durduruluyor...")
                scan_stop_event.set()
                detection_active[0] = False
                v1.mode = VehicleMode("LOITER")

        elif t == 'land' or t == 'ui_land':
            print(f"[UAV{DRONE_ID}] İniş komutu alındı")
            scan_stop_event.set()
            detection_active[0] = False
            v1.mode = VehicleMode("LAND")
            break

    running[0] = False
    time.sleep(0.4)
    mesh.close()
    v1.close()
    print(f"[UAV{DRONE_ID}] Bitti.")

if __name__ == "__main__":
    main()