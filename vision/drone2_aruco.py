#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone 2 – RESCUE/INTERVENTION DRONE
- Tespit durumunda otomatik kalkış
- Hedef konumuna gitme ve iniş
- Bekleme modunda başlar
"""
import time, math, threading, json
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from esp_mesh import ESPMesh

DRONE_ID = 2
CONN_STR = "udp:127.0.0.1:14552"
ESP_PORT = "/dev/ttyUSB2"
M_PER_DEG = 1.113195e5

def ne_from_latlon(origin, lat, lon):
    """Lat/lon'dan North-East ofset hesapla"""
    dN = (lat - origin.lat) * M_PER_DEG
    dE = (lon - origin.lon) * M_PER_DEG * math.cos(math.radians(origin.lat))
    return np.array([float(dN), float(dE)], dtype=float)

def get_location_metres(origin, dN, dE, alt):
    """North-East ofsetten GPS konumu hesapla"""
    R = 6378137.0
    dLat = dN / R
    dLon = dE / (R * math.cos(math.pi * origin.lat / 180.0))
    return LocationGlobalRelative(
        origin.lat + (dLat * 180.0 / math.pi),
        origin.lon + (dLon * 180.0 / math.pi), 
        alt
    )

def latlon_from_ne(origin, dN, dE):
    """North-East ofsetten lat/lon hesapla"""
    R = 6378137.0
    dLat = dN / R
    dLon = dE / (R * math.cos(math.pi * origin.lat / 180.0))
    lat = origin.lat + (dLat * 180.0 / math.pi)
    lon = origin.lon + (dLon * 180.0 / math.pi)
    return lat, lon

def arm_and_takeoff(vehicle, target_alt):
    """Drone'u arm et ve belirtilen irtifaya kaldır"""
    print(f"[RESCUE-UAV{DRONE_ID}] Acil kalkış hazırlığı...")
    
    # Hızlı arm için pre-arm check'leri bypass etmeye çalış
    vehicle.parameters['ARMING_CHECK'] = 0
    
    while not vehicle.is_armable:
        print(f"[RESCUE-UAV{DRONE_ID}] Arm edilebilirlik bekleniyor...")
        time.sleep(0.3)
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED": 
        time.sleep(0.1)
    
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(0.1)
    
    print(f"[RESCUE-UAV{DRONE_ID}] ACİL KALKIŞ -> {target_alt} m")
    vehicle.simple_takeoff(target_alt)
    
    t0 = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None and alt >= target_alt * 0.90:  # %90'a ulaşınca yeterli
            break
        if time.time() - t0 > 60:  # 60 saniye timeout
            print(f"[RESCUE-UAV{DRONE_ID}] Kalkış zaman aşımı!")
            break
        print(f"[RESCUE-UAV{DRONE_ID}] İrtifa: {alt:.1f}m / {target_alt}m")
        time.sleep(0.3)
    
    print(f"[RESCUE-UAV{DRONE_ID}] Kalkış tamamlandı, hedefe yönleniyor!")

def goto_location_direct(vehicle, target_lat, target_lon, target_alt, speed=5.0):
    """Doğrudan GPS koordinatına git"""
    vehicle.groundspeed = speed
    target_loc = LocationGlobalRelative(target_lat, target_lon, target_alt)
    vehicle.simple_goto(target_loc)
    print(f"[RESCUE-UAV{DRONE_ID}] Hedef: Lat={target_lat:.6f}, Lon={target_lon:.6f}, Alt={target_alt}m")

def wait_to_reach_location(vehicle, target_lat, target_lon, tol=3.0, timeout=180):
    """Hedef konuma ulaşana kadar bekle"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        if not (loc and loc.lat and loc.lon):
            time.sleep(0.3)
            continue
        
        # Mesafe hesapla (basit haversine)
        dlat = abs(loc.lat - target_lat) * M_PER_DEG
        dlon = abs(loc.lon - target_lon) * M_PER_DEG * math.cos(math.radians(loc.lat))
        dist = math.sqrt(dlat**2 + dlon**2)
        
        if dist < tol:
            print(f"[RESCUE-UAV{DRONE_ID}] Hedefe ulaşıldı! (Hata: {dist:.1f}m)")
            return True
        
        print(f"[RESCUE-UAV{DRONE_ID}] Hedefe mesafe: {dist:.1f}m")
        time.sleep(0.5)
    
    print(f"[RESCUE-UAV{DRONE_ID}] Hedefe ulaşma zaman aşımı!")
    return False

def state_broadcast_loop(v, mesh, running_flag):
    """Durum yayınlama döngüsü"""
    while running_flag[0]:
        try:
            loc = v.location.global_relative_frame
            if loc and loc.lat and loc.lon:
                yaw = v.attitude.yaw if v.attitude else 0
                mesh.broadcast_state(loc.lat, loc.lon, loc.alt or 0.0, math.degrees(yaw))
            time.sleep(0.5)
        except Exception as e:
            print(f"[RESCUE-UAV{DRONE_ID}] State broadcast hatası: {e}")
            time.sleep(0.5)

def emergency_intervention(vehicle, target_data, mesh):
    """Acil müdahale görevi"""
    try:
        # Hedef bilgilerini al
        if 'lat' in target_data and 'lon' in target_data:
            # Direkt GPS koordinatları verilmiş
            target_lat = float(target_data['lat'])
            target_lon = float(target_data['lon'])
        elif 'north' in target_data and 'east' in target_data and 'origin' in target_data:
            # North-East koordinatları verilmiş, dönüştür
            origin = target_data['origin']
            dN = float(target_data['north'])
            dE = float(target_data['east'])
            target_lat, target_lon = latlon_from_ne(origin, dN, dE)
        else:
            print(f"[RESCUE-UAV{DRONE_ID}] HATA: Hedef konumu eksik!")
            return False
        
        target_alt = float(target_data.get('alt', 15.0))
        approach_alt = float(target_data.get('approach_alt', 20.0))  # Yaklaşma irtifası
        speed = float(target_data.get('speed', 5.0))  # Hız
        
        print(f"[RESCUE-UAV{DRONE_ID}] === ACİL MÜDAHALE BAŞLIYOR ===")
        print(f"[RESCUE-UAV{DRONE_ID}] Hedef: {target_lat:.6f}, {target_lon:.6f}")
        
        # 1. Acil kalkış
        arm_and_takeoff(vehicle, approach_alt)
        
        # 2. Hedefe git
        goto_location_direct(vehicle, target_lat, target_lon, approach_alt, speed)
        
        # 3. Hedefe ulaşana kadar bekle
        reached = wait_to_reach_location(vehicle, target_lat, target_lon, tol=3.0, timeout=180)
        
        if reached:
            print(f"[RESCUE-UAV{DRONE_ID}] Hedef bölgeye ulaşıldı, alçalıyor...")
            
            # 4. Hedef irtifaya alçal
            current_loc = vehicle.location.global_relative_frame
            vehicle.simple_goto(LocationGlobalRelative(target_lat, target_lon, target_alt))
            time.sleep(5)  # Alçalma için bekle
            
            # 5. İniş yap
            print(f"[RESCUE-UAV{DRONE_ID}] İNİŞ BAŞLIYOR!")
            vehicle.mode = VehicleMode("LAND")
            
            # İniş tamamlanana kadar bekle
            while vehicle.armed:
                alt = vehicle.location.global_relative_frame.alt
                if alt:
                    print(f"[RESCUE-UAV{DRONE_ID}] İniş... İrtifa: {alt:.1f}m")
                time.sleep(1)
            
            print(f"[RESCUE-UAV{DRONE_ID}] İNİŞ TAMAMLANDI! Görev başarılı.")
            
            # Başarı mesajı gönder
            mesh.send({
                "type": "rescue_complete",
                "drone_id": DRONE_ID,
                "status": "landed",
                "location": {
                    "lat": target_lat,
                    "lon": target_lon
                }
            })
            return True
        else:
            print(f"[RESCUE-UAV{DRONE_ID}] Hedefe ulaşılamadı!")
            vehicle.mode = VehicleMode("LOITER")
            return False
            
    except Exception as e:
        print(f"[RESCUE-UAV{DRONE_ID}] Müdahale hatası: {e}")
        vehicle.mode = VehicleMode("LOITER")
        return False

def connect_vehicle(conn_str):
    print(f"[RESCUE-UAV{DRONE_ID}] {conn_str} adresine bağlanılıyor...")
    v = connect(conn_str, baud=57600, wait_ready=True)
    v._label = f"RESCUE-UAV{DRONE_ID}"
    print(f"[RESCUE-UAV{DRONE_ID}] Bağlantı başarılı!")
    return v

def main():
    mesh = ESPMesh(port=ESP_PORT, drone_id=DRONE_ID)
    if not mesh.connect():
        print(f"[RESCUE-UAV{DRONE_ID}] ESP32 Mesh bağlantısı başarısız!")
        return

    v = None  # Başlangıçta bağlanma, komut bekle
    origin = None
    running = [True]
    state_thread = None
    is_armed = False
    intervention_active = False

    print(f"[RESCUE-UAV{DRONE_ID}] BEKLEME MODUNDA - Tespit sinyali bekleniyor...")
    
    while True:
        msg = mesh.receive(timeout=0.05)
        if not msg or 'data' not in msg: 
            continue
        
        data = msg['data']
        t = data.get('type')
        
        if t == 'tespit_hedef_ortalandi':
            # TESPİT MESAJI - ACİL MÜDAHALE
            if not intervention_active:
                intervention_active = True
                
                print(f"[RESCUE-UAV{DRONE_ID}] !!! TESPİT ALARMI ALINDI !!!")
                print(f"[RESCUE-UAV{DRONE_ID}] Hedef ortalandı, acil müdahale başlıyor!")
                
                # Drone'a henüz bağlanmadıysak bağlan
                if v is None:
                    v = connect_vehicle(CONN_STR)
                    
                    # State broadcast başlat
                    if not state_thread:
                        state_thread = threading.Thread(
                            target=state_broadcast_loop,
                            args=(v, mesh, running),
                            daemon=True
                        )
                        state_thread.start()
                
                # Origin'i belirle (ilk tespit konumu)
                if origin is None:
                    current_loc = v.location.global_relative_frame
                    if current_loc:
                        origin = current_loc
                
                # Acil müdahale görevini başlat
                target_data = {
                    'lat': data.get('lat'),
                    'lon': data.get('lon'),
                    'north': data.get('north'),
                    'east': data.get('east'),
                    'alt': data.get('alt', 5.0),  # İniş irtifası
                    'approach_alt': data.get('approach_alt', 20.0),
                    'speed': data.get('speed', 5.0),
                    'origin': origin
                }
                
                # Müdahale thread'i başlat
                intervention_thread = threading.Thread(
                    target=lambda: emergency_intervention(v, target_data, mesh),
                    daemon=True
                )
                intervention_thread.start()
                
                mesh.send({
                    "type": "ack",
                    "for": "tespit_hedef_ortalandi",
                    "status": "intervention_started",
                    "drone_id": DRONE_ID
                })
        
        elif t == 'abort_mission':
            # Görevi iptal et
            if v and v.armed:
                print(f"[RESCUE-UAV{DRONE_ID}] Görev iptal edildi, RTL moduna geçiyor...")
                v.mode = VehicleMode("RTL")
                intervention_active = False
        
        elif t == 'emergency_land':
            # Acil iniş
            if v and v.armed:
                print(f"[RESCUE-UAV{DRONE_ID}] ACİL İNİŞ!")
                v.mode = VehicleMode("LAND")
                intervention_active = False
                break
        
        elif t == 'ui_land' or t == 'land':
            # Normal iniş komutu
            if v and v.armed:
                print(f"[RESCUE-UAV{DRONE_ID}] İniş komutu alındı")
                v.mode = VehicleMode("LAND")
            break

    running[0] = False
    time.sleep(0.4)
    mesh.close()
    if v:
        v.close()
    print(f"[RESCUE-UAV{DRONE_ID}] Sistem kapatıldı.")

if __name__ == '__main__':
    main()
