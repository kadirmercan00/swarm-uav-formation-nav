#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone 2 – FOLLOWER (PyMAVLink Velocity Control + Smooth Path)
- Hedef noktaya velocity kontrolü ile smooth hareket
- SET_POSITION_TARGET_LOCAL_NED mesajı ile hassas hız kontrolü
- Potansiyel alan yaklaşımı ile çarpışma önleme
"""
import time, math, threading, json
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from esp_mesh import ESPMesh

DRONE_ID = 2
CONN_STR = "/dev/ttyACM0"
ESP_PORT = "/dev/ttyUSB0"
M_PER_DEG = 1.113195e5

# Navigasyonda formasyon yaw'ını uygula
USE_FORMATION_YAW_IN_NAV = True
YAW_DEADBAND_DEG = 3.0
_last_yaw_cmd_ts = 0.0
_min_yaw_cmd_dt = 0.4  # saniye (spam engelle)

# ------------- Velocity Control Parametreleri -------------
MAX_VELOCITY = 2.0           # m/s, maksimum hız
MIN_VELOCITY = 0.5           # m/s, minimum hız  
CRUISE_VELOCITY = 3.0        # m/s, seyir hızı
APPROACH_DISTANCE = 5.0      # m, yaklaşma mesafesi
VELOCITY_UPDATE_RATE = 10    # Hz, hız güncelleme frekansı
POSITION_TOLERANCE = 1.5     # m, pozisyon toleransı

# Potansiyel Alan Parametreleri
ATTRACTIVE_GAIN = 1.0        # Hedef çekim kuvveti
REPULSIVE_GAIN = 8.0        # İtme kuvveti kazancı
INFLUENCE_DISTANCE = 5.0     # m, etki mesafesi
CRITICAL_DISTANCE = 3.0      # m, kritik mesafe

# Path Smoothing Parametreleri
PATH_SMOOTHING_FACTOR = 0.7  # Yol yumuşatma faktörü
VELOCITY_SMOOTHING = 0.8     # Hız yumuşatma faktörü

# ------------- Yardımcılar -------------
def wrap180(a): 
    return (a + 180.0) % 360.0 - 180.0

def wrap360(a): 
    return (a % 360.0 + 360.0) % 360.0

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

def send_velocity_ned(vehicle, vn, ve, vd, yaw_rate=0):
    """
    NED frame'de velocity komutu gönder
    vn: North velocity (m/s)
    ve: East velocity (m/s)
    vd: Down velocity (m/s) - pozitif aşağı
    yaw_rate: Yaw dönüş hızı (rad/s)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (kullanılmıyor)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (sadece velocity kullan)
        0, 0, 0,  # x, y, z pozisyonları (kullanılmıyor)
        float(vn), float(ve), float(vd),  # x, y, z velocity
        0, 0, 0,  # x, y, z acceleration (kullanılmıyor)
        0, float(yaw_rate))  # yaw, yaw_rate
    
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_position_target_local_ned(vehicle, north, east, down, vn=0, ve=0, vd=0):
    """
    Hem pozisyon hem velocity hedefi gönder
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # type_mask (pozisyon ve velocity kullan)
        float(north), float(east), float(down),  # pozisyon
        float(vn), float(ve), float(vd),  # velocity
        0, 0, 0,  # acceleration
        0, 0)     # yaw, yaw_rate
    
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_yaw_rate(vehicle, yaw_rate_deg_s):
    """Yaw dönüş hızı ayarla (derece/saniye)"""
    yaw_rate_rad = math.radians(yaw_rate_deg_s)
    send_velocity_ned(vehicle, 0, 0, 0, yaw_rate_rad)

def set_yaw_target(vehicle, target_deg):
    """Hedef yaw açısı ayarla"""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(wrap360(target_deg)),  # yaw angle
        20.0,  # yaw speed deg/s
        1,     # direction (1: shortest)
        0,     # relative (0: absolute)
        0, 0, 0)
    vehicle.send_mavlink(msg)

def maybe_set_yaw(v, target_yaw_deg):
    global _last_yaw_cmd_ts
    now = time.time()
    if now - _last_yaw_cmd_ts < _min_yaw_cmd_dt:
        return
    try:
        cur_yaw = math.degrees(v.attitude.yaw)
    except Exception:
        cur_yaw = None
    if cur_yaw is None or abs(((target_yaw_deg - cur_yaw + 540) % 360) - 180) > YAW_DEADBAND_DEG:
        set_yaw_target(v, target_yaw_deg)
        _last_yaw_cmd_ts = now

def connect_vehicle(conn_str):
    print(f"[UAV{DRONE_ID}] {conn_str} adresine bağlanılıyor...")
    v = connect(conn_str, baud=57600, wait_ready=True)
    v._label = f"UAV{DRONE_ID}"
    print(f"[UAV{DRONE_ID}] Bağlantı başarılı!")
    return v

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
    
    t0 = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None and alt >= target_alt * 0.95: 
            break
        if time.time() - t0 > 90: 
            print(f"[UAV{DRONE_ID}] Takeoff zaman aşımı!")
            break
        print(f"[UAV{DRONE_ID}] İrtifa: {alt:.1f}m / {target_alt}m")
        time.sleep(0.4)
    
    print(f"[UAV{DRONE_ID}] İrtifa tamam.")

def calculate_potential_field(current_ne, target_ne, obstacles, origin):
    """
    Potansiyel alan yaklaşımı ile hız vektörü hesapla
    Attractive + Repulsive forces
    """
    # Attractive force (hedef çekimi)
    to_target = target_ne - current_ne
    dist_to_target = np.linalg.norm(to_target)
    
    if dist_to_target < POSITION_TOLERANCE:
        return np.zeros(2), True  # Hedefe ulaştık
    
    # Attractive velocity
    if dist_to_target > APPROACH_DISTANCE:
        attractive_vel = (to_target / dist_to_target) * CRUISE_VELOCITY * ATTRACTIVE_GAIN
    else:
        # Hedefe yaklaşırken yavaşla
        speed = MIN_VELOCITY + (CRUISE_VELOCITY - MIN_VELOCITY) * (dist_to_target / APPROACH_DISTANCE)
        attractive_vel = (to_target / dist_to_target) * speed * ATTRACTIVE_GAIN
    
    # Repulsive forces (engellerden kaçış)
    repulsive_vel = np.zeros(2)
    
    for obs in obstacles:
        if obs.get('lat') and obs.get('lon'):
            obs_ne = ne_from_latlon(origin, obs['lat'], obs['lon'])
            to_obs = obs_ne - current_ne
            dist_to_obs = np.linalg.norm(to_obs)
            
            if dist_to_obs < INFLUENCE_DISTANCE and dist_to_obs > 0.1:
                # İtme kuvveti hesapla
                if dist_to_obs < CRITICAL_DISTANCE:
                    # Kritik mesafede güçlü itme
                    repulsion_mag = REPULSIVE_GAIN * 2.0 * (1.0 - dist_to_obs / CRITICAL_DISTANCE)
                else:
                    # Normal itme
                    repulsion_mag = REPULSIVE_GAIN * (1.0 - dist_to_obs / INFLUENCE_DISTANCE)
                
                # İtme yönü (engellerden uzağa)
                repulsion_dir = -to_obs / dist_to_obs
                repulsive_vel += repulsion_dir * repulsion_mag
    
    # Toplam velocity
    total_vel = attractive_vel + repulsive_vel
    
    # Velocity limitleme
    vel_mag = np.linalg.norm(total_vel)
    if vel_mag > MAX_VELOCITY:
        total_vel = (total_vel / vel_mag) * MAX_VELOCITY
    elif vel_mag < MIN_VELOCITY and dist_to_target > POSITION_TOLERANCE:
        total_vel = (total_vel / vel_mag) * MIN_VELOCITY if vel_mag > 0 else attractive_vel
    
    return total_vel, False

def velocity_control_loop(v, origin, target_ne, alt, neighbor_states, stop_event):
    """
    Velocity control döngüsü - smooth hareket için
    """
    rate = 1.0 / VELOCITY_UPDATE_RATE
    last_velocity = np.zeros(2)
    
    while not stop_event.is_set():
        try:
            # Mevcut pozisyon
            loc = v.location.global_relative_frame
            if not loc or not loc.lat or not loc.lon:
                time.sleep(rate)
                continue
            
            current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
            current_alt = loc.alt or alt
            
            # Engel listesi (diğer drone'lar)
            obstacles = list(neighbor_states.values())
            
            # Potansiyel alan ile velocity hesapla
            velocity_ne, reached = calculate_potential_field(
                current_ne, target_ne, obstacles, origin
            )
            
            if reached:
                # Hedefe ulaştık, dur
                send_velocity_ned(v, 0, 0, 0)
                print(f"[UAV{DRONE_ID}] Hedefe ulaşıldı!")
                break
            
            # Velocity smoothing (ani değişimleri önle)
            velocity_ne = VELOCITY_SMOOTHING * velocity_ne + (1 - VELOCITY_SMOOTHING) * last_velocity
            last_velocity = velocity_ne
            
            # Altitude kontrolü
            alt_error = alt - current_alt
            vd = -np.clip(alt_error * 0.5, -1.0, 1.0)  # Down velocity (pozitif aşağı)
            
            # Velocity komutunu gönder
            send_velocity_ned(v, velocity_ne[0], velocity_ne[1], vd)
            
            # Yaw'ı hareket yönüne çevir
            if np.linalg.norm(velocity_ne) > 0.5:
                target_yaw = math.degrees(math.atan2(velocity_ne[1], velocity_ne[0]))
                set_yaw_target(v, target_yaw)
            
            # Debug
            dist_to_target = np.linalg.norm(target_ne - current_ne)
            print(f"[UAV{DRONE_ID}] Velocity: N={velocity_ne[0]:.1f} E={velocity_ne[1]:.1f} m/s, Mesafe: {dist_to_target:.1f}m")
            
        except Exception as e:
            print(f"[UAV{DRONE_ID}] Velocity control hatası: {e}")
        
        time.sleep(rate)
    
    # Durdur
    send_velocity_ned(v, 0, 0, 0)

def goto_ne_smooth(v, origin, target_ne, alt, neighbor_states, timeout=60):
    """
    Smooth velocity control ile hedefe git
    """
    stop_event = threading.Event()
    
    # Velocity control thread'i başlat
    control_thread = threading.Thread(
        target=velocity_control_loop,
        args=(v, origin, target_ne, alt, neighbor_states, stop_event)
    )
    control_thread.start()
    
    # Timeout veya hedefe ulaşana kadar bekle
    start_time = time.time()
    while time.time() - start_time < timeout:
        loc = v.location.global_relative_frame
        if loc and loc.lat and loc.lon:
            current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
            dist = np.linalg.norm(current_ne - target_ne)
            if dist < POSITION_TOLERANCE:
                break
        time.sleep(0.5)
    
    # Thread'i durdur
    stop_event.set()
    control_thread.join(timeout=2)
    
    # Son kontrol
    loc = v.location.global_relative_frame
    if loc and loc.lat and loc.lon:
        current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
        dist = np.linalg.norm(current_ne - target_ne)
        return dist < POSITION_TOLERANCE * 2
    return False

def state_broadcast_loop(v, mesh, running_flag):
    """Durum yayınlama döngüsü"""
    while running_flag[0]:
        try:
            loc = v.location.global_relative_frame
            if loc and loc.lat and loc.lon:
                yaw = v.attitude.yaw
                mesh.broadcast_state(loc.lat, loc.lon, loc.alt or 0.0, math.degrees(yaw))
            time.sleep(0.5)
        except Exception as e:
            print(f"[UAV{DRONE_ID}] State broadcast hatası: {e}")
            time.sleep(0.5)

def main():
    mesh = ESPMesh(port=ESP_PORT, drone_id=DRONE_ID)
    if not mesh.connect():
        print(f"[UAV{DRONE_ID}] ESP32 Mesh bağlantısı başarısız!")
        return

    v = connect_vehicle(CONN_STR)
    origin = None
    running = [True]
    state_thread = None
    leader_yaw = 0.0

    # Komşu state havuzu
    neighbor_states = {}  # id -> {lat, lon, alt, yaw, ts}

    print(f"[UAV{DRONE_ID}] Komutlar bekleniyor...")
    
    while True:
        msg = mesh.receive(timeout=0.02)
        if not msg or 'data' not in msg: 
            continue
        
        data = msg['data']
        t = data.get('type')
        sender = data.get('from', data.get('id', '?'))

        if t == 'state':
            sid = data.get('id', sender)
            if sid == DRONE_ID:
                continue  # kendimizi kaydetme
            
            neighbor_states[sid] = {
                "id": sid,
                "lat": data.get('lat'),
                "lon": data.get('lon'),
                "alt": data.get('alt'),
                "yaw": data.get('yaw'),
                "ts": time.time()
            }
            
            if sid == 0:
                leader_yaw = data.get('yaw', leader_yaw)

        elif t == 'takeoff':
            origin_lat = data.get('lat')
            origin_lon = data.get('lon')
            target_alt = data.get('alt', 10.0)
            leader_yaw = data.get('leader_yaw', leader_yaw)
            
            if origin_lat and origin_lon:
                class O: 
                    pass
                origin = O()
                origin.lat = origin_lat
                origin.lon = origin_lon
                origin.alt = target_alt
                
                arm_and_takeoff(v, target_alt)
                set_yaw_target(v, leader_yaw)
                
                if not state_thread:
                    state_thread = threading.Thread(
                        target=state_broadcast_loop, 
                        args=(v, mesh, running), 
                        daemon=True
                    )
                    state_thread.start()

        elif t == 'formation':
            if origin is None: 
                continue
            
            alt = data.get('alt', origin.alt)
            dwell = int(data.get('dwell', 0))
            
            target = data.get('targets', {}).get(str(DRONE_ID))
            if target is not None:
                dN, dE = float(target[0]), float(target[1])
                target_ne = np.array([dN, dE])
                
                print(f"[UAV{DRONE_ID}] Formation pozisyonuna velocity control ile gidiyor...")
                
                # Smooth velocity control ile git
                success = goto_ne_smooth(v, origin, target_ne, alt, neighbor_states, timeout=120)
                
                if success:
                    print(f"[UAV{DRONE_ID}] Formation pozisyonuna ulaşıldı")
                else:
                    print(f"[UAV{DRONE_ID}] Formation pozisyonuna ulaşılamadı")
                
                # Dwell süresini burada yapmıyoruz, leader kontrol edecek

        elif t == 'nav_velocity':
            # Liderle aynı hız vektörüyle git
            if origin is None: 
                continue
            
            alt = data.get('alt', origin.alt)
            velocity = data.get('velocity', {})
            vn = float(velocity.get('north', 0))
            ve = float(velocity.get('east', 0))
            duration = float(data.get('duration', 10))
            
            target = data.get('targets', {}).get(str(DRONE_ID))
            
            if target is not None:
                dN, dE = float(target[0]), float(target[1])
                target_ne = np.array([dN, dE])
                
                print(f"[UAV{DRONE_ID}] Lider hız vektörüyle hareket: vN={vn:.1f}, vE={ve:.1f} m/s")
                
                # Mevcut pozisyon
                loc = v.location.global_relative_frame
                if not loc or not loc.lat or not loc.lon:
                    continue
                    
                current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
                current_alt = loc.alt or alt
                
                # Hedef mesafesi
                distance_to_target = np.linalg.norm(target_ne - current_ne)
                
                # Altitude kontrolü
                alt_error = alt - current_alt
                vd = -np.clip(alt_error * 0.5, -1.0, 1.0)
                
                # Hareket başlat
                start_time = time.time()
                
                while time.time() - start_time < duration:
                    # Mevcut pozisyonu güncelle
                    loc = v.location.global_relative_frame
                    if loc and loc.lat and loc.lon:
                        current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
                        distance_to_target = np.linalg.norm(target_ne - current_ne)
                        
                        # Hedefe yakınsa hızı azalt
                        if distance_to_target < POSITION_TOLERANCE:
                            # Hedefe ulaştık, dur
                            send_velocity_ned(v, 0, 0, 0)
                            print(f"[UAV{DRONE_ID}] Hedefe ulaşıldı!")
                            break
                        elif distance_to_target < APPROACH_DISTANCE:
                            # Yaklaşma modunda, hızı orantılı azalt
                            scale = distance_to_target / APPROACH_DISTANCE
                            adjusted_vn = vn * scale
                            adjusted_ve = ve * scale
                            send_velocity_ned(v, adjusted_vn, adjusted_ve, vd)
                        else:
                            # Normal hızda devam et
                            send_velocity_ned(v, vn, ve, vd)
                        
                        # Altitude düzeltmesi
                        current_alt = loc.alt or alt
                        alt_error = alt - current_alt
                        vd = -np.clip(alt_error * 0.5, -1.0, 1.0)
                        
                        # Yaw tercihi: explicit/formasyon yaw varsa onu uygula, yoksa hız vektöründen
                        if USE_FORMATION_YAW_IN_NAV:
                            explicit_yaw = data.get('yaw_deg', data.get('formation_heading', data.get('leader_yaw', None)))
                            if explicit_yaw is not None:
                                maybe_set_yaw(v, float(explicit_yaw))
                            elif abs(vn) > 0.1 or abs(ve) > 0.1:
                                target_yaw = math.degrees(math.atan2(ve, vn))
                                maybe_set_yaw(v, target_yaw)
                        else:
                            if abs(vn) > 0.1 or abs(ve) > 0.1:
                                target_yaw = math.degrees(math.atan2(ve, vn))
                                set_yaw_target(v, target_yaw)
                    
                    time.sleep(0.1)  # 10Hz güncelleme
                
                # Hareket sonunda dur
                send_velocity_ned(v, 0, 0, 0)
                
                # Tam hedefe git (fine tuning)
                if distance_to_target > POSITION_TOLERANCE:
                    print(f"[UAV{DRONE_ID}] Hedef pozisyona ince ayar yapılıyor...")
                    goto_ne_smooth(v, origin, target_ne, alt, neighbor_states, timeout=30)
        elif t == 'nav':
            if origin is None: 
                continue
        
        default_alt = 10.0 if origin is None else getattr(origin, 'alt', 10.0)
        alt = data.get('alt', default_alt)
        stream = bool(data.get('stream', False))
        velocity_vector = data.get('velocity_vector', None)
        
        target = data.get('targets', {}).get(str(DRONE_ID))
        
        if target is not None:
            dN, dE = float(target[0]), float(target[1])
            target_ne = np.array([dN, dE])
            # FORMATION YAW uygula (öncelik: yaw_deg > formation_heading > leader_yaw)
            if USE_FORMATION_YAW_IN_NAV:
                target_yaw = (data.get('yaw_deg', data.get('formation_heading', data.get('leader_yaw', None))))
                if target_yaw is not None:
                    maybe_set_yaw(v, float(target_yaw))
            
            if stream and velocity_vector:
                # STREAMING MODDA: Liderle AYNI hız vektörünü kullan
                vn = float(velocity_vector.get('north', 0))
                ve = float(velocity_vector.get('east', 0))
                
                # Mevcut pozisyon
                loc = v.location.global_relative_frame
                if loc and loc.lat and loc.lon:
                    current_ne = ne_from_latlon(origin, loc.lat, loc.lon)
                    
                    # Hedefe olan mesafe
                    dist_to_target = np.linalg.norm(target_ne - current_ne)
                    
                    # Altitude düzeltmesi
                    current_alt = loc.alt or alt
                    alt_error = alt - current_alt
                    vd = -np.clip(alt_error * 0.5, -1.0, 1.0)
                    
                    # Eğer hedefe çok yakınsak dur
                    if dist_to_target < 1.0:
                        send_velocity_ned(v, 0, 0, 0)
                    else:
                        # LİDERLE AYNI HIZDA GİT
                        send_velocity_ned(v, vn, ve, vd)
                        
                        # Yaw'ı hareket yönüne çevir
                        if abs(vn) > 0.1 or abs(ve) > 0.1:
                            target_yaw = math.degrees(math.atan2(ve, vn))
                            set_yaw_target(v, target_yaw)
            else:
                # Non-stream modda hedefe git
                print(f"[UAV{DRONE_ID}] Final pozisyona gidiyor...")
                
                # Basit goto kullan (velocity control yerine)
                target_loc = get_location_metres(origin, dN, dE, alt)
                v.simple_goto(target_loc)
                v.groundspeed = data.get('gs', 3.0)


                    # Dwell süresini burada yapmıyoruz, leader kontrol edecek


        elif t == 'land':
            print(f"[UAV{DRONE_ID}] LAND komutu alındı")
            v.mode = VehicleMode('LAND')
            break

    running[0] = False
    time.sleep(0.4)
    mesh.close()
    v.close()
    print(f"[UAV{DRONE_ID}] Bağlantı kapatıldı.")

if __name__ == '__main__':
    main()
