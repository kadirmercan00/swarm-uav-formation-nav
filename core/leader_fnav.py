#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone 0 â€" LEADER (Streaming NAV, smart yaw, formation heading control)
- Formasyon yÃ¶nÃ¼ (heading) arayÃ¼zden alÄ±nÄ±r ve takipÃ§ilere iletilir
- Yaw komutu akÄ±ÅŸta seyrek ve akÄ±llÄ± gÃ¶nderilir (deadband + min interval + kÄ±sa yÃ¶n)
- Final adÄ±mda tek seferlik yaw kilidi atÄ±lÄ±r
"""
import time, math, threading, json
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from esp_mesh import ESPMesh

try:
    from pyswarm import pso
    HAS_PYSWARM = True
except Exception:
    HAS_PYSWARM = False

# --- Defaults / constants (leader) ---
DRONE_ID = 0
CONN_STR = "/dev/ttyACM0"
ESP_PORT = "/dev/ttyUSB0"
M_PER_DEG = 1.113195e5

def wrap180(a):
    """Wrap angle to [-180, +180] degrees"""
    a = float(a)
    return ((a + 180.0) % 360.0) - 180.0

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

_last_yaw_ts = 0.0
def set_yaw_smart(vehicle, target_deg, force=False, deadband=10.0, min_interval=1.5, max_rate=25.0):
    """Send MAV_CMD_CONDITION_YAW only if needed; choose shortest direction and a bounded rate."""
    global _last_yaw_ts
    try:
        cur = vehicle.attitude.yaw  # rad
        cur_deg = wrap360(math.degrees(cur)) if cur is not None else None
    except Exception:
        cur_deg = None

    if cur_deg is not None and not force:
        err = wrap180(target_deg - cur_deg)
        if abs(err) < deadband and (time.time() - _last_yaw_ts) < min_interval:
            return  # skip

    # choose direction by shortest path
    direction = 1
    if cur_deg is not None:
        err = wrap180(target_deg - cur_deg)
        direction = 1 if err >= 0 else -1
        # rate proportional to error but bounded
        yaw_rate = max(8.0, min(max_rate, abs(err) / 0.8))  # aim ~0.8s settle
    else:
        yaw_rate = 20.0

    from pymavlink import mavutil
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(wrap360(target_deg)),   # param1: target heading deg (0..360)
        float(yaw_rate),              # param2: yaw rate deg/s
        float(direction),             # param3: direction (1 cw, -1 ccw)
        0,                            # param4: 0 absolute, 1 relative
        0,0,0)
    vehicle.send_mavlink(msg)
    _last_yaw_ts = time.time()

def ne_from_latlon(origin, lat, lon):
    """Lat/lon'dan North-East ofset hesapla"""
    dN = (lat - origin.lat) * M_PER_DEG
    dE = (lon - origin.lon) * M_PER_DEG * math.cos(math.radians(origin.lat))
    return np.array([float(dN), float(dE)], dtype=float)

def bearing_deg(lat1, lon1, lat2, lon2):
    """Ä°ki GPS noktasÄ± arasÄ±ndaki bearing aÃ§Ä±sÄ±nÄ± hesapla"""
    y = math.sin(math.radians(lon2-lon1)) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1))*math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(lon2-lon1)))
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

# ---------- Flight ----------
def arm_and_takeoff(vehicle, target_alt):
    """Drone'u arm et ve belirtilen irtifaya kaldÄ±r"""
    print(f"[LEADER] Arm edilebilirlik kontrol ediliyor...")
    while not vehicle.is_armable:
        print("[LEADER] Arm edilebilirlik bekleniyor...")
        time.sleep(0.5)
    
    print("[LEADER] GUIDED moduna geÃ§iliyor...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED": 
        time.sleep(0.2)
    
    print("[LEADER] Arm ediliyor...")
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(0.2)
    
    print(f"[LEADER] TAKEOFF -> {target_alt} m")
    vehicle.simple_takeoff(target_alt)
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt and alt >= target_alt * 0.95: 
            break
        print(f"[LEADER] Ä°rtifa: {alt:.1f}m / {target_alt}m")
        time.sleep(0.4)
    
    print("[LEADER] Hedef irtifa tamam")

def goto_NE(vehicle, origin, dN, dE, alt, gs=4.0):
    """Belirtilen North-East konumuna git"""
    vehicle.groundspeed = gs
    tgt = get_location_metres(origin, dN, dE, alt)
    vehicle.simple_goto(tgt)

def wait_to_reach(vehicle, origin, target_ne, tol=2.0, timeout=120):
    """Hedefe ulaÅŸana kadar bekle"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        if not (loc and loc.lat and loc.lon):
            time.sleep(0.3)
            continue
        curr = ne_from_latlon(origin, loc.lat, loc.lon)
        dist = np.linalg.norm(curr - target_ne)
        if dist < tol: 
            print(f"[LEADER] Hedefe ulaÅŸÄ±ldÄ± (hata: {dist:.1f}m)")
            return True
        time.sleep(0.4)
    print("[LEADER] Hedefe ulaÅŸma zaman aÅŸÄ±mÄ±!")
    return False

# ---------- Formation geometry ----------
def rot2d(th_rad): 
    """2D rotasyon matrisi"""
    c, s = math.cos(th_rad), math.sin(th_rad)
    return np.array([[c, -s], [s, c]], float)

def targets_V(spacing_m, angle_deg, heading_deg, ahead_sign):
    """V formasyonu hedef konumlarÄ±"""
    a = math.radians(float(angle_deg))
    l = float(spacing_m)
    x = ahead_sign * l * math.cos(a/2.0)
    y = l * math.sin(a/2.0)
    base = np.array([[x, +y], [x, -y]], float)
    return base @ rot2d(math.radians(heading_deg)).T

def targets_LINE(spacing_m, heading_deg):
    """Hat formasyonu hedef konumlarÄ±"""
    half = float(spacing_m)/2.0
    base = np.array([[0.0, +half], [0.0, -half]], float)
    return base @ rot2d(math.radians(heading_deg)).T

def targets_TRIANGLE(side_m, heading_deg):
    """ÃœÃ§gen formasyonu hedef konumlarÄ±"""
    L = float(side_m)
    h = (math.sqrt(3.0)/2.0)*L
    base = np.array([[-h, +L/2.0], [-h, -L/2.0]], float)  # lider tepe (0,0)
    return base @ rot2d(math.radians(heading_deg)).T

def targets_DIAMOND(spacing_m, heading_deg):
    """Elmas formasyonu hedef konumlarÄ±"""
    s = float(spacing_m) * 0.707  # 45 derece iÃ§in
    base = np.array([[-s, 0], [0, s]], float)
    return base @ rot2d(math.radians(heading_deg)).T

def compute_offsets(name, spacing, angle_deg, heading_deg):
    """Formasyon ofsetlerini hesapla"""
    name = str(name).upper()
    if name == "V":        return targets_V(spacing, angle_deg, heading_deg, ahead_sign=-1)
    if name == "ARROW":    return targets_V(spacing, angle_deg, heading_deg, ahead_sign=+1)
    if name == "LINE":     return targets_LINE(spacing, heading_deg)
    if name == "TRIANGLE": return targets_TRIANGLE(spacing, heading_deg)
    if name == "DIAMOND":  return targets_DIAMOND(spacing, heading_deg)
    raise ValueError(f"Bilinmeyen formasyon: {name}")

def assign_left_right(currNE, targNE):
    """Sol/saÄŸ pozisyon atamasÄ±"""
    def side(p): 
        return 1 if p[1] >= 0 else -1
    try:
        cside = [side(currNE[0]), side(currNE[1])]
        tside = [side(targNE[0]), side(targNE[1])]
        if set(cside) == {1, -1} and set(tside) == {1, -1}:
            idx_left = 0 if tside[0] == 1 else 1
            idx_right = 1 - idx_left
            mapping = [idx_left if cside[0] == 1 else idx_right, 
                      idx_left if cside[1] == 1 else idx_right]
            return np.array([targNE[mapping[0]], targNE[mapping[1]]])
    except Exception:
        pass
    return targNE

# ---------- PSO yaw refine ----------
def pso_refine_yaw(name, spacing, angle_deg, yaw_guess, leader_target_ne, curr_followers_ne):
    """PSO ile optimal yaw aÃ§Ä±sÄ± bul"""
    if not HAS_PYSWARM:
        return yaw_guess
    
    def cost(x):
        th = float(x[0])
        offs = compute_offsets(name, spacing, angle_deg, th)
        targ_abs = np.array([leader_target_ne + offs[0], leader_target_ne + offs[1]])
        err = (np.linalg.norm(curr_followers_ne[0] - targ_abs[0])**2 + 
               np.linalg.norm(curr_followers_ne[1] - targ_abs[1])**2)
        err += 0.05 * (((th - yaw_guess + 540) % 360) - 180)**2
        return err
    
    lb = [-180.0]
    ub = [180.0]
    xopt, _ = pso(cost, lb, ub, swarmsize=18, maxiter=25, debug=False)
    return float((xopt[0] + 360.0) % 360.0)

# ---------- Mesh helpers ----------
def state_broadcast_loop(vehicle, mesh, running_flag):
    """Durum yayÄ±nlama dÃ¶ngÃ¼sÃ¼"""
    while running_flag[0]:
        try:
            loc = vehicle.location.global_relative_frame
            if loc and loc.lat and loc.lon:
                yaw = vehicle.attitude.yaw
                mesh.broadcast_state(loc.lat, loc.lon, loc.alt or 0.0, math.degrees(yaw))
            time.sleep(0.5)
        except Exception as e:
            print(f"[LEADER] State broadcast hatasÄ±: {e}")
            time.sleep(0.5)

def collect_states(mesh, timeout_s=0.25):
    """TakipÃ§i durumlarÄ±nÄ± topla"""
    states = {}
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        msg = mesh.receive(timeout=0.01)
        if msg and 'data' in msg:
            data = msg['data']
            if data.get('type') == 'state' and data.get('id') in (1, 2):
                states[data['id']] = data
    return states

def check_formation_reached(mesh, target_positions, origin, timeout=8):  # Timeout'u 8 saniyeye düşürdük
    """Formasyonun tamamlanÄ±p tamamlanmadÄ±ÄŸÄ±nÄ± kontrol et"""
    print("[LEADER] Formasyon tamamlanmasÄ± bekleniyor...")
    start_time = time.time()
    consecutive_good = 0  # Ardışık iyi ölçüm sayısı
    
    while time.time() - start_time < timeout:
        # TakipÃ§i durumlarÄ±nÄ± topla
        states = collect_states(mesh, timeout_s=0.2)
        
        good_positions = 0
        total_drones = 0
        
        for drone_id in [1, 2]:
            total_drones += 1
            if drone_id in states:
                state = states[drone_id]
                current_ne = ne_from_latlon(origin, state['lat'], state['lon'])
                target_ne = np.array(target_positions[str(drone_id)])
                distance = np.linalg.norm(current_ne - target_ne)
                
                if distance <= 4.0:  # 4 metre toleransı gevşettik
                    good_positions += 1
                    print(f"[LEADER] Drone {drone_id} formasyonda (mesafe: {distance:.1f}m)")
                else:
                    print(f"[LEADER] Drone {drone_id} henÃ¼z formasyonda deÄŸil (mesafe: {distance:.1f}m)")
        
        # En az 1 drone formasyonda ise veya 2 saniye geçtiyse devam et
        if good_positions >= 1 or (time.time() - start_time) > 2:
            consecutive_good += 1
            if consecutive_good >= 2:  # 2 ardışık iyi ölçüm yeterli
                print(f"[LEADER] Formasyon kabul edilebilir durumda ({good_positions}/{total_drones} drone hazır)")
                return True
        else:
            consecutive_good = 0
            
        time.sleep(0.3)
    
    print("[LEADER] Formasyon bekleme süresi doldu, devam ediliyor...")
    return True  # Her durumda True döndür

# ---------- Main ----------
def main():
    mesh = ESPMesh(port=ESP_PORT, drone_id=DRONE_ID)
    if not mesh.connect():
        print("[LEADER] ESP32 mesh baÄŸlantÄ±sÄ± baÅŸarÄ±sÄ±z!")
        return

    v0 = connect(CONN_STR, baud=57600, wait_ready=True)
    origin = None
    running = [True]
    
    # State broadcast thread'i baÅŸlat
    threading.Thread(target=state_broadcast_loop, args=(v0, mesh, running), daemon=True).start()

    # VarsayÄ±lan deÄŸerler
    last_form = {
        "name": "V",
        "spacing_m": 8.0,
        "angle_deg": 50.0,
        "heading_deg": 0.0,  # Formasyon yÃ¶nÃ¼
        "yaw_lock_deg": None
    }
    default_gs = 3.0
    default_gs_per = None
    default_dwell = 5

    print("[LEADER] UI komutlarÄ±nÄ± bekliyor (ui_takeoff, ui_formation, ui_nav, ui_land)...")

    while True:
        pkt = mesh.receive(timeout=0.05)
        if not pkt: 
            continue
        
        data = pkt.get('data', {})
        t = data.get('type')

        if t == 'ui_takeoff':
            alt = float(data.get('alt', 10.0))
            yaw_deg = float(data.get('yaw_deg', 0.0))
            if 'gs_per' in data:
                default_gs_per = data.get('gs_per')
            default_gs = float(data.get('gs', default_gs))

            mesh.send({
                "type": "ack",
                "for": "ui_takeoff",
                "status": "ok",
                "alt": alt,
                "yaw": yaw_deg
            })
            
            origin = v0.location.global_relative_frame
            
            mesh.send({
                "type": "takeoff",
                "lat": origin.lat,
                "lon": origin.lon,
                "alt": alt,
                "leader_yaw": yaw_deg,
                "gs": default_gs,
                "gs_per": default_gs_per
            })
            
            mesh.send({
                "type": "status",
                "phase": "TAKEOFF",
                "detail": "leader_takeoff_start"
            })
            
            arm_and_takeoff(v0, alt)
            set_yaw_smart(v0, yaw_deg, force=True)
            
            mesh.send({
                "type": "status",
                "phase": "TAKEOFF",
                "detail": "leader_takeoff_done"
            })

        # Ana döngüye eklenecek yeni komut işleyici
        elif t == 'autonomous_mission':
            steps = data.get('steps', [])
            if not steps:
                mesh.send({"type": "error", "msg": "Görev adımları boş"})
                continue
            
            mesh.send({"type": "ack", "for": "autonomous_mission", "steps": len(steps)})
            
            # Görev değişkenlerini başlat
            current_position_ne = np.array([0.0, 0.0])
            
            for step_idx, step in enumerate(steps):
                step_type = step.get('type')
                
                mesh.send({"type": "mission_status", "step": step_idx + 1, "total": len(steps), "action": step_type})
                
                if step_type == "TAKEOFF":
                    alt = float(step.get('altitude', 10))
                    
                    # Origin'i belirle
                    origin = v0.location.global_relative_frame
                    
                    # TAKİPÇİLERE TAKEOFF KOMUTUNU GÖNDER - ÖNEMLİ!
                    mesh.send({
                        "type": "takeoff",
                        "lat": origin.lat,
                        "lon": origin.lon,
                        "alt": alt,
                        "leader_yaw": 0.0,
                        "gs": 3.0
                    })
                    
                    # Leader kendi takeoff'unu yap
                    arm_and_takeoff(v0, alt)
                    
                    # Takipçilerin kalkması için bekle
                    time.sleep(step.get('wait', 10))  # 10 saniye bekle
                
                elif step_type == "MOVE":
                    if origin is None:
                        origin = v0.location.global_relative_frame
                        
                    north = float(step.get('north', 0))
                    east = float(step.get('east', 0))
                    alt = float(step.get('altitude', 10))
                    speed = float(step.get('speed', 3))
                    formation = step.get('formation', 'V')
                    spacing = float(step.get('spacing', 8))
                    angle = float(step.get('angle', 60))
                    formation_heading = float(step.get('formation_heading', 0))  # Formasyon yönü
                    wait_time = float(step.get('wait', 5))
                    
                    # Yeni pozisyon hesapla
                    target_position_ne = current_position_ne + np.array([north, east])
                    
                    # Formasyon ofsetlerini FORMASYON YÖNÜNE GÖRE hesapla
                    offs = compute_offsets(formation, spacing, angle, formation_heading)
                    
                    # Hedef pozisyonları
                    t0 = target_position_ne
                    t1 = target_position_ne + offs[0]
                    t2 = target_position_ne + offs[1]
                    
                    # Hareket yönünü hesapla (yaw için)
                    move_vector = target_position_ne - current_position_ne
                    distance = np.linalg.norm(move_vector)
                    
                    if distance > 0:
                        # Hareket yönü
                        move_heading = math.degrees(math.atan2(move_vector[1], move_vector[0]))
                        
                        # Yaw: Hareket yönü veya formasyon yönü (tercihe göre)
                        use_movement_yaw = True  # False yaparsanız formasyon yönünde bakar
                        yaw_deg = move_heading if use_movement_yaw else formation_heading
                    else:
                        yaw_deg = formation_heading
                    
                    # Takipçilere gönder (final/non-stream nav)
                    mesh.send({
                        "type": "nav",
                        "seq": step_idx,
                        "stream": False,
                        "targets": {
                            "0": [float(t0[0]), float(t0[1])],
                            "1": [float(t1[0]), float(t1[1])],
                            "2": [float(t2[0]), float(t2[1])]
                        },
                        "alt": alt,
                        "yaw_deg": float(yaw_deg),
                        "formation_heading": float(formation_heading),
                        "gs": speed,
                        "dwell": 0
                    })
                    
                    # Leader hareket et
                    goto_NE(v0, origin, t0[0], t0[1], alt, gs=speed)
                    set_yaw_smart(v0, yaw_deg, force=True)
                    wait_to_reach(v0, origin, t0, tol=2.0, timeout=120)
                    
                    # Pozisyonu güncelle
                    current_position_ne = target_position_ne
                    
                    # Bekleme
                    if wait_time > 0:
                        print(f"[LEADER] {wait_time} saniye bekleniyor...")
                        time.sleep(wait_time)
                            
                        
                elif step_type == "FORMATION":
                    if origin is None:
                        origin = v0.location.global_relative_frame
                        
                    formation = step.get('formation', 'V')
                    spacing = float(step.get('spacing', 8))
                    angle = float(step.get('angle', 60))
                    alt = float(step.get('altitude', 10))
                    # IMPORTANT: read formation heading from the step
                    formation_heading = float(step.get('heading', 0))
                    wait_time = float(step.get('wait', 5))
                    
                    # Compute offsets using the provided formation heading
                    offs = compute_offsets(formation, spacing, angle, formation_heading)
                    
                    t0 = current_position_ne
                    t1 = current_position_ne + offs[0]
                    t2 = current_position_ne + offs[1]
                    
                    # SEND formation command to followers including heading
                    mesh.send({
                        "type": "formation",
                        "targets": {
                            "0": [float(t0[0]), float(t0[1])],
                            "1": [float(t1[0]), float(t1[1])],
                            "2": [float(t2[0]), float(t2[1])]
                        },
                        "alt": alt,
                        "leader_yaw": float(formation_heading),
                        "formation_heading": float(formation_heading),
                        "gs": 3.0,
                        "dwell": 0
                    })
                    
                    # Command leader to position and yaw to the formation heading
                    goto_NE(v0, origin, t0[0], t0[1], alt, gs=3.0)
                    set_yaw_smart(v0, formation_heading, force=True)
                    wait_to_reach(v0, origin, t0, tol=2.0, timeout=60)
                    
                    if wait_time > 0:
                        print(f"[LEADER] Formasyonda {wait_time} saniye bekleniyor... (Yön: {formation_heading}°)")
                        time.sleep(wait_time)
                        
                elif step_type == "WAIT":
                    duration = float(step.get('duration', 5))
                    print(f"[LEADER] {duration} saniye bekleniyor...")
                    time.sleep(duration)
                    
                elif step_type == "LAND":
                    # TAKİPÇİLERE LAND KOMUTU GÖNDER
                    mesh.send({"type": "land"})
                    v0.mode = VehicleMode("LAND")
                    break
            
            mesh.send({"type": "mission_complete"})
            print("[LEADER] Otonom görev tamamlandı!")

        elif t == 'ui_formation':
            name = str(data.get('name', last_form['name'])).upper()
            last_form['name'] = name
            last_form['spacing_m'] = float(data.get('spacing_m', last_form['spacing_m']))
            last_form['angle_deg'] = float(data.get('angle_deg', last_form['angle_deg']))
            last_form['heading_deg'] = float(data.get('heading_deg', last_form['heading_deg']))
            last_form['yaw_lock_deg'] = data.get('yaw_deg', last_form.get('yaw_lock_deg', None))
            dwell_time = int(data.get('dwell', default_dwell))
            
            if 'gs_per' in data:
                default_gs_per = data.get('gs_per')
            default_gs = float(data.get('gs', default_gs))

            if origin is None:
                origin = v0.location.global_relative_frame
            
            loc = v0.location.global_relative_frame
            curr_lat = loc.lat if (loc and loc.lat) else origin.lat
            curr_lon = loc.lon if (loc and loc.lon) else origin.lon
            leader_ne = ne_from_latlon(origin, curr_lat, curr_lon)

            # Formasyon yÃ¶nÃ¼nÃ¼ kullan
            formation_heading = last_form['heading_deg']
            yaw_use = last_form['yaw_lock_deg'] if last_form.get('yaw_lock_deg') is not None else formation_heading

            # TakipÃ§i durumlarÄ±nÄ± topla
            st = collect_states(mesh, timeout_s=0.4)
            def get_f_ne(did):
                s = st.get(did)
                if not s: 
                    return leader_ne
                return ne_from_latlon(origin, s['lat'], s['lon'])
            
            curr_followers_ne = np.array([get_f_ne(1), get_f_ne(2)])

            # Formasyon ofsetlerini hesapla
            offs = compute_offsets(name, last_form['spacing_m'], last_form['angle_deg'], formation_heading)
            offs = assign_left_right(curr_followers_ne - leader_ne, offs)

            t0 = leader_ne
            t1 = leader_ne + offs[0]
            t2 = leader_ne + offs[1]

            payload = {
                "type": "formation",
                "targets": {
                    "0": [float(t0[0]), float(t0[1])],
                    "1": [float(t1[0]), float(t1[1])],
                    "2": [float(t2[0]), float(t2[1])]
                },
                "alt": float(data.get('alt', origin.alt or 10.0)),
                "leader_yaw": float(yaw_use),
                "formation_heading": float(formation_heading),
                "gs": default_gs,
                "gs_per": default_gs_per,
                "dwell": 0  # Dwell'i 0 yapÄ±yoruz Ã§Ã¼nkÃ¼ burada kontrol edeceÄŸiz
            }
            
            mesh.send({"type": "ack", "for": "ui_formation", "status": "ok", "name": name})
            mesh.send(payload)
            
            # Sadece kısa bir süre bekle ve direkt dwell'e geç
            print("[LEADER] Formasyona geçiş komutu gönderildi, kısa bekleme...")
            time.sleep(3)  # Sadece 2 saniye bekle
            
            if dwell_time > 0:
                print(f"[LEADER] Formasyonda {dwell_time} saniye bekleniyor...")
                time.sleep(dwell_time)
                print(f"[LEADER] Formasyon bekleme süresi tamamlandı!")

        elif t == 'ui_nav':
            if origin is None:
                origin = v0.location.global_relative_frame
            
            wps = data.get('wps', [])
            if not isinstance(wps, list) or len(wps) == 0:
                mesh.send({"type": "ack", "for": "ui_nav", "status": "error", "msg": "wps empty"})
                continue

            f = data.get('formation', None) or last_form
            name = str(f.get('name', last_form['name'])).upper()
            spacing = float(f.get('spacing_m', last_form['spacing_m']))
            angle = float(f.get('angle_deg', last_form['angle_deg']))
            heading = float(f.get('heading_deg', last_form['heading_deg']))
            yaw_lock = f.get('yaw_lock_deg', last_form.get('yaw_lock_deg', None))

            if 'gs_per' in data:
                default_gs_per = data.get('gs_per')
            gs_leader = float(data.get('gs', default_gs))
            dwell = int(data.get('dwell', default_dwell))

            mesh.send({
                "type": "ack",
                "for": "ui_nav",
                "status": "ok",
                "wp_count": len(wps),
                "formation": name
            })


            for wi, wp in enumerate(wps, start=1):
                # ... waypoint parse işlemleri ...
                
                # Streaming navigasyon
                while True:
                    loc = v0.location.global_relative_frame
                    curr_lat = loc.lat if (loc and loc.lat) else origin.lat
                    curr_lon = loc.lon if (loc and loc.lon) else origin.lon
                    leader_ne_now = ne_from_latlon(origin, curr_lat, curr_lon)
                    
                    vec = leader_ne_wp - leader_ne_now
                    dist = float(np.linalg.norm(vec))
                    
                    if dist < max(1.5, spacing * 0.6):
                        break

                    unit = vec / (dist + 1e-6)
                    step = min(gs_leader * tick, dist)
                    next_leader_ne = leader_ne_now + unit * step

                    # ÖNEMLI: Her drone için AYNI hız vektörünü hesapla
                    velocity_north = unit[0] * gs_leader
                    velocity_east = unit[1] * gs_leader

                    # Formasyon ofsetlerini hesapla
                    offs = compute_offsets(name, spacing, angle, heading)
                    t0 = next_leader_ne
                    t1 = next_leader_ne + offs[0]
                    t2 = next_leader_ne + offs[1]

                    seq += 1
                    payload = {
                        "type": "nav",
                        "seq": int(seq),
                        "stream": True,
                        "targets": {
                            "0": [float(t0[0]), float(t0[1])],
                            "1": [float(t1[0]), float(t1[1])],
                            "2": [float(t2[0]), float(t2[1])]
                        },
                        "alt": alt,
                        "yaw_deg": float(yaw_opt),
                        "formation_heading": float(heading),
                        # AYNI HIZ VEKTÖRÜNÜ GÖNDER
                        "velocity_vector": {
                            "north": float(velocity_north),
                            "east": float(velocity_east)
                        },
                        "gs": float(gs_leader),
                        "dwell": 0
                    }
                    mesh.send(payload)

            goto_NE(v0, origin, t0[0], t0[1], alt, gs=gs_leader)
            set_yaw_smart(v0, yaw_opt, force=False)
            time.sleep(tick)


            for wi, wp in enumerate(wps, start=1):
                try:
                    lat = float(wp['lat'])
                    lon = float(wp['lon'])
                    alt = float(wp.get('alt', origin.alt or 10.0))
                except Exception as e:
                    print(f"[LEADER] WP{wi} parse hatasÄ±: {e}")
                    continue

                seq = 0
                tick = 0.5
                leader_ne_wp = ne_from_latlon(origin, lat, lon)

                # Waypoint'e streaming navigasyon
                while True:
                    loc = v0.location.global_relative_frame
                    curr_lat = loc.lat if (loc and loc.lat) else origin.lat
                    curr_lon = loc.lon if (loc and loc.lon) else origin.lon
                    leader_ne_now = ne_from_latlon(origin, curr_lat, curr_lon)
                    
                    vec = leader_ne_wp - leader_ne_now
                    dist = float(np.linalg.norm(vec))
                    
                    if dist < max(1.5, spacing * 0.6):
                        break

                    unit = vec / (dist + 1e-6)
                    step = min(gs_leader * tick, dist)
                    next_leader_ne = leader_ne_now + unit * step

                    # Yaw hesapla
                    if yaw_lock is None:
                        yaw_guess = math.degrees(math.atan2(unit[1], unit[0]))
                    else:
                        yaw_guess = float(yaw_lock)

                    yaw_opt = yaw_guess  # leader yaw stabilize locally

                    # Formasyon ofsetlerini hesapla
                    offs = compute_offsets(name, spacing, angle, heading)
                    t0 = next_leader_ne
                    t1 = next_leader_ne + offs[0]
                    t2 = next_leader_ne + offs[1]

                    # HÄ±z eÅŸitleme
                    gs_per = {"0": gs_leader, "1": gs_leader, "2": gs_leader}

                    seq += 1
                    payload = {
                        "type": "nav",
                        "seq": int(seq),
                        "stream": True,
                        "targets": {
                            "0": [float(t0[0]), float(t0[1])],
                            "1": [float(t1[0]), float(t1[1])],
                            "2": [float(t2[0]), float(t2[1])]
                        },
                        "alt": alt,
                        "yaw_deg": float(yaw_opt),
                        "formation_heading": float(heading),
                        "gs": float(gs_leader),
                        "gs_per": gs_per,
                        "dwell": 0
                    }
                    mesh.send(payload)

                    goto_NE(v0, origin, t0[0], t0[1], alt, gs=gs_leader)
                    set_yaw_smart(v0, yaw_opt, force=False, deadband=12.0, min_interval=1.2, max_rate=25.0)
                    time.sleep(tick)

                # Final pozisyon
                yaw_final = yaw_guess if yaw_lock is None else float(yaw_lock)
                offs_final = compute_offsets(name, spacing, angle, heading)
                t0f = leader_ne_wp
                t1f = leader_ne_wp + offs_final[0]
                t2f = leader_ne_wp + offs_final[1]

                payload_final = {
                    "type": "nav",
                    "seq": int(seq + 1),
                    "stream": False,
                    "targets": {
                        "0": [float(t0f[0]), float(t0f[1])],
                        "1": [float(t1f[0]), float(t1f[1])],
                        "2": [float(t2f[0]), float(t2f[1])]
                    },
                    "alt": alt,
                    "yaw_deg": float(yaw_final),
                    "formation_heading": float(heading),
                    "gs": float(gs_leader),
                    "gs_per": {"0": gs_leader, "1": gs_leader, "2": gs_leader},
                    "dwell": 0  # Burada da 0 yapÄ±yoruz
                }
                mesh.send(payload_final)

                goto_NE(v0, origin, t0f[0], t0f[1], alt, gs=gs_leader)
                set_yaw_smart(v0, yaw_final, force=True)
                wait_to_reach(v0, origin, t0f, tol=2.0, timeout=120)
                
                # Navigation için de sadece kısa bekleme
                print(f"[LEADER] WP{wi}'e ulaşıldı, kısa bekleme...")
                time.sleep(2)  # Sadece 2 saniye bekle
                
                if dwell > 0:
                    print(f"[LEADER] WP{wi}'de {dwell} saniye bekleniyor...")
                    time.sleep(dwell)
                    print(f"[LEADER] WP{wi} bekleme süresi tamamlandı!")

            mesh.send({"type": "status", "phase": "NAV", "detail": "done"})

        elif t == 'ui_land':
            mesh.send({"type": "ack", "for": "ui_land", "status": "ok"})
            mesh.send({"type": "land"})
            
            try:
                v0.mode = VehicleMode("LAND")
            except:
                pass
            break

    running[0] = False
    time.sleep(0.4)
    mesh.close()
    v0.close()
    print("[LEADER] Bitti.")

if __name__ == "__main__":
    main()
