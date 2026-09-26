#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 PainlessMesh Communication Module
ESP32 mesh network üzerinden JSON haberleşme
"""

import serial
import json
import time
import threading
from queue import Queue, Empty

class ESPMesh:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, drone_id=0):
        """
        port: ESP32'nin bağlı olduğu seri port
        baudrate: Seri haberleşme hızı
        drone_id: Drone numarası (0, 1, 2)
        """
        self.port = port
        self.baudrate = baudrate
        self.drone_id = drone_id
        self.serial = None
        self.rx_queue = Queue()
        self.tx_queue = Queue()
        self.running = False
        
    def connect(self):
        """ESP32 modülüne bağlan"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                
                
                rtscts=False,
                dsrdtr=False
            )
            self.serial.flush()
            self.running = True
            
            # RX/TX thread'lerini başlat
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()
            
            print(f"[ESP-Drone{self.drone_id}] {self.port} üzerinden bağlandı")
            
            # ESP'ye drone ID'sini bildir
            self.send({"type": "init", "drone_id": self.drone_id})
            
            return True
            
        except Exception as e:
            print(f"[ESP-Drone{self.drone_id}] Bağlantı hatası: {e}")
            return False
    
    def _rx_loop(self):
        """Gelen mesajları oku"""
        buffer = ""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Satır satır işle (JSON mesajlar \n ile ayrılmış)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_rx_line(line)
                            
                time.sleep(0.001)  # Daha hızlı okuma
                
            except Exception as e:
                print(f"[ESP-Drone{self.drone_id}] RX hatası: {e}")
                time.sleep(0.1)
    
    def _process_rx_line(self, line):
        """Gelen satırı işle"""
        try:
            # JSON mesaj olarak parse et
            msg_data = json.loads(line)
            
            # Kendi mesajımızı almamak için kontrol
            if msg_data.get("from") == self.drone_id:
                return
            
            # Mesajı kuyruğa ekle
            self.rx_queue.put({
                'data': msg_data,
                'timestamp': time.time()
            })
            
            msg_type = msg_data.get('type', '?')
            from_id = msg_data.get('from', '?')
            print(f"[ESP-Drone{self.drone_id}] RX <- Drone{from_id}: {msg_type}")
            
        except json.JSONDecodeError:
            # JSON değilse sistem mesajı olabilir
            if line.startswith("[ESP]") or line.startswith("[MESH]"):
                print(f"[ESP-Drone{self.drone_id}] System: {line}")
            else:
                print(f"[ESP-Drone{self.drone_id}] Unknown: {line}")
        except Exception as e:
            print(f"[ESP-Drone{self.drone_id}] Mesaj işleme hatası: {e}")
    
    def _tx_loop(self):
        """Gönderilecek mesajları işle"""
        while self.running:
            try:
                # Kuyruktaki mesajı al
                msg = self.tx_queue.get(timeout=0.1)
                
                # Mesaja drone ID ekle
                if isinstance(msg, dict):
                    msg["from"] = self.drone_id
                    if "ts" not in msg:
                        msg["ts"] = int(time.time())
                    json_str = json.dumps(msg, separators=(',', ':'))
                else:
                    json_str = str(msg)
                
                # JSON mesajı gönder
                tx_data = json_str + "\n"
                
                if self.serial:
                    self.serial.write(tx_data.encode('utf-8'))
                    self.serial.flush()
                    
                    msg_type = msg.get('type', '?') if isinstance(msg, dict) else 'raw'
                    print(f"[ESP-Drone{self.drone_id}] TX -> {msg_type}")
                
            except Empty:
                pass
            except Exception as e:
                print(f"[ESP-Drone{self.drone_id}] TX hatası: {e}")
                time.sleep(0.1)
    
    def send(self, message):
        """Mesaj gönder (broadcast)"""
        if not self.running:
            return False
        
        self.tx_queue.put(message)
        return True
    
    def receive(self, timeout=0.01):
        """Mesaj al (bloklama)"""
        try:
            return self.rx_queue.get(timeout=timeout)
        except Empty:
            return None
    
    def receive_nowait(self):
        """Mesaj al (bloklama yok)"""
        try:
            return self.rx_queue.get_nowait()
        except Empty:
            return None
    
    def broadcast_state(self, lat, lon, alt, yaw=0.0):
        """Durum mesajı yayınla"""
        msg = {
            "type": "state",
            "id": self.drone_id,
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt),
            "yaw": float(yaw)
        }
        return self.send(msg)
    
    def close(self):
        """Bağlantıyı kapat"""
        self.running = False
        time.sleep(0.2)
        if self.serial:
            try:
                # ESP'ye disconnect mesajı gönder
                self.serial.write(json.dumps({"type": "disconnect", "from": self.drone_id}).encode('utf-8'))
                self.serial.write(b"\n")
                self.serial.flush()
                time.sleep(0.1)
                self.serial.close()
            except:
                pass
        print(f"[ESP-Drone{self.drone_id}] Bağlantı kapatıldı")
