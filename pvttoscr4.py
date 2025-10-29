#!/usr/bin/env python3
import serial
import struct
import time
from datetime import datetime
import sys

class UBXParser:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.UBX_HEADER = b'\xB5\x62'
        # Храним последние полученные данные для создания SOL
        self.pvt = None
        self.posecef = None
        self.velecef = None
        self.timeutc = None
        
    def calc_checksum(self, data):
        """Calculate UBX checksum for message"""
        CK_A, CK_B = 0, 0
        for byte in data:
            CK_A = (CK_A + byte) & 0xFF
            CK_B = (CK_B + CK_A) & 0xFF
        return CK_A, CK_B
    
    def verify_checksum(self, msg_class, msg_id, payload):
        """Verify message checksum"""
        data = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload)) + payload
        CK_A, CK_B = self.calc_checksum(data)
        return CK_A, CK_B
    
    def send_ubx_message(self, msg_class, msg_id, payload):
        """Send UBX message with proper header and checksum"""
        # Формируем данные для контрольной суммы
        data = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload)) + payload
        
        # Вычисляем контрольную сумму
        CK_A, CK_B = self.calc_checksum(data)
        
        # Формируем полное сообщение
        message = self.UBX_HEADER + data + bytes([CK_A, CK_B])
        
        # Отправляем в порт
        self.ser.write(message)
        print(f"Sent UBX message: Class=0x{msg_class:02X}, ID=0x{msg_id:02X}, Length={len(payload)}")
    
    def parse_ubx_nav_pvt(self, payload):
        """Parse UBX-NAV-PVT message (0x01 0x07)"""
        if len(payload) < 92:
            return None
            
        format_string ='<I H B B B B B B I i b b B B i i i i I i i i i i I I I H 6x i h H'
        values = struct.unpack_from(format_string, payload, 0)
        
        (iTOW, year, month, day, hour, min, sec, valid, 
         tAcc, nano, fixType, flags, flags2, numSV, lon, lat, 
         height, hMSL, hAcc, vAcc, velN, velE, velD, gSpeed, 
         headMot, sAcc, headAcc, pDOP, headVeh, magDec, magAcc) = values
      
        result = {
            'iTOW': iTOW,
            'datetime': {
                'year': year,
                'month': month,
                'day': day,
                'hour': hour,
                'minute': min,
                'second': sec,
                'valid': valid,
                'tAcc': tAcc,
                'nano': nano
            },
            'position': {
                'lat': lat,  # градусы * 1e-7
                'lon': lon,  # градусы * 1e-7
                'height': height,  # мм
                'hMSL': hMSL,  # мм
                'hAcc': hAcc,  # мм
                'vAcc': vAcc   # мм
            },
            'velocity': {
                'velN': velN,  # мм/с
                'velE': velE,  # мм/с
                'velD': velD,  # мм/с
                'gSpeed': gSpeed,  # мм/с
                'headMot': headMot,  # градусы * 1e-5
                'sAcc': sAcc,  # мм/с
                'headAcc': headAcc  # градусы * 1e-5
            },
            'navigation': {
                'fixType': fixType,
                'flags': flags,
                'flags2': flags2,
                'numSV': numSV,
                'pDOP': pDOP,  # * 0.01
                'headVeh': headVeh,  # градусы * 1e-5
                'magDec': magDec,  # градусы * 1e-2
                'magAcc': magAcc   # градусы * 1e-2
            }
        }
        
        return result

    def parse_ubx_nav_posecef(self, payload):
        """Parse UBX-NAV-POSECEF message (0x01 0x01)"""
        if len(payload) < 20:
            return None
            
        # Формат: iTOW(U4), ecefX(I4), ecefY(I4), ecefZ(I4), pAcc(U4)
        format_string = '<I i i i I'
        values = struct.unpack_from(format_string, payload, 0)
        
        (iTOW, ecefX, ecefY, ecefZ, pAcc) = values
        
        result = {
            'iTOW': iTOW,
            'position': {
                'ecefX': ecefX,  # см
                'ecefY': ecefY,  # см
                'ecefZ': ecefZ,  # см
            },
            'accuracy': {
                'pAcc': pAcc  # см
            }
        }
        
        return result

    def parse_ubx_nav_velecef(self, payload):
        """Parse UBX-NAV-VELECEF message (0x01 0x11)"""
        if len(payload) < 20:
            return None
            
        # Формат: iTOW(U4), ecefVX(I4), ecefVY(I4), ecefVZ(I4), sAcc(U4)
        format_string = '<I i i i I'
        values = struct.unpack_from(format_string, payload, 0)
        
        (iTOW, ecefVX, ecefVY, ecefVZ, sAcc) = values
        
        result = {
            'iTOW': iTOW,
            'velocity': {
                'ecefVX': ecefVX,  # см/с
                'ecefVY': ecefVY,  # см/с
                'ecefVZ': ecefVZ,  # см/с
            },
            'accuracy': {
                'sAcc': sAcc  # см/с
            }
        }
        
        return result

    def parse_ubx_nav_timeutc(self, payload):
        """Parse UBX-NAV-TIMEUTC message (0x01 0x21)"""
        if len(payload) < 20:
            return None
            
        # Формат: iTOW(U4), tAcc(U4), nano(I4), year(U2), month(U1), day(U1), hour(U1), min(U1), sec(U1), valid(U1)
        format_string = '<I I i H B B B B B B'
        values = struct.unpack_from(format_string, payload, 0)
        
        (iTOW, tAcc, nano, year, month, day, hour, min, sec, valid) = values
        
        result = {
            'iTOW': iTOW,
            'tAcc': tAcc,  # нс
            'nano': nano,  # нс
            'datetime': {
                'year': year,
                'month': month,
                'day': day,
                'hour': hour,
                'minute': min,
                'second': sec
            },
            'valid': valid
        }
        
        return result
    
    def create_ubx_nav_sol(self):
        """Create UBX-NAV-SOL message from available data"""
        if not self.pvt or not self.velecef:
            return None
            
        # Берем данные из PVT и VELECEF
        iTOW = self.pvt['iTOW']
        fTOW = self.pvt['datetime']['nano']  # наносекунды из PVT
        week = 2035  # номер недели GPS (можно вычислить из даты при необходимости)
        
        # Статус фиксации и флаги из PVT
        gpsFix = self.pvt['navigation']['fixType']

        # Формируем флаги для SOL на основе флагов из PVT
        flags_pvt = self.pvt['navigation']['flags']
        valid_pvt = self.pvt['datetime']['valid']
        
        # Инициализируем флаги для SOL
        flags_sol = 0
        
        # Устанавливаем GPSfixOK если установлен gnssFixOK в PVT
        if flags_pvt & 0x01:  # gnssFixOK в PVT
            flags_sol |= 0x01  # gpsFixOK в SOL
        
        # Устанавливаем WKNSET и TOWSET если установлен validTime в PVT
        if valid_pvt & 0x02:  # validTime в PVT
            flags_sol |= 0x04  # WKNSET в SOL
            flags_sol |= 0x08  # TOWSET в SOL
        
        # Копируем diffSoln если он установлен в PVT
        if flags_pvt & 0x02:  # diffSoln в PVT
            flags_sol |= 0x02  # diffSoln в SOL

        
        # Позиция в ECEF (если есть POSECEF, используем его, иначе ставим 0)
        if self.posecef:
            ecefX = self.posecef['position']['ecefX']
            ecefY = self.posecef['position']['ecefY']
            ecefZ = self.posecef['position']['ecefZ']
            pAcc = self.posecef['accuracy']['pAcc']
        else:
            ecefX = ecefY = ecefZ = pAcc = 0
        
        # Скорость в ECEF из VELECEF
        ecefVX = self.velecef['velocity']['ecefVX']
        ecefVY = self.velecef['velocity']['ecefVY']
        ecefVZ = self.velecef['velocity']['ecefVZ']
        sAcc = self.velecef['accuracy']['sAcc']
        
        # DOP и количество спутников из PVT
        pDOP = self.pvt['navigation']['pDOP']
        numSV = self.pvt['navigation']['numSV']
        
        # Формируем payload для UBX-NAV-SOL (52 байта)
        # Структура: iTOW(U4), fTOW(I4), week(I2), gpsFix(U1), flags(X1), 
        # ecefX(I4), ecefY(I4), ecefZ(I4), pAcc(U4), 
        # ecefVX(I4), ecefVY(I4), ecefVZ(I4), sAcc(U4), 
        # pDOP(U2), reserved1(U1), numSV(U1), reserved2(U4)
        sol_payload = struct.pack('<I i h B B i i i I i i i I H B B I',
            iTOW,      # GPS time of week (ms)
            fTOW,      # fractional nanoseconds (-1e9..1e9)
            week,      # GPS week number
            gpsFix,    # GPS fix type
            flags_sol,     # Fix status flags
            ecefX,     # ECEF X coordinate (cm)
            ecefY,     # ECEF Y coordinate (cm)  
            ecefZ,     # ECEF Z coordinate (cm)
            pAcc,      # Position accuracy estimate (cm)
            ecefVX,    # ECEF X velocity (cm/s)
            ecefVY,    # ECEF Y velocity (cm/s)
            ecefVZ,    # ECEF Z velocity (cm/s)
            sAcc,      # Speed accuracy estimate (cm/s)
            pDOP,      # Position DOP (0.01)
            0,         # reserved1
            numSV,     # Number of satellites used in solution
            0          # reserved2
        )
        
        return sol_payload
    
    def decode_fix_type(self, fix_type):
        """Decode fix type to human readable string"""
        fix_types = {
            0: 'No Fix',
            1: 'Dead Reckoning Only',
            2: '2D-Fix',
            3: '3D-Fix',
            4: 'GNSS + Dead Reckoning',
            5: 'Time Only Fix'
        }
        return fix_types.get(fix_type, f'Unknown ({fix_type})')
    
    def decode_valid_flags(self, valid):
        """Decode validity flags"""
        flags = []
        if valid & 0x01: flags.append('Valid Date')
        if valid & 0x02: flags.append('Valid Time')
        if valid & 0x04: flags.append('Fully Resolved')
        if valid & 0x08: flags.append('Valid Magnetic Declination')
        return ', '.join(flags) if flags else 'None'
    
    def decode_nav_flags(self, flags):
        """Decode navigation flags"""
        nav_flags = []
        if flags & 0x01: nav_flags.append('GNSS Fix OK')
        if flags & 0x02: nav_flags.append('Diff Soln')
        if flags & 0x04: nav_flags.append('PSM State')
        if flags & 0x08: nav_flags.append('HeadVeh Valid')
        # Carrier phase status
        carr_soln = (flags >> 4) & 0x03
        if carr_soln == 1: nav_flags.append('Float Solution')
        if carr_soln == 2: nav_flags.append('Fixed Solution')
        return ', '.join(nav_flags) if nav_flags else 'None'
    
    def decode_timeutc_valid_flags(self, valid):
        """Decode TIMEUTC validity flags"""
        flags = []
        if valid & 0x01: flags.append('Valid Time')
        if valid & 0x02: flags.append('Valid Week Number')
        if valid & 0x04: flags.append('Valid Time of Week')
        return ', '.join(flags) if flags else 'None'
    
    def format_datetime(self, data):
        """Format datetime from PVT data"""
        try:
            dt = datetime(
                data['datetime']['year'],
                data['datetime']['month'],
                data['datetime']['day'],
                data['datetime']['hour'],
                data['datetime']['minute'],
                data['datetime']['second']
            )
            return dt.strftime("%Y-%m-%d %H:%M:%S")
        except:
            return "Invalid Date/Time"
    
    def display_pvt_data(self, data):
        """Display PVT data in human readable format"""
        print("\n" + "="*60)
        print("UBX-NAV-PVT DATA")
        print("="*60)
        
        # Время
        print(f"GPS Time of Week: {data['iTOW']} ms")
        print(f"UTC Date/Time: {self.format_datetime(data)}")
        print(f"Time Accuracy: {data['datetime']['tAcc']} ns")
        print(f"Validity Flags: {self.decode_valid_flags(data['datetime']['valid'])}")
        
        # Позиция
        print(f"\n--- POSITION ---")
        print(f"Latitude: {data['position']['lat']} (degrees * 1e-7)")
        print(f"Longitude: {data['position']['lon']} (degrees * 1e-7)")
        print(f"Height above Ellipsoid: {data['position']['height']} mm")
        print(f"Height above MSL: {data['position']['hMSL']} mm")
        print(f"Horizontal Accuracy: {data['position']['hAcc']} mm")
        print(f"Vertical Accuracy: {data['position']['vAcc']} mm")
        
        # Навигация
        print(f"\n--- NAVIGATION ---")
        print(f"Fix Type: {self.decode_fix_type(data['navigation']['fixType'])}")
        print(f"Number of SVs: {data['navigation']['numSV']}")
        print(f"Navigation Flags: {self.decode_nav_flags(data['navigation']['flags'])}")
        print(f"pDOP: {data['navigation']['pDOP']} (DOP * 100)")
        
        # Скорость
        print(f"\n--- VELOCITY ---")
        print(f"Ground Speed: {data['velocity']['gSpeed']} mm/s")
        print(f"Heading of Motion: {data['velocity']['headMot']} (degrees * 1e-5)")
        print(f"Velocity N: {data['velocity']['velN']} mm/s")
        print(f"Velocity E: {data['velocity']['velE']} mm/s")
        print(f"Velocity D: {data['velocity']['velD']} mm/s")
        print(f"Speed Accuracy: {data['velocity']['sAcc']} mm/s")
        print(f"Heading Accuracy: {data['velocity']['headAcc']} (degrees * 1e-5)")
        
        # Дополнительно
        print(f"\n--- ADDITIONAL ---")
        print(f"Vehicle Heading: {data['navigation']['headVeh']} (degrees * 1e-5)")
        print(f"Magnetic Declination: {data['navigation']['magDec']} (degrees * 1e-2)")
        print(f"Magnetic Declination Accuracy: {data['navigation']['magAcc']} (degrees * 1e-2)")
        
        print("="*60)

    def display_posecef_data(self, data):
        """Display POSECEF data in human readable format"""
        print("\n" + "="*60)
        print("UBX-NAV-POSECEF DATA")
        print("="*60)
        
        print(f"GPS Time of Week: {data['iTOW']} ms")
        
        print(f"\n--- ECEF POSITION ---")
        print(f"ECEF X: {data['position']['ecefX']} cm")
        print(f"ECEF Y: {data['position']['ecefY']} cm")
        print(f"ECEF Z: {data['position']['ecefZ']} cm")
        print(f"Position Accuracy: {data['accuracy']['pAcc']} cm")
        
        print("="*60)

    def display_velecef_data(self, data):
        """Display VELECEF data in human readable format"""
        print("\n" + "="*60)
        print("UBX-NAV-VELECEF DATA")
        print("="*60)
        
        print(f"GPS Time of Week: {data['iTOW']} ms")
        
        print(f"\n--- ECEF VELOCITY ---")
        print(f"ECEF VX: {data['velocity']['ecefVX']} cm/s")
        print(f"ECEF VY: {data['velocity']['ecefVY']} cm/s")
        print(f"ECEF VZ: {data['velocity']['ecefVZ']} cm/s")
        print(f"Speed Accuracy: {data['accuracy']['sAcc']} cm/s")
        
        print("="*60)

    def display_timeutc_data(self, data):
        """Display TIMEUTC data in human readable format"""
        print("\n" + "="*60)
        print("UBX-NAV-TIMEUTC DATA")
        print("="*60)
        
        print(f"GPS Time of Week: {data['iTOW']} ms")
        print(f"UTC Date/Time: {self.format_datetime(data)}")
        print(f"Time Accuracy: {data['tAcc']} ns")
        print(f"Fraction of Second: {data['nano']} ns")
        print(f"Validity Flags: {self.decode_timeutc_valid_flags(data['valid'])}")
        
        print("="*60)
    
    def read_messages(self):
        """Main loop to read and parse UBX messages"""
        print(f"Starting UBX parser on {self.ser.port}...")
        buffer = b''
        
        try:
            while True:
                # Читаем данные из порта
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer += data
                    
                    # Ищем UBX заголовок
                    while len(buffer) >= 6:
                        header_pos = buffer.find(self.UBX_HEADER)
                        if header_pos == -1:
                            buffer = b''
                            break
                            
                        if header_pos > 0:
                            buffer = buffer[header_pos:]
                            
                        if len(buffer) < 6:
                            break
                            
                        # Извлекаем класс, ID и длину
                        msg_class, msg_id, length = struct.unpack_from('<BBH', buffer, 2)
                        
                        # Проверяем полное сообщение
                        if len(buffer) >= length + 8:  # заголовок(2) + класс(1) + ID(1) + длина(2) + данные + CRC(2)
                            payload = buffer[6:6+length]
                            crc = buffer[6+length:6+length+2]
                            
                            # Проверяем контрольную сумму
                            CK_A, CK_B = self.verify_checksum(msg_class, msg_id, payload)
                            if crc == bytes([CK_A, CK_B]):
                                # Обрабатываем NAV-PVT сообщение
                                if msg_class == 0x01 and msg_id == 0x07:
                                    pvt_data = self.parse_ubx_nav_pvt(payload)
                                    if pvt_data:
                                        self.pvt = pvt_data
                                        self.display_pvt_data(pvt_data)
                                        # Передаем без изменений
                                        self.send_ubx_message(msg_class, msg_id, payload)
                                
                                # Обрабатываем NAV-POSECEF сообщение
                                elif msg_class == 0x01 and msg_id == 0x01:
                                    posecef_data = self.parse_ubx_nav_posecef(payload)
                                    if posecef_data:
                                        self.posecef = posecef_data
                                        self.display_posecef_data(posecef_data)
                                        # Передаем без изменений
                                        self.send_ubx_message(msg_class, msg_id, payload)
                                
                                # Обрабатываем NAV-VELECEF сообщение
                                elif msg_class == 0x01 and msg_id == 0x11:
                                    velecef_data = self.parse_ubx_nav_velecef(payload)
                                    if velecef_data:
                                        self.velecef = velecef_data
                                        self.display_velecef_data(velecef_data)
                                        # Передаем без изменений
                                        self.send_ubx_message(msg_class, msg_id, payload)
                                        
                                        # Создаем и отправляем NAV-SOL
                                        sol_payload = self.create_ubx_nav_sol()
                                        if sol_payload:
                                            self.send_ubx_message(0x01, 0x06, sol_payload)
                                
                                # Обрабатываем NAV-TIMEUTC сообщение
                                elif msg_class == 0x01 and msg_id == 0x21:
                                    timeutc_data = self.parse_ubx_nav_timeutc(payload)
                                    if timeutc_data:
                                        self.timeutc = timeutc_data
                                        self.display_timeutc_data(timeutc_data)
                                        # Передаем без изменений
                                        self.send_ubx_message(msg_class, msg_id, payload)
                                
                                # Обрабатываем другие сообщения (передаем без изменений)
                                elif msg_class == 0x01:  # NAV класс
                                    self.send_ubx_message(msg_class, msg_id, payload)
                                
                                # Удаляем обработанное сообщение из буфера
                                buffer = buffer[6+length+2:]
                            else:
                                # Неверная контрольная сумма, пропускаем байт
                                buffer = buffer[1:]
                        else:
                            break
                            
        except KeyboardInterrupt:
            print("\nStopping UBX parser...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.ser.close()

def main():
    # Настройка последовательного порта
    port = '/dev/ttyUSB0'  # Альтернативный вариант
    
    parser = UBXParser(port=port)
    parser.read_messages()

if __name__ == "__main__":
    main()
