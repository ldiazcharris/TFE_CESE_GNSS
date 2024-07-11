import random
import datetime

def generate_nmea(lat, lon):
    # Hora y fecha actuales
    now = datetime.datetime.utcnow()
    time_str = now.strftime('%H%M%S')
    date_str = now.strftime('%d%m%y')

    # Convertir latitud y longitud a formato NMEA
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60
    lat_hem = 'N' if lat_deg >= 0 else 'S'
    lat_nmea = f"{abs(lat_deg):02d}{abs(lat_min):07.4f}"

    lon_deg = int(lon)
    lon_min = (lon - lon_deg) * 60
    lon_hem = 'E' if lon_deg >= 0 else 'W'
    lon_nmea = f"{abs(lon_deg):03d}{abs(lon_min):07.4f}"

    # Generar velocidad y ángulo de curso aleatorios
    speed = random.uniform(0, 20)
    course = random.uniform(0, 360)

    # Trama base sin checksum
    nmea_base = f"GNRMC,{time_str}.00,A,{lat_nmea},{lat_hem},{lon_nmea},{lon_hem},{speed:.1f},{course:.1f},{date_str},,,A"

    # Calcular checksum
    checksum = 0
    for char in nmea_base:
        checksum ^= ord(char)
    checksum_str = f"{checksum:02X}"

    # Trama final
    nmea_sentence = f"${nmea_base}*{checksum_str}"
    return nmea_sentence

# Coordenadas proporcionadas
coordinates = [
    (10.96046963, -74.85447104),
    (10.96096967, -74.85449264),
    (10.96052396, -74.85474651),
    (10.96056399, -74.85516585),
    (10.96013613, -74.8545933),
    (10.95987377, -74.85462887),
    (10.95935576, -74.85469958),
    (10.95941637, -74.85509867),
    (10.96084288, -74.85487636),
    (10.96033852, -74.85468831),
    (10.95995456, -74.85446811)
]

# Generar tramas NMEA y guardarlas en un archivo de texto
with open('tramas_nmea.txt', 'w') as file:
    for lat, lon in coordinates:
        nmea = generate_nmea(lat, lon)
        file.write(nmea + '\n')
        print(nmea)
