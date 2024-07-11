def parse_nmea_rmc(sentence):
    if not sentence.startswith('$GNRMC'):
        raise ValueError("La trama no es una sentencia RMC válida")

    fields = sentence.split(',')

    # Extraer y formatear la hora
    time_str = fields[1]
    hour = time_str[0:2]
    minute = time_str[2:4]
    second = time_str[4:6]
    time_formatted = f"{hour}:{minute}:{second}"

    # Extraer y formatear la fecha
    date_str = fields[9]
    day = date_str[0:2]
    month = date_str[2:4]
    year = "20" + date_str[4:6]
    date_formatted = f"{day}/{month}/{year}"

    # Extraer y convertir la latitud
    lat_str = fields[3]
    lat_deg = int(lat_str[0:2])
    lat_min = float(lat_str[2:])
    lat_hem = fields[4]

    lat_dd = convert_to_dd(lat_deg, lat_min, lat_hem)

    # Extraer y convertir la longitud
    lon_str = fields[5]
    lon_deg = int(lon_str[0:3])
    lon_min = float(lon_str[3:])
    lon_hem = fields[6]

    lon_dd = convert_to_dd(lon_deg, lon_min, lon_hem)

    return time_formatted, date_formatted, lat_dd, lon_dd

def convert_to_dd(degrees, minutes, hemisphere):
    dd = degrees + minutes / 60.0
    if hemisphere in ['S', 'W']:
        dd = -dd
    return dd
    return dms

# Ejemplo de trama RMC
nmea_rmc = [
            "$GNRMC,154130.00,A,1057.6282,N,07451.2683,W,9.2,314.6,080724,,,A*57",
            "$GNRMC,154130.00,A,1057.6582,N,07451.2696,W,15.6,85.3,080724,,,A*53",
            "$GNRMC,154130.00,A,1057.6314,N,07451.2848,W,15.9,168.6,080724,,,A*6F",
            "$GNRMC,154130.00,A,1057.6338,N,07451.3100,W,16.6,182.7,080724,,,A*6C",
            "$GNRMC,154130.00,A,1057.6082,N,07451.2756,W,18.5,254.6,080724,,,A*6E",
            "$GNRMC,154130.00,A,1057.5924,N,07451.2777,W,13.0,244.6,080724,,,A*64",
            "$GNRMC,154130.00,A,1057.5613,N,07451.2820,W,17.7,101.1,080724,,,A*64",
            "$GNRMC,154130.00,A,1057.5650,N,07451.3059,W,0.1,261.1,080724,,,A*51",
            "$GNRMC,154130.00,A,1057.6506,N,07451.2926,W,8.6,290.5,080724,,,A*57",
            "$GNRMC,154130.00,A,1057.6203,N,07451.2813,W,18.3,108.5,080724,,,A*64",
            "$GNRMC,154130.00,A,1057.5973,N,07451.2681,W,14.8,182.4,080724,,,A*6A"
           ]


# Decodificar trama RMC

for i in nmea_rmc:
    time, date, lat_dms, lon_dms = parse_nmea_rmc(i)

    print(f"{lat_dms}, {lon_dms}")
    
