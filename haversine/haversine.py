import math

def haversine(lat1, lon1, lat2, lon2):

    # Radio de la Tierra en kilómetros
    R = 6371.0
    
    # Convertir grados a radianes
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    # Diferencias de coordenadas
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Fórmula de Haversine
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Calcular la distancia
    distance = R * c
    
    return distance


lat1, lon1 = 10.95995456, -74.85446811
lat2, lon2 = 10.9598994, -74.8546695	

distance = haversine(lat1, lon1, lat2, lon2)
print(f"La distancia entre los puntos es: {distance*1000:.3} m")