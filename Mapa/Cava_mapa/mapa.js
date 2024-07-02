var map;
var marker;
let path = [];
let polyline
const home = { lat: 10.918982886682658, lng: -74.87194240611939 };
let current_cava_position;

const options = {
    clean: true, // retain session
    connectTimeout: 4000, // Timeout period
    clientId: 'web_app_cava_position',
    username: 'cava_pos_test',
    password: 'test1234',
    Keepalive: 60,
};

const connectUrl = 'ws://emqx@127.0.0.1:8083/mqtt';
const client = mqtt.connect(connectUrl, options);
const topic_cava = 'proyectoLuis/cava/datos';
let cava_data = { "lat": 0, "long": 0, "occup": 0, "NMEA_st": 1 };

function init_map(){
    //map = L.map('cava_map').setView([home.lat, home.lng], 13);
    map = L.map('map').setView([home.lat, home.lng], 13);
    //OpenStreetMap 
    var tile_map = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    });

    tile_map.addTo(map)

    marker = L.marker([home.lat, home.lng]).addTo(map);
    marker.bindPopup("<b>Posición</b>"+"<br>"+"lat: "+home.lat.toString()+
                     "<br>"+" lng: "+home.lng.toString()+
                     "<br> CD"
                    ).openPopup();
}

function get_cava_data(cava_data) {

    let latitude = parseFloat(cava_data.lat);
    let longitude = parseFloat(cava_data.long);

    return { lat: latitude, lng: longitude };
}

function set_cava_position_map(position){
    
    document.getElementById('lat').textContent = "Lat: "+position.lat.toString();
    document.getElementById('lng').textContent = "Lng: "+position.lng.toString();
    
    if (marker) {
        map.removeLayer(marker);
    }

    marker = L.marker([position.lat, position.lng]).addTo(map);
    var popup_info = "<b>Posición</b>"+"<br>"+position.lat.toString()+", "+position.lng.toString();
    marker.bindPopup(popup_info).openPopup();

    path.push([position.lat, position.lng]);
/*
    if (path.length > 1) {
        map.removeLayer(rastro);
    }
        */

    var rastro = L.polyline(path, {color: 'blue'}).addTo(map);

    map.setView([position.lat, position.lng], 15);
}

init_map();

client.on('connect', function () {
    console.log('¡Conectado!')
    // Subscribe to a topic
    client.subscribe(topic_cava, { qos: 0 }, function (err) {
        if (!err) {
            console.log('Suscrito')
        }
    })
});

client.on('reconnect', (error) => {
    console.log('reconectando:', error)
});

client.on('error', (error) => {
    console.log('Conexión fallida:', error)
});

client.on('message', (topic, message) => {

    //console.log('mensaje recibido：', topic, current_cava_position)
    current_cava_position = get_cava_data(JSON.parse(message.toString()));
    set_cava_position_map(current_cava_position);
});


