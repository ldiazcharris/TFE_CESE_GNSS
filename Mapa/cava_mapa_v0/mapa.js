let map;
let marker;
let path = [];
let polyline
const home = { lat: 10.918982886682658, lng: -74.87194240611939 };
let current_cava_position;

async function initMap() {
    const { Map } = await google.maps.importLibrary("maps");
    //const { AdvancedMarkerElement, PinElement } = await google.maps.importLibrary("marker");
    

    map = new Map(document.getElementById("map"), {
        center: home,
        zoom: 16,
        mapId: "CavaTracker_TFE_CESE"

    });

    marker = new google.maps.Marker({
        map: map,
        position: home,
        title: "Cava001_position",
    });

    /*
    const marker = new google.maps.marker.AdvancedMarkerElement({
        map,
        position: home,
        title: "Cava001_position",
    });

     */

    polyline = new google.maps.Polyline({
        path: path,
        geodesic: true,
        strokeColor: '#FF0000',
        strokeOpacity: 1.0,
        strokeWeight: 2,
        map: map
    });

}

initMap();

function get_cava_position(cava_data) {

    let latitude = parseFloat(cava_data.lat);
    let longitude = parseFloat(cava_data.long);

    return { lat: latitude, lng: longitude };

}

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


client.on('connect', function () {
    console.log('Connected')
    // Subscribe to a topic
    client.subscribe(topic_cava, { qos: 0 }, function (err) {
        if (!err) {
            //client.publish(topic_cava, 'web app conectada')
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
    
    current_cava_position = get_cava_position(JSON.parse(message.toString()));
    console.log('mensaje recibido：', topic, current_cava_position)

    path.push(current_cava_position);
    polyline.setPath(path);

    marker.setPosition(current_cava_position);
    map.setCenter(current_cava_position);

});


