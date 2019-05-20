let url = window.location.href.split(':')[1].split('/').pop();
var socket = new WebSocket("ws://"+url+":9876/ws");
var map;
//while(document.getElementById("map")==null);
socket.onmessage = function (event) {
  let json = JSON.parse(event.data);
  if (json.type==="wod") {
    let keys = Object.keys(json);
    for (var i = 0; i < Object.keys(json).length; i++) {
      if (keys[i]==="type") {
        continue;
      } else {
        document.getElementById("wod_" + keys[i]).innerHTML = json[keys[i]];
      }
    }
    if (json.lat <= 90.0) {
        map.setView([json.lat, json.lon]);
    }
  }
}

setTimeout(function(){
    map = L.map("map").setView([0, 0], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png?{foo}', {foo: 'bar', attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>'}).addTo(map);
}, 100);

function changeMode() {
    socket.send(document.getElementById("cc_mode").value);
}
