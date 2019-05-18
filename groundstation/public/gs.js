let url = window.location.href.split(':')[1].split('/').pop();
console.log(url)
var socket = new WebSocket("ws://"+url+":9876/ws");
socket.onmessage = function (event) {
  let json = JSON.parse(event.data);
  if (json.type==="wod") {
    let keys = Object.keys(json);
    for (var i = 0; i < Object.keys(json).length; i++) {
      if (keys[i]==="type") {
        continue;
      } else {
        document.getElementById("wod_" + keys[i]).innerHTML = json[keys[i]].toFixed(3);
      }
    }
  }

}
// setInterval(function (){
//     console.log(socket.readyState);
// },2000);
