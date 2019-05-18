let url = window.location.href.split(':')[1].split('/').pop();
console.log(url)
var socket = new WebSocket("ws://"+url+":9876/ws");
socket.onmessage = function (event) {
  document.getElementById("batteryVoltage").innerHTML = "Battery Voltage: " + event.data;
}
// setInterval(function (){
//     console.log(socket.readyState);
// },2000);
