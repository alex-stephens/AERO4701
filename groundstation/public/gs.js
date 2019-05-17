let url = window.location.href.split(':')[1].split('/').pop();
console.log(url)
var socket = new WebSocket("ws://"+url+":9876/ws");
socket.onmessage = function (event) {
  console.log(event.data);
}
// setInterval(function (){
//     console.log(socket.readyState);
// },2000);
