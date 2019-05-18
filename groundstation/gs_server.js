// setup
    var express = require('express');
    var app = express();
    var expressWs = require('express-ws')(app);
    var morgan = require('morgan');
    const SerialPort = require('serialport')
    const ByteLength = require('@serialport/parser-byte-length')
    const port = new SerialPort('/dev/cu.usbserial-AI04QKTK')

    var buffer = new Buffer.alloc(0);

// config
    let httpport = 9876;
    app.use(express.static(__dirname + '/public'));
    app.use(morgan('combined'));

// start
    app.listen(httpport);
    console.log("Listening on " + httpport);

// app
    // app.get('*', function(req, res) {
    //     port.write('send');
    //     res.sendFile('index.html', { root: path.join(__dirname, '../public') });
    // });
    app.ws('/ws', function(ws, req) {
        ws.on('message', function(msg) {
            console.log(msg);
            //ws.send(msg);
        });
    });
    var ws = expressWs.getWss('/ws');


// SerialPort
    const parser = port.pipe(new ByteLength({length: 1}));
    parser.on('data', datain)



function datain(newbuffer) {
    var char = newbuffer.toString();
    if (char === '~') {
        buffer = new Buffer.alloc(0);
    } else {
        buffer = Buffer.concat([buffer, newbuffer]);
    }
    if (buffer.length >= 4) {
        console.log(buffer.readFloatLE())
        ws.clients.forEach(function (client) {
            client.send(buffer.readFloatLE());
        });
    }

    //console.log(char);
}
