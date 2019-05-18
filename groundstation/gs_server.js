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
        //console.log(buffer.length);
    }
    if (buffer.length >= 81) {
        wod = {};
        wod.type = 'wod';
        //wod.time = buffer.readBigUInt64LE(0); //use when we figure out 8byte timestamp
        wod.time = buffer.readUInt16LE(0)/1000; //use for now
        wod.mode = buffer.readUInt8(8);
        wod.vBat = buffer.readFloatLE(9);
        wod.iBat = buffer.readFloatLE(13);
        wod.tBat = buffer.readFloatLE(17);
        wod.v5V = buffer.readFloatLE(21);
        wod.i5V = buffer.readFloatLE(25);
        wod.v3V3 = buffer.readFloatLE(29);
        wod.i3V3 = buffer.readFloatLE(33);
        wod.tRegs = buffer.readFloatLE(37);
        wod.tRFD = buffer.readFloatLE(41);
        wod.lat = buffer.readFloatLE(45);
        wod.lon = buffer.readFloatLE(49);
        wod.alt = buffer.readFloatLE(53);
        wod.yaw = buffer.readFloatLE(57);
        wod.pitch = buffer.readFloatLE(61);
        wod.roll = buffer.readFloatLE(65);
        wod.yawRate = buffer.readFloatLE(69);
        wod.pitchRate = buffer.readFloatLE(73);
        wod.rollRate = buffer.readFloatLE(77);

        console.log(wod)
        ws.clients.forEach(function (client) {
            client.send(JSON.stringify(wod));
        });
    }

    //console.log(char);
}
