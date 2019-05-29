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
    app.listen(httpport, '0.0.0.0');
    console.log("Listening on " + httpport);

// app
    // app.get('*', function(req, res) {
    //     port.write('send');
    //     res.sendFile('index.html', { root: path.join(__dirname, '../public') });
    // });
    app.ws('/ws', function(ws, req) {
        ws.on('message', function(msg) {
            console.log("Set mode: " + msg);
            port.write(Buffer.from([Number(msg)]));
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
    if (buffer.length > 0 && buffer[0]==='W'.charCodeAt(0) && buffer.length >= 87) {
        wod = {};
        buffer = buffer.slice(1);
        wod.type = 'wod';
        //probably won't use this but here in case... //wod.time = (BigInt(buffer.readUInt32LE(0)<<32) + BigInt(buffer.readUInt32LE(4))).toString(); //use when we figure out 8byte timestamp
        wod.time = new Date(buffer.readUInt32LE(0)*1000);wod.time = wod.time.toJSON().toString() //unix time, seconds
        wod.uptime = buffer.readUInt32LE(4); //milliseconds
        wod.mode = buffer.readUInt8(8);
        wod.mode = ["Operational","Downlink","Detumble","Pointing","Deployment","Safe","Launch","Startup","GroundTesting"][wod.mode];
        wod.vBat = buffer.readFloatLE(9).toFixed(1);
        wod.iBat = buffer.readFloatLE(13).toFixed(0);
        wod.tBat = buffer.readFloatLE(17).toFixed(0);
        wod.v5V = buffer.readFloatLE(21).toFixed(1);
        wod.i5V = buffer.readFloatLE(25).toFixed(0);
        wod.v3V3 = buffer.readFloatLE(29).toFixed(1);
        wod.i3V3 = buffer.readFloatLE(33).toFixed(0);
        wod.tRegs = buffer.readFloatLE(37).toFixed(0);
        wod.tRFD = buffer.readFloatLE(41).toFixed(0);
        wod.lat = buffer.readFloatLE(45).toFixed(5);
        wod.lon = buffer.readFloatLE(49).toFixed(5);
        wod.alt = buffer.readFloatLE(53).toFixed(0);
        wod.yaw = buffer.readFloatLE(57).toFixed(2);
        wod.pitch = buffer.readFloatLE(61).toFixed(2);
        wod.roll = buffer.readFloatLE(65).toFixed(2);
        wod.yawRate = buffer.readFloatLE(69).toFixed(2);
        wod.pitchRate = buffer.readFloatLE(73).toFixed(2);
        wod.rollRate = buffer.readFloatLE(77).toFixed(2);
        wod.sats = buffer.readUInt8(81).toFixed(0);
        wod.hdop = ((buffer.readUInt32LE(82))/100).toFixed(2);

        //console.log(wod)
        ws.clients.forEach(function (client) {
            client.send(JSON.stringify(wod));
        });
    }
    if (buffer.length > 0 && buffer[0]==='S'.charCodeAt(0) && buffer.length >= 9) {
        sci = {};
        buffer = buffer.slice(1);
        sci.type = 'sci';
        sci.time = new Date(buffer.readUInt32LE(0)*1000);sci.time = sci.time.toJSON().toString() //unix time, seconds
        sci.vTether = buffer.readFloatLE(4).toFixed(3);
        if (Number(sci.vTether) > 2.5) {
            sci.vTether += " (OVER RANGE)"
        }

        //console.log(JSON.stringify(sci));

        //console.log(wod)
        ws.clients.forEach(function (client) {
            client.send(JSON.stringify(sci));
        });
    }

    //console.log(char);
}
