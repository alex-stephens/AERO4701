// setup
    var express = require('express');
    var app = express();
    var morgan = require('morgan');

// config
    app.use(express.static(__dirname + '/public'));
    app.use(morgan('dev'));

// start
    app.listen(9876);
    console.log("Listening on 9876");

// app
    app.get('*', function(req, res) {
        res.sendfile('./public/index.html');
    });
