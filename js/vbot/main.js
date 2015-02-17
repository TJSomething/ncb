/* jslint browser: true */
/* global $, VBOT: true */

'use strict';
var $ = require('jquery');
var VBOT = require('./app');
var controller = require('./controller');

var startedLoading = false;

$().ready( function() {
    $('#vbot-file').on('change', function () {
        if (!startedLoading) {
            var character = $('input[name=vbot-character]:checked').val();
            VBOT.loadScene(this.files, character);
            startedLoading = true;
        }
    });

    $('#vbot-test').on('click', function () {
        if (!startedLoading) {
            var character = $('input[name=vbot-character]:checked').val();
            VBOT.loadScene(['assets/3d/test_level.kmz'], character);
            startedLoading = true;
        }
    });

    $('#mainNav').find('a').click(function() {
        if ($('#vbot-body').is(':visible')) {
            VBOT.pause(false);
        } else {
            VBOT.pause(true);
        }
    });
} );

VBOT.on('scene-loaded', function () {
    document.addEventListener( 'keyup', function (event) {
        var req = new XMLHttpRequest();
        switch (event.keyCode) {
            case 13: // enter
                req.addEventListener('load', function () {
                    console.log('loaded example');
                    controller.start(VBOT, this.responseText);
                });
                req.open('GET', 'assets/js/sample_script.js', true);
                req.send();
        }
    }, false );

    $('div #vbot-options').hide();
});
