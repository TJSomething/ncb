/* jslint browser: true */
/* global $, VBOT: true */

'use strict';
var $ = require('jquery');
var VBOT = require('./app');
var controller = require('./controller');
var physics = require('./physics');
var _ = require('underscore');

var startedLoading = false;
var script;

function loadScript() {
    var reader = new FileReader();
    var req = new XMLHttpRequest();
    var files = $('input#vbot-script')[0].files;

    if (files.length === 1) {
        reader.addEventListener('load', function (evt) {
            script = evt.target.result;
        });

        reader.readAsText(files[0]);
    } else {
        req.addEventListener('load', function () {
            script = this.responseText;
        });
        req.open('GET', 'assets/js/sample_script.js', true);
        req.send();
    }
}

$().ready( function() {
    $('#vbot-run-ncs').click(function () {
        var server = $('#vbot-server').val();
        var params = $('#vbot-params').val();
        $.ajax({
            type: 'POST',
            url: 'http://' + server + '/simulations/',
            data: params,
            contentType: 'text/plain',
            success: function (data) {
                $('#vbot-ncs-url').val(data);
            }
        });
    });

    $('#vbot-script').on('change', function () {
        $('#vbot-script-text').text(
            _.map(this.files, function(file) {
                return file.name;
            }));
    });

    $('#vbot-level').on('change', function () {
        $('#vbot-level-text').text(
            _.map(this.files, function(file) {
                return file.name;
            }));
    });

    $('#mainNav').find('a').click(function() {
        if ($('#vbot-body').is(':visible')) {
            VBOT.pause(false);
        } else {
            VBOT.pause(true);
        }
    });

    $('#vbot-run').on('click', function () {
        if (!startedLoading) {
            var character = $('input[name=vbot-character]:checked').val();
            var files = $('input#vbot-level')[0].files;

            // Default to the test level
            if (files.length === 0) {
                files = ['assets/3d/test_level.kmz'];
            }

            loadScript();
            VBOT.setDaemonURL($('#vbot-ncs-url').val());
            VBOT.loadScene(files, character);
            startedLoading = true;
        }
    });

} );

VBOT.on('scene-loaded', function () {
    document.addEventListener( 'keyup', function (event) {
        var req = new XMLHttpRequest();
        switch (event.keyCode) {
            case 13: // enter
                controller.start(VBOT, script);
                break;
            case 80:
                physics.toggleCollisionVolumes(VBOT.scene);
                break;
        }
    }, false );

    $('div #vbot-options').hide();
});
