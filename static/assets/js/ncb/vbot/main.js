/* jslint browser: true */
/* global $, VBOT: true */

requirejs.config({
    baseUrl: 'assets/js/ncb',
    shim: {
        threeCore: {
            exports: 'THREE'
        },
        OrbitControls: {
            deps: ['threeCore'],
            exports: 'THREE'
        },
        ColladaLoader: {
            deps: ['threeCore'],
            exports: 'THREE'
        },
        stats: {
            exports: 'Stats'
        },
        underscore: {
            exports: '_'
        },
        numeric: {
            exports: 'numeric'
        }
    },
    paths: {
        three: './vbot/three',
        threeCore: '../lib/three.min',
        OrbitControls: '../lib/OrbitControls',
        ColladaLoader: '../lib/ColladaLoader',
        stats: '../lib/stats.min',
        underscore: '../lib/underscore',
        jszip: '../lib/jszip.min',
        jquery: '../lib/jquery',
        numeric: '../lib/numeric-1.2.6.min'
    }
});

requirejs(['jquery', 'vbot/app', 'vbot/controller'], function ($, VBOT, controller) {
	'use strict';
    
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
                VBOT.loadScene(['assets/kmz/test.kmz'], character);   
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
        document.addEventListener( 'keydown', function (event) {
            if (event.altKey) {
                return;
            }

            switch (event.keyCode) {
                case 83: // S
                    VBOT.robot.speed = -1.7;
                    break;
                case 87: // W
                    VBOT.robot.speed = 1.7;
                    break;
                case 65: // A
                    VBOT.robot.angularVelocity = 1;
                    break;
                case 68: // D
                    VBOT.robot.angularVelocity = -1;
                    break;
                default:
                    break;
            }
        }, false );
        document.addEventListener( 'keyup', function (event) {
            switch (event.keyCode) {
                case 83: // S
                    VBOT.robot.speed = 0;
                    break;
                case 87: // W
                    VBOT.robot.speed = 0;
                    break;
                case 65: // A
                    VBOT.robot.angularVelocity = 0;
                    break;
                case 68: // D
                    VBOT.robot.angularVelocity = 0;
                    break;
                case 69: // E
                    controller.test(VBOT);
            }
        }, false );

        $('div #vbot-options').hide();
	});
});