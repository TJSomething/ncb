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

requirejs(['jquery', 'vbot/app'], function ($, VBOT) {
	'use strict';

	$().ready( function() {
		$('#vbot-file').on('change', function () {
			var character = $('input[name=vbot-character]:checked').val();
			VBOT.loadScene(this.files, character);
		});

		$('#vbot-test').on('click', function () {
			var character = $('input[name=vbot-character]:checked').val();
			VBOT.loadScene(['assets/kmz/test.kmz'], character);
		});

		$('#mainNav a').click(function() {
			if ($('#vbot-body').is(':visible')) {
				VBOT.pause(false);
			} else {
				VBOT.pause(true);
			}
		});
	} );

	VBOT.on('scene-loaded', function () {
		$('div #vbot-options').hide();
	});
});