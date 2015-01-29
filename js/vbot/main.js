/* jslint browser: true */
/* global $, VBOT: true */

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