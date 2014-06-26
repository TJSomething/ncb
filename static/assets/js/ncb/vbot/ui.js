/* jslint browser: true */
/* global $, VBOT: true */

(function (global) {
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
})(this);