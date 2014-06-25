/* jslint browser: true */
/* global $, VBOT: true */

(function (global) {
	'use strict';

	$('#vbot-file').on('change', function () {
		var character = $('input[name=vbot-character]:checked').val();
		VBOT.loadScene(this.files, character);
	});

	$('#vbot-test').on('click', function () {
		var character = $('input[name=vbot-character]:checked').val();
		VBOT.loadScene(['assets/kmz/test.kmz'], character);
	});

	VBOT.on('scene-loaded', function () {
		$('div #vbot-options').hide();
	});
})(this);