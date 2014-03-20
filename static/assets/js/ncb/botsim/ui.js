/* jslint browser: true */
/* global $, BOTSIM: true */

'use strict';

$('#botsim-file').on('change', function () {
	var character = $('input[name=botsim-character]:checked').val();
    BOTSIM.loadScene(this.files, character);
});

$('#botsim-test').on('click', function () {
	var character = $('input[name=botsim-character]:checked').val();
    BOTSIM.loadScene(['assets/kmz/test.kmz'], character);
});

BOTSIM.on('scene-loaded', function () {
    $('div #botsim-options').hide();
});
