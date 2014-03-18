/* jslint browser: true */
/* global $, BOTSIM: true */

'use strict';

$('input[name=botsim-character]').on('change', function () {
    BOTSIM.CHARACTER = $(this).val();
});

$('#botsim-file').on('change', function () {
    BOTSIM.loadScene(this.files);
});

BOTSIM.on('scene-loaded', function () {
    $('div #botsim-options').hide();
});
