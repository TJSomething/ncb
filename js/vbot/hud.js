'use strict';
var $ = require('jquery');

module.exports = function (app) {
    // This will hold the HUD updating function. We stick a dummy here,
    // just in case it gets called.
    var module = { update: function () {} };
    app.on('scene-ready', function () {
        // Insert a DOM element on top of the canvas
        var hudElem = document.createElement('div');
        hudElem.id = 'vbot-hud';

        app.container.appendChild(hudElem);

        // Add a directive to inject raw HTMLElements
        angular.module('vbot-hud', ['ng'])
        .directive('rawElement', function () {
            return {
                link: function (scope, element, attrs) {
                    element.replaceWith(scope[attrs.rawElement]);
                }
            };
        });

        angular.injector(['vbot-hud']).invoke(['$compile', '$rootScope',
        function(compile, rootScope){
            var scope = rootScope.$new();
            scope.sensors = {};
            scope.renderStats = app.renderStats.domElement;
            scope.logicStats = app.logicStats.domElement;
            var template =
                    '<ul>' +
                    '    <li>Rendering: <div raw-element="renderStats" /></li>' +
                    '    <li>Logic: <div raw-element="logicStats" /></li>' +
                    '    <li>Sensors:' +
                    '        <ul>' +
                    '            <li>Speed: {{ sensors.speed | number }}</li>' +
                    '            <li>Angular velocity: {{ sensors.angularVelocity | number }}</li>' +
                    '            <li>Odometer: {{ sensors.odometer | number }}</li>' +
                    '            <li>Heading: {{ sensors.compass | number }}</li>' +
                    '            <li>Left-hand object: {{ sensors.arms.left.held || "&lt;none&gt;"}}</li>' +
                    '            <li>Right-hand object: {{ sensors.arms.right.held || "&lt;none&gt;"}}</li>' +
                    '            <li>Expression: {{ sensors.expression || "&lt;none&gt;"}}</li>' +
                    '            <li>Collisions:' +
                    '                <ul>' +
                    '                    <li>Top: {{sensors.collision.top}}</li>' +
                    '                    <li>Bottom: {{sensors.collision.bottom}}</li>' +
                    '                    <li>Left: {{sensors.collision.left}}</li>' +
                    '                    <li>Right: {{sensors.collision.right}}</li>' +
                    '                    <li>Front: {{sensors.collision.front}}</li>' +
                    '                    <li>Back: {{sensors.collision.back}}</li>' +
                    '                </ul>' +
                    '            </li>' +
                    '            <li>Keys:' +
                    '                <ul>' +
                    '                    <li ng-repeat="(key,val) in sensors.keys">' +
                    '                        {{key}}: {{val}}' +
                    '                    </li>' +
                    '                </ul>' +
                    '            </li>' +
                    '            <li>Actions:' +
                    '                <ul>' +
                    '                    <li ng-repeat="action in actions">' +
                    '                        {{action}}' +
                    '                    </li>' +
                    '                </ul>' +
                    '            </li>' +
                    '        </ul>' +
                    '    </li>' +
                    '</ul>';

            var result = compile(template)(scope);
            scope.$digest();
            hudElem.appendChild(result[0]);
            
            module.update = function (key, value) {
                scope[key] = value;
                scope.$apply();
            }

            // Add a toggling hide/show
            var hidden = false;
            function show() {
                $(hudElem).animate({
                    left: '-25px'
                }, { queue: false });
            }
            function hide() {
                $(hudElem).animate({
                    left: (-hudElem.scrollWidth + 5) + 'px'
                }, { queue: false });
            }

            // Show on hover
            hudElem.addEventListener('mouseenter', function (e) {
                if (hidden) {
                    show();
                }
            });
            hudElem.addEventListener('mouseleave', function (e) {
                if (hidden) {
                    hide();
                }
            });
            // Toggle showing with click
            hudElem.addEventListener('click', function (e) {
                if (hidden) {
                    show();
                    hidden = false;
                } else {
                    hide();
                    hidden = true;
                }
            });

        }]);
    });

    return module;
}
