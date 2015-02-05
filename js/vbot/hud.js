define(['angular', 'jquery'],
function (angular, $) {
    'use strict';

    return function (app) {
        // This will hold the HUD updating function. We stick a dummy here,
        // just in case it gets called.
        var module = { update: function () {} };
        app.on('scene-ready', function () {
            // Insert a DOM element on top of the canvas
            var hudElem = document.createElement('div');
            hudElem.id = 'vbot-hud';

            app.container.appendChild(hudElem);

            // Move the performance stats into the HUD
            hudElem.appendChild(document.createTextNode('Rendering:'));
            hudElem.appendChild(app.renderStats.domElement);
            hudElem.appendChild(document.createTextNode('Logic:'));
            hudElem.appendChild(app.logicStats.domElement);

            // I think this makes a new scope, attaches it to the new HUD element,
            // and pulls out a closure that updates the element
            angular.injector(['ng']).invoke(['$compile', '$rootScope', function(compile, rootScope){
                var scope = rootScope.$new();
                scope.sensors = {};
                var template =
                        '<ul>' +
                        '    <li>Speed: {{ sensors.speed | number }}</li>' +
                        '    <li>Angular velocity: {{ sensors.angularVelocity | number }}</li>' +
                        '    <li>Odometer: {{ sensors.odometer | number }}</li>' +
                        '    <li>Heading: {{ sensors.compass | number }}</li>' +
                        '    <li>Left-hand object: {{ sensors.arms.left.held || "&lt;none&gt;"}}</li>' +
                        '    <li>Right-hand object: {{ sensors.arms.right.held || "&lt;none&gt;"}}</li>' +
                        '    <li>Expression: {{ sensors.expression || "&lt;none&gt;"}}</li>' +
                        '    <li>Collisions:' +
                        '        <ul>' +
                        '            <li>Top: {{sensors.collision.top}}</li>' +
                        '            <li>Bottom: {{sensors.collision.bottom}}</li>' +
                        '            <li>Left: {{sensors.collision.left}}</li>' +
                        '            <li>Right: {{sensors.collision.right}}</li>' +
                        '            <li>Front: {{sensors.collision.front}}</li>' +
                        '            <li>Back: {{sensors.collision.back}}</li>' +
                        '        </ul>' +
                        '    </li>' +
                        '    <li>Keys:' +
                        '        <ul>' +
                        '            <li ng-repeat="(key,val) in sensors.keys">' +
                        '                {{key}}: {{val}}' +
                        '            </li>' +
                        '        </ul>' +
                        '    </li>' +
                        '</ul>';

                var result = compile(template)(scope);
                scope.$digest();
                hudElem.appendChild(result[0]);
                
                module.update = function (newSensors) {
                    scope.sensors = newSensors;
                    scope.$apply();
                }

                // Add a toggling hide/show
                var hidden = false;
                hudElem.addEventListener('click', function (e) {
                    if (hidden) {
                        $(hudElem).animate({
                            left: '-25px'
                        });
                        hidden = false;
                    } else {
                        $(hudElem).animate({
                            left: (-hudElem.scrollWidth + 5) + 'px'
                        });
                        hidden = true;
                    }
                });

            }]);
        });

        return module;
    }
});
