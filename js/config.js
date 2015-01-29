var require = {
    baseUrl: 'assets/js/',
    shim: {
        three: {
            exports: 'THREE'
        },
        OrbitControls: {
            deps: [
                'three'
            ],
            exports: 'THREE'
        },
        ColladaLoader: {
            deps: [
                'three'
            ],
            exports: 'THREE'
        },
        'threejs-stats': {
            exports: 'Stats'
        },
        underscore: {
            exports: '_'
        },
        numericjs: {
            exports: 'numeric'
        },
        jszip: {
            exports: 'JSZip'
        }
    },
    paths: {
        three: '../bower_components/three.js/three.min',
        OrbitControls: './lib/OrbitControls',
        ColladaLoader: './lib/ColladaLoader',
        almond: '../bower_components/almond/almond',
        angular: '../bower_components/angular/angular',
        'angular-bootstrap-colorpicker': '../bower_components/angular-bootstrap-colorpicker/js/bootstrap-colorpicker-module',
        'angular-snap': '../bower_components/angular-snap/angular-snap',
        'angular-strap': '../bower_components/angular-strap/dist/angular-strap',
        'angular-strap.tpl': '../bower_components/angular-strap/dist/angular-strap.tpl',
        'angular-ui-bootstrap-bower': '../bower_components/angular-ui-bootstrap-bower/ui-bootstrap-tpls',
        'angular-xeditable': '../bower_components/angular-xeditable/dist/js/xeditable',
        'jquery-ui': '../bower_components/jquery-ui/jquery-ui',
        jssha: '../bower_components/jssha/src/sha',
        mocha: '../bower_components/mocha/mocha',
        requirejs: '../bower_components/requirejs/require',
        restangular: '../bower_components/restangular/dist/restangular',
        snapjs: '../bower_components/snapjs/snap',
        underscore: '../bower_components/underscore/underscore',
        jquery: '../bower_components/jquery/dist/jquery',
        esprima: '../bower_components/esprima/esprima',
        jszip: '../bower_components/jszip/jszip.min',
        numericjs: '../bower_components/numericjs/src/numeric',
        'threejs-stats': '../bower_components/threejs-stats/Stats',
        'threex-colladaloader': '../bower_components/threex-colladaloader/ColladaLoader'
    },
    packages: [

    ]
};