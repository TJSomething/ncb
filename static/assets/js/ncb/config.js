var require = {
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
        }
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
        numeric: '../lib/numeric-1.2.6'
    }
};