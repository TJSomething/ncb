/* global THREE, JSZip */

/**
 * @author Thomas Kelly
 */


THREE.KMZLoader = function (colladaLoader) {
    'use strict';

    // Libraries
    var loader = colladaLoader || new THREE.ColladaLoader(),
        textureLibrary = {},
        that = this,
        mimeTypes = {
            jpg: 'image/jpeg',
            jpeg: 'image/jpeg',
            gif: 'image/gif',
            png: 'image/png',
            bmp: 'image/bmp'
        };

    function load (url, readyCallback) {
        var request = new XMLHttpRequest();

        request.onload = function () {
            that.parse(request.response, readyCallback);
        };

        request.open( 'GET', url, true );
        request.responseType = 'arraybuffer';
        request.send( null );
    }

    function parse (data, readyCallback) {
        var zip,
            sceneSource;

        // Open the file
        function loadZip(data) {
            var textureFiles;

            zip = new JSZip(data);

            // Grab the data
            sceneSource = zip.file(/\.dae$/i)[0].asText();
            textureFiles = zip.file(/\.(gif|jpg|jpeg|png|bmp)$/i);

            loadImages(textureFiles);
        }

        function loadImages(textureFiles) {
            var count = 0,
                i;

            // A basic semaphore to use as a barrier
            function signal() {
                count += 1;
            }

            function wait() {
                count -= 1;

                if (count <= 0) {
                    loadModel();
                }
            }

            function loadImage(file) {
                var // Remove the 'models/' prefix
                    name = file.name.slice(7),
                    extension = name.split('.').pop().toLowerCase(),
                    blob = new Blob([file.asArrayBuffer()],
                        { type: mimeTypes[extension] } ),
                    reader = new FileReader();
                    
                signal();

                reader.onload = function () {
                    textureLibrary[name] = reader.result;
                    wait();
                };

                reader.readAsDataURL(blob);
            }

            // This makes sure that we try to load all the of the images
            //  before loading the model
            signal();

            for (i = 0; i < textureFiles.length; i += 1) {
                loadImage(textureFiles[i]);
            }

            wait();
        }

        function loadModel() {
            var imageName,
                xmlParser = new DOMParser(),
                xml;

            // Replace the texture names with data URLs
            for (imageName in textureLibrary) {
                if (textureLibrary.hasOwnProperty(imageName)) {
                    sceneSource =
                        sceneSource.replace(new RegExp(imageName, 'g'),
                            textureLibrary[imageName]);
                }
            }

            // Now that the textures have been inlined with
            //  the scene, we can use the loader to load it.
            xml = xmlParser.parseFromString(sceneSource, 'application/xml');
            
            loader.parse(xml, function (obj) {
                readyCallback(obj);
            });
        }

        loadZip(data);
    }

    return {
        load: load,
        parse: parse
    };
};