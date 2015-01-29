all: motion.js doc

motion.js: build_motion.py
	python build_motion.py > motion.js

doc: *.js conf.json
	~/node_modules/.bin/jsdoc -d doc -c conf.json ./

clean:
	rm -r motion.js doc
