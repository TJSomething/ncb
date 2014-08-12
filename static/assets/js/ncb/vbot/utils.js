/* jslint browser: true */
/* global THREE: false, mod: true */

define(['three'],
function (THREE) {
  'use strict';

  /** @exports vbot/utils */
  var utils = {};

  //////////////// Convenience functions and objects ////////////////////
  utils.requestAnimationFrame = window.requestAnimationFrame ||
                                window.mozRequestAnimationFrame ||
                                window.webkitRequestAnimationFrame ||
                                window.msRequestAnimationFrame;
  // If it's called with the utils context, then you get an illegal invocation
  // error.
  utils.requestAnimationFrame = utils.requestAnimationFrame.bind(window);

  // ES6 Sign function
  var isNaN = Number.isNaN;

  Object.defineProperty(Math, 'sign', {
    value: function sign(value) {
      var n = +value;
      if (isNaN(n))
        return n /* NaN */;

      if (n === 0)
        return n; // Keep the sign of the zero.

      return (n < 0) ? -1 : 1;
    },
    configurable: true,
    enumerable: false,
    writable: true
  });

  /**
   * A real modulo operator that works properly on negative numbers.
   *
   * @memberof module:vbot/utils
   * @param  {Number} n dividend
   * @param  {Number} d divisor
   * @return {Number}   the result
   */
  function mod(n, d) {
      return n - (d * Math.floor(n / d));
  }
  utils.mod = mod;

  /**
   * The methods for the publisher pattern mixin. Taken from Javascript Patterns
   * by Stoyan Stefanov.
   *
   * @memberof module:vbot/utils
   * @mixin
   */
  var publisher = {
      subscribers: {
          any: []
      },
      /**
       * Adds a listener for an event.
       *
       * @instance
       * @param  {string} type          the event to fire on
       * @param  {(string|Function)} fn the function to fire on that event
       * @param  {Object}   context     the object to be used as the context
       *                                upon firing
       */
      on: function (type, fn, context) {
          type = type || 'any';
          fn = typeof fn === 'function' ? fn : context[fn];

          if (typeof this.subscribers[type] === 'undefined') {
              this.subscribers[type] = [];
          }
          this.subscribers[type].push({fn: fn, context: context || this});
      },
      /**
       * Removes a listener for an event.
       *
       * @instance
       * @param  {string}   type    the event to remove the listener from
       * @param  {Function} fn      the function to remove from that event
       * @param  {Object}   context the object to be used as the context upon
       *                            firing
       */
      remove: function (type, fn, context) {
          this.visitSubscribers('unsubscribe', type, fn, context);
      },
      /**
       * Fires an event.
       *
       * @instance
       * @param  {string} type   the event to fire
       * @param  {*} publication the argument to pass to the listeners
       */
      fire: function (type, publication) {
          this.visitSubscribers('publish', type, publication);
      },
      visitSubscribers: function (action, type, arg, context) {
          var pubtype = type || 'any',
              subscribers = this.subscribers[pubtype],
              i,
              max = subscribers ? subscribers.length : 0;

          for (i = 0; i < max; i += 1) {
              if (action === 'publish') {
                  subscribers[i].fn.call(subscribers[i].context, arg);
              } else {
                  if (subscribers[i].fn === arg && subscribers[i].context === context) {
                      subscribers.splice(i, 1);
                  }
              }
          }
      }
  };

  /**
   * Decorates an object with the observer pattern. Taken from
   * Javascript Patterns by Stoyan Stefanov.
   *
   * @memberof module:vbot/utils
   * @param  {Object} o the object to decorate
   */
  function makePublisher(o) {
      var i;
      for (i in publisher) {
          if (publisher.hasOwnProperty(i) && typeof publisher[i] === 'function') {
              o[i] = publisher[i];
          }
      }
      o.subscribers = {any: []};
  }
  utils.makePublisher = makePublisher;

  return utils;
});