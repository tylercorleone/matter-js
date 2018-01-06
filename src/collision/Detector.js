/**
* The `Matter.Detector` module contains methods for detecting collisions given a set of pairs.
*
* @class Detector
*/

// TODO: speculative contacts

var Detector = {};

module.exports = Detector;

var SAT = require('./SAT');
var Pairs = require('./Pairs');
var Sleeping = require('../core/Sleeping');
var Bounds = require('../geometry/Bounds');

var dummyPair = { idA: 1 << 30, idB: (1 << 30) + 1 };

(function() {

    /**
     * Finds all collisions given a list of pairs.
     * @method collisions
     * @param {pair[]} broadphasePairs
     * @param {engine} engine
     * @return {array} collisions
     */
    Detector.collisions = function(broadphasePairs, engine) {
        var pairs = engine.pairs,
            oldPairs = pairs.list,
            newPairs = [],
            collisionStart,
            collisionActive,
            collisionEnd;

        // @if DEBUG
        var metrics = engine.metrics;
        // @endif

        var engineEvents = engine.events;
        var hasCollisionEvent = !!(
            engineEvents.collisionStart ||
            engineEvents.collisionActive ||
            engineEvents.collisionEnd
        );

        if (hasCollisionEvent) {
            collisionStart = [];
            collisionActive = [];
            collisionEnd = [];
            pairs.collisionStart = collisionStart;
            pairs.collisionActive = collisionActive;
            pairs.collisionEnd = collisionEnd;
        }

        oldPairs.push(dummyPair);

        var pairIndex = 0,
            oldPair = oldPairs[pairIndex++];

        for (var i = 0; i < broadphasePairs.length; i++) {
            var bodyA = broadphasePairs[i][0], 
                bodyB = broadphasePairs[i][1];

            if ((bodyA.isStatic || bodyA.isSleeping) && (bodyB.isStatic || bodyB.isSleeping))
                continue;
            
            if (!Detector.canCollide(bodyA.collisionFilter, bodyB.collisionFilter))
                continue;

            // @if DEBUG
            metrics.midphaseTests += 1;
            // @endif

            // mid phase
            if (Bounds.overlaps(bodyA.bounds, bodyB.bounds)) {
                for (var j = bodyA.parts.length > 1 ? 1 : 0; j < bodyA.parts.length; j++) {
                    var partA = bodyA.parts[j];

                    for (var k = bodyB.parts.length > 1 ? 1 : 0; k < bodyB.parts.length; k++) {
                        var partB = bodyB.parts[k];

                        if ((partA === bodyA && partB === bodyB) || Bounds.overlaps(partA.bounds, partB.bounds)) {
                            // narrow phase
                            var collision = SAT.collides(partA, partB);

                            // @if DEBUG
                            metrics.narrowphaseTests += 1;
                            // @endif

                            if (collision) {
                                newPairs.push(collision);

                                if (hasCollisionEvent) {
                                    // Check old pairs to determine which collisions are new
                                    // and which collisions are not active anymore
                                    var idA = collision.bodyA.id;
                                    var idB = collision.bodyB.id;
                                    while (oldPair.bodyA.id < idA || (oldPair.bodyA.id === idA && oldPair.bodyB.id < idB)) {
                                        collisionEnd.push(oldPair);
                                        oldPair = oldPairs[pairIndex++];
                                    }

                                    if (oldPair.bodyA.id === idA && oldPair.bodyB.id === idB) {
                                        // Pair was already active
                                        collisionActive.push(newPair);
                                        oldPair = oldPairs[pairIndex++];
                                    } else {
                                        // Pair could not be found, collision is new
                                        collisionStart.push(newPair);
                                    }
                                }

                                // @if DEBUG
                                metrics.narrowDetections += 1;
                                // @endif
                            }
                        }
                    }
                }
            }
        }

        if (hasCollisionEvent) {
            pairIndex -= 1;
            while (pairIndex < oldPairs.length - 1) {
                collisionEnd.push(oldPairs[pairIndex++]);
            }
        }

        pairs.list = newPairs;
    };

    /**
     * Returns `true` if both supplied collision filters will allow a collision to occur.
     * See `body.collisionFilter` for more information.
     * @method canCollide
     * @param {} filterA
     * @param {} filterB
     * @return {bool} `true` if collision can occur
     */
    Detector.canCollide = function(filterA, filterB) {
        if (filterA.group === filterB.group && filterA.group !== 0)
            return filterA.group > 0;

        return (filterA.mask & filterB.category) !== 0 && (filterB.mask & filterA.category) !== 0;
    };

})();
