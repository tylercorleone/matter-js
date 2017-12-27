/**
* The `Matter.SAT` module contains methods for detecting collisions using the Separating Axis Theorem.
*
* @class SAT
*/

// TODO: true circles and curves

var SAT = {};

module.exports = SAT;

var Vertices = require('../geometry/Vertices');
var Vector = require('../geometry/Vector');

var tempBounds1 = { min: 0, max: 0 };
var tempBounds2 = { min: 0, max: 0 };

(function() {

    /**
     * Detect collision between two bodies using the Separating Axis Theorem.
     * @method collides
     * @param {body} bodyA
     * @param {body} bodyB
     * @return {collision} collision
     */
    SAT.collides = function(bodyA, bodyB) {
        if (bodyA.circleRadius > 0) {
            if (bodyB.circleRadius > 0) {
                return SAT.collidesCircleCircle(bodyA, bodyB);
            } else {
                return SAT.collidesCirclePolygon(bodyA, bodyB, false);
            }
        } else {
            if (bodyB.circleRadius > 0) {
                return SAT.collidesCirclePolygon(bodyB, bodyA, true);
            } else {
                return SAT.collidesPolygonPolygon(bodyA, bodyB);
            }
        }
    };

    SAT.collidesCirclePolygon = function(circle, polygon, invert) {
        var overlap = SAT._overlapAxesWithCircle(circle, polygon.vertices, polygon.axes);
        if (overlap.overlap <= 0) {
            return null;
        }

        var bodyA,
            bodyB;
        if (invert) {
            bodyA = polygon;
            bodyB = circle;
        } else {
            bodyA = circle;
            bodyB = polygon;
        }

        // ensure normal is facing away from bodyA
        var positionA = bodyA.position,
            positionB = bodyB.position,
            axis = overlap.axis,
            normal;

        if (axis.x * (positionB.x - positionA.x) + axis.y * (positionB.y - positionA.y) < 0) {
            normal = {
                x: axis.x,
                y: axis.y
            };
        } else {
            normal = {
                x: -axis.x,
                y: -axis.y
            };
        }

        // find support points, there is always either exactly one or two
        var potentialSupports = SAT._findSupports(circle, polygon.vertices, normal),
            circleRadius = circle.circleRadius,
            supports = [],
            potentialSupport,
            distance;

        // find the supports from the polygon that are inside the circle
        potentialSupport = potentialSupports[0];
        distance = SAT._getDistance(potentialSupport, positionA);
        if (distance < circleRadius)
            supports.push(potentialSupport);

        potentialSupport = potentialSupports[1];
        distance = SAT._getDistance(potentialSupport, positionA);
        if (distance < circleRadius)
            supports.push(potentialSupport);

        // find the supports from the circle that are inside the polygon
        if (supports.length < 2) {
            // var circlePosition = circle.position,
            //     polygonPosition = polygon.position,
            //     distanceX = polygonPosition.x - circlePosition.x,
            //     distanceY = polygonPosition.y - circlePosition.y,
            //     distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY),
            //     supportX = circlePosition.x + distanceX / distance * circleRadius,
            //     supportY = circlePosition.y + distanceY / distance * circleRadius;

            // if (supports.length < 1) {
            //     var a = 5.1;
            //     supports.push(Vertices.createVertex(circle, 0, supportX + a * distanceY / distance, supportY - a * distanceX / distance));
            //     supports.push(Vertices.createVertex(circle, 0, supportX - a * distanceY / distance, supportY + a * distanceX / distance));
            // } else {
            //     supports.push(Vertices.createVertex(circle, 0, supportX, supportY));
            // }

            var potentialSupportsA =  SAT._findSupports(bodyB, circle.vertices, Vector.neg(normal));
                
            if (Vertices.contains(bodyB.vertices, potentialSupportsA[0]))
                supports.push(potentialSupportsA[0]);

            if (supports.length < 2 && Vertices.contains(bodyB.vertices, potentialSupportsA[1]))
                supports.push(potentialSupportsA[1]);
        }

        // account for the edge case of overlapping but no vertex containment
        if (supports.length < 1)
            supports = [potentialSupports[0]];

        return SAT._createCollision(bodyA, bodyB, overlap.overlap, normal, supports);
    };

    SAT._getDistance = function(vertex, position) {
        var distanceX = vertex.x - position.x,
            distanceY = vertex.y - position.y;
        return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    };

    /**
     * Find the overlap between a circle and a set of vertices
     * @method _overlapAxesWithCircle
     * @private
     * @param {} circle
     * @param {} vertices
     * @param {} axes
     * @return result
     */
    SAT._overlapAxesWithCircle = function(circle, vertices, axes) {
        var projectionA = tempBounds1,
            projectionB = tempBounds2,
            pointA = Vector._temp[0], 
            pointB = Vector._temp[1],
            result = { overlap: Number.MAX_VALUE, axis: null },
            overlap,
            axis,
            unitX,
            unitY,
            position = circle.position,
            positionX = position.x,
            positionY = position.y,
            circleRadius = circle.circleRadius;

        var axesCount = axes.length;
        var totalAxesCount = vertices.length + axesCount;

        for (var i = 0; i < totalAxesCount; i++) {
            if (i < axes.length) {
                axis = axes[i];
            } else {
                var vertex = vertices[i - axesCount];
                var distanceX = vertex.x - position.x,
                    distanceY = vertex.y - position.y,
                    distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

                axis = {
                    x: distanceX / distance,
                    y: distanceY / distance
                };
            }

            SAT._projectToAxis(projectionA, vertices, axis);

            // projecting sircle onto axis
            unitX = axis.x * circleRadius;
            unitY = axis.y * circleRadius;

            pointA.x = positionX + unitX;
            pointA.y = positionY + unitY;

            pointB.x = positionX - unitX;
            pointB.y = positionY - unitY;

            var a = Vector.dot(pointA, axis),
                b = Vector.dot(pointB, axis);

            if (a < b) {
                projectionB.min = a;
                projectionB.max = b;
            } else {
                projectionB.min = b;
                projectionB.max = a;
            }

            overlap = Math.min(projectionA.max - projectionB.min, projectionB.max - projectionA.min);

            if (overlap <= 0) {
                result.overlap = overlap;
                return result;
            }

            if (overlap < result.overlap) {
                result.overlap = overlap;
                result.axis = axis;
            }
        }

        return result;
    };

    SAT.collidesCircleCircle = function(bodyA, bodyB) {
        var radiusA = bodyA.circleRadius,
            radiusB = bodyB.circleRadius,
            positionA = bodyA.position,
            positionB = bodyB.position,
            distanceX = positionA.x - positionB.x,
            distanceY = positionA.y - positionB.y,
            distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

        if (distance > radiusA + radiusB) {
            return null;
        }

        var normal = {
            x: distanceX / distance,
            y: distanceY / distance,
        };

        var depth = radiusA + radiusB - distance,
            supportX = positionA.x + normal.x * radiusA,
            supportY = positionA.y + normal.y * radiusA,
            support = Vertices.createVertex(bodyB, 0, supportX, supportY);

        return SAT._createCollision(bodyA, bodyB, depth, normal, [support]);
    };

    SAT.collidesPolygonPolygon = function(bodyA, bodyB) {
        var overlapAB =  SAT._overlapAxes(bodyA.vertices, bodyB.vertices, bodyA.axes);
        if (overlapAB.overlap <= 0) {
            return null;
        }

        var overlapBA =  SAT._overlapAxes(bodyB.vertices, bodyA.vertices, bodyB.axes);
        if (overlapBA.overlap <= 0) {
            return null;
        }

        var minOverlap = (overlapAB.overlap < overlapBA.overlap) ? overlapAB : overlapBA;

        // ensure normal is facing away from bodyA
        var positionA = bodyA.position,
            positionB = bodyB.position,
            axis = minOverlap.axis,
            normal;

        if (axis.x * (positionB.x - positionA.x) + axis.y * (positionB.y - positionA.y) < 0) {
            normal = {
                x: axis.x,
                y: axis.y
            };
        } else {
            normal = {
                x: -axis.x,
                y: -axis.y
            };
        }

        // find support points, there is always either exactly one or two
        var verticesA = bodyA.vertices,
            verticesB = bodyB.vertices,
            potentialSupportsB = SAT._findSupports(bodyA, verticesB, normal),
            supports = [];

        // find the supports from bodyB that are inside bodyA
        if (Vertices.contains(verticesA, potentialSupportsB[0]))
            supports.push(potentialSupportsB[0]);

        if (Vertices.contains(verticesA, potentialSupportsB[1]))
            supports.push(potentialSupportsB[1]);

        // find the supports from bodyA that are inside bodyB
        if (supports.length < 2) {
            var potentialSupportsA =  SAT._findSupports(bodyB, verticesA, Vector.neg(normal));
                
            if (Vertices.contains(verticesB, potentialSupportsA[0]))
                supports.push(potentialSupportsA[0]);

            if (supports.length < 2 && Vertices.contains(verticesB, potentialSupportsA[1]))
                supports.push(potentialSupportsA[1]);
        }

        // account for the edge case of overlapping but no vertex containment
        if (supports.length < 1)
            supports = [potentialSupportsB[0]];

        return SAT._createCollision(bodyA, bodyB, minOverlap.overlap, normal, supports);
    };

    /**
     * Creates a collision object
     * @method _createCollision
     * @private
     * @param {body} bodyA
     * @param {body} bodyB
     * @param {depth} depth
     * @param {normal} normal
     * @param {supports} support vertices
     * @return result
     */
    SAT._createCollision = function(bodyA, bodyB, depth, normal, supports) {
        return {
            bodyA: bodyA,
            bodyB: bodyB,
            depth: depth,
            parentA: bodyA.parent,
            parentB: bodyB.parent,
            normal: normal,
            tangent: {
                x: -normal.y,
                y: normal.x
            },
            penetration: {
                x: normal.x * depth,
                y: normal.y * depth
            },
            supports: supports
        };
    };

    /**
     * Find the overlap between two sets of vertices.
     * @method _overlapAxes
     * @private
     * @param {} verticesA
     * @param {} verticesB
     * @param {} axes
     * @return result
     */
    SAT._overlapAxes = function(verticesA, verticesB, axes) {
        var projectionA = tempBounds1,
            projectionB = tempBounds2,
            result = { overlap: Number.MAX_VALUE, axis: null },
            overlap,
            axis;

        for (var i = 0; i < axes.length; i++) {
            axis = axes[i];

            SAT._projectToAxis(projectionA, verticesA, axis);
            SAT._projectToAxis(projectionB, verticesB, axis);

            overlap = Math.min(projectionA.max - projectionB.min, projectionB.max - projectionA.min);

            if (overlap <= 0) {
                result.overlap = overlap;
                return result;
            }

            if (overlap < result.overlap) {
                result.overlap = overlap;
                result.axis = axis;
            }
        }

        return result;
    };

    /**
     * Projects vertices on an axis and returns an interval.
     * @method _projectToAxis
     * @private
     * @param {} projection
     * @param {} vertices
     * @param {} axis
     */
    SAT._projectToAxis = function(projection, vertices, axis) {
        var min = Vector.dot(vertices[0], axis),
            max = min;

        for (var i = 1; i < vertices.length; i += 1) {
            var dot = Vector.dot(vertices[i], axis);

            if (dot > max) { 
                max = dot; 
            } else if (dot < min) { 
                min = dot; 
            }
        }

        projection.min = min;
        projection.max = max;
    };
    
    /**
     * Finds supporting vertices given a body and a set of vertices along a given direction using hill-climbing.
     * @method _findSupports
     * @private
     * @param {} body
     * @param {} vertices
     * @param {} normal
     * @return [vector]
     */
    SAT._findSupports = function(body, vertices, normal) {
        var nearestDistance = Number.MAX_VALUE,
            vertexToBodyX,
            vertexToBodyY,
            position = body.position,
            positionX = position.x,
            positionY = position.y,
            distance,
            vertex,
            vertexA,
            vertexB;

        // find closest vertex
        for (var i = 0; i < vertices.length; i++) {
            vertex = vertices[i];
            vertexToBodyX = vertex.x - positionX;
            vertexToBodyY = vertex.y - positionY;
            distance = -(normal.x * vertexToBodyX + normal.y * vertexToBodyY);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                vertexA = vertex;
            }
        }

        // find next closest vertex using the two connected to it
        var prevIndex = vertexA.index - 1 >= 0 ? vertexA.index - 1 : vertices.length - 1;
        vertex = vertices[prevIndex];
        vertexToBodyX = vertex.x - positionX;
        vertexToBodyY = vertex.y - positionY;
        nearestDistance = -(normal.x * vertexToBodyX + normal.y * vertexToBodyY);
        vertexB = vertex;

        var nextIndex = (vertexA.index + 1) % vertices.length;
        vertex = vertices[nextIndex];
        vertexToBodyX = vertex.x - positionX;
        vertexToBodyY = vertex.y - positionY;
        distance = -(normal.x * vertexToBodyX + normal.y * vertexToBodyY);
        if (distance < nearestDistance) {
            vertexB = vertex;
        }

        return [vertexA, vertexB];
    };

})();
