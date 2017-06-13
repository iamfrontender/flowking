'use strict';

/**
 * Defines the shape of a boid
 * @type {Array<Number[]>} - array of position vectors
 */
const cursor = [
    [-14, 6],
    [-12, 1],
    [-22, 1],
    [-22, -1],
    [-12, -1],
    [-14, -6],
    [0, 0]
];

/**
 * Holds helper utilities
 */
const utils = {

    /**
     * Creates random float number within given interval
     *
     * @param {Number} bottom
     * @param {Number} top
     * @return {Number}
     */
    rand(bottom, top) {
        if (top === undefined) {
            top = bottom;
            bottom = 0;
        }

        return bottom + (Math.random() * (top - bottom));
    },

    /**
     * Double-dog-leg hypothenuse approximation
     *
     * @param {Number} a
     * @param {Number} b
     * @return {Number} c - hypothenuse length
     */
    hypot(a, b) {
        a = Math.abs(a);
        b = Math.abs(b);
        let lo = Math.min(a, b);
        let hi = Math.max(a, b);
        return hi + 3 * lo / 32 + Math.max(0, 2 * lo - hi) / 8 + Math.max(0, 4 * lo - hi) / 16;
    },

    /**
     * Normalizes given vector
     *
     * @param {Number} x
     * @param {Number} y
     * @return {Number[]}
     */
    normalize([x, y]) {
        let length = Math.sqrt(x * x + y * y);
        return [x / length, y / length];
    }
};

/**
 * Implements boids flocking behavior which
 * could be then rendered
 */
class Boids {
    constructor(opts = {}) {
        this.speedLimitRoot = opts.speedLimit || 0;
        this.accelerationLimitRoot = opts.accelerationLimit || 1;
        this.speedLimit = Math.pow(this.speedLimitRoot, 2);
        this.accelerationLimit = Math.pow(this.accelerationLimitRoot, 2);
        this.separationDistance = Math.pow(opts.separationDistance || 60, 2);
        this.alignmentDistance = Math.pow(opts.alignmentDistance || 180, 2);
        this.cohesionDistance = Math.pow(opts.cohesionDistance || 180, 2);
        this.separationForce = opts.separationForce || 0.15;
        this.cohesionForce = opts.cohesionForce || 0.1;
        this.alignmentForce = opts.alignmentForce || opts.alignment || 0.25;
        this.attractors = opts.attractors || [];

        let quarterWidth = window.innerWidth / 4;
        let quarterHeight = window.innerHeight / 4;

        this.boids = Array(opts.boids || 50)
            .fill(0)
            .map(() => ({
                speed: [
                    utils.rand(-25, 25), utils.rand(-25, 25)
                ],
                position: [utils.rand(-quarterWidth, quarterWidth), utils.rand(-quarterHeight, quarterHeight)],
                acceleration: [0, 0]
            }));
    }

    tick() {
        this.boids.forEach(boid => {
            let [
                sForceX, sForceY,
                cForceX, cForceY,
                aForceX, aForceY
            ] = Array(6).fill(0);
            let length;

            // Attractors
            this.attractors.forEach(([attractorX, attractorY, radius, force]) => {
                let [spareX, spareY] = [
                    boid.position[0] - attractorX,
                    boid.position[1] - attractorY
                ];
                let dist = (spareX * spareX) + (spareY * spareY);

                if (dist < radius * radius) {
                    let length = utils.hypot(spareX, spareY);
                    boid.speed[0] -= force * spareX / length || 0;
                    boid.speed[1] -= force * spareY / length || 0;
                }
            });

            this.boids.forEach(anotherBoid => {
                if (boid !== anotherBoid) {
                    let [spareX, spareY] = [
                        boid.position[0] - anotherBoid.position[0],
                        boid.position[1] - anotherBoid.position[1]
                    ];
                    let dist = (spareX * spareX) + (spareY * spareY);

                    if (dist < this.separationDistance) {
                        sForceX += spareX;
                        sForceY += spareY;
                    } else {
                        if (dist < this.cohesionDistance) {
                            cForceX += spareX;
                            cForceY += spareY;
                        }

                        if (dist < this.alignmentDistance) {
                            aForceX += anotherBoid.speed[0];
                            aForceY += anotherBoid.speed[1];
                        }
                    }
                }
            });

            length = utils.hypot(sForceX, sForceY);
            boid.acceleration[0] += (this.separationForce * sForceX / length) || 0;
            boid.acceleration[1] += (this.separationForce * sForceY / length) || 0;

            length = utils.hypot(cForceX, cForceY);
            boid.acceleration[0] -= (this.cohesionForce * cForceX / length) || 0;
            boid.acceleration[1] -= (this.cohesionForce * cForceY / length) || 0;

            length = utils.hypot(aForceX, aForceY);
            boid.acceleration[0] -= (this.alignmentForce * aForceX / length) || 0;
            boid.acceleration[1] -= (this.alignmentForce * aForceY / length) || 0;

            if (this.accelerationLimit) {
                let dist = boid.acceleration[0] * boid.acceleration[0] + boid.acceleration[1] * boid.acceleration[1];

                if (dist > this.accelerationLimit) {
                    let ratio = this.accelerationLimitRoot / utils.hypot(boid.acceleration[0], boid.acceleration[1]);
                    boid.acceleration[0] *= ratio;
                    boid.acceleration[1] *= ratio;
                }
            }

            boid.speed[0] += boid.acceleration[0];
            boid.speed[1] += boid.acceleration[1];

            if (this.speedLimit) {
                let dist = boid.speed[0] * boid.speed[0] + boid.speed[1] * boid.speed[1];

                if (dist > this.speedLimit) {
                    let ratio = this.speedLimitRoot / utils.hypot(boid.speed[0], boid.speed[1]);
                    boid.speed[0] *= ratio;
                    boid.speed[1] *= ratio;
                }
            }

            boid.position[0] += boid.speed[0];
            boid.position[1] += boid.speed[1];
        });
    }

    draw() {
        this.boids.forEach((boid) => {
            let [x, y] = boid.position;
            let halves = flow.halves;

            boid.position[0] = x > halves.width ? -halves.width : -x > halves.width ? halves.width : x;
            boid.position[1] = y > halves.height ? -halves.height : -y > halves.height ? halves.height : y;

            drawCursor(boid);
        });
    }
}

/**
 * Implements uber-precise chronodisplay
 */
class Clocks {
    constructor(selector) {
        this.el = document.querySelector(selector);
    }

    run() {
        this.tick();
        setInterval(() => this.tick(), 5000);
    }

    tick() {
        let now = new Date();
        this.el.textContent = `${now.getHours() % 12}:${now.getMinutes()} ${now.getHours() >= 12 ? 'PM' : 'AM'}`;
    }
}

////////////////////////////////////////////////////////////////////////

const ctx = flow.getContext('2d');
const boids = new Boids({
    boids: 140,            // The amount of boids to use
    speedLimit: 2,          // Max steps to take per tick
    accelerationLimit: .05,   // Max acceleration per tick
    separationDistance: 60, // Radius at which boids avoid others
    alignmentDistance: 1000, // Radius at which boids align with others
    cohesionDistance: 1000,  // Radius at which boids approach others
    separationForce: 50,  // Speed to avoid at
    alignmentForce: 50,   // Speed to align with other boids
    cohesionForce: 50,     // Speed to move towards other boids
    attractors: []
});
const clocks = new Clocks('#time');

window.addEventListener('resize', onResize);
document.addEventListener('mouseenter', onMouseEnter);
document.addEventListener('mousemove', onMousemove);
document.addEventListener('mouseleave', onMouseLeave);

onResize();
run();
clocks.run();

////////////////////////////////////////////////////////////////////////

function run() {
    ctx.clearRect(0, 0, flow.width, flow.height);
    ctx.fillStyle = 'white';

    ctx.save();
    ctx.translate(flow.halves.width, flow.halves.height);

    boids.tick();
    boids.draw(ctx);

    ctx.restore();

    window.requestAnimationFrame(run);
}

function onResize() {
    flow.width = window.innerWidth;
    flow.height = window.innerHeight;
    flow.halves = {
        width: flow.width / 2,
        height: flow.height / 2
    };
}

function onMouseEnter(e) {
    boids.attractors.push([
        e.x, e.y, 150, .4
    ]);
}

function onMouseLeave() {
    boids.attractors.shift();
}

function onMousemove(e) {
    boids.attractors[0][0] = e.x - flow.halves.width;
    boids.attractors[0][1] = e.y - flow.halves.height;
}

function drawCursor(boid) {
    let position = boid.position;
    let direction = utils.normalize(boid.speed);

    direction = Math.atan2(direction[0], direction[1]);

    ctx.strokeStyle = 'black';
    ctx.strokeWidth = '2px';
    ctx.fillStyle = 'white';

    ctx.beginPath();

    let cos = Math.cos(direction);
    let sin = Math.sin(direction);
    let coords = cursor.map(([x, y]) => ([x * sin - y * cos, x * cos + y * sin]));

    ctx.moveTo(position[0], position[1]);
    coords.forEach(([x, y]) => {
        ctx.lineTo(position[0] + x, position[1] + y);
    });

    ctx.fill();
    ctx.stroke();
}

