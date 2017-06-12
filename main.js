'use strict';

const utils = {
    rand(bottom, top) {
        if (top === undefined) {
            top = bottom;
            bottom = 0;
        }

        return bottom + (Math.random() * (top - bottom));
    },

    hypot(a, b) {
        a = Math.abs(a);
        b = Math.abs(b);
        let lo = Math.min(a, b);
        let hi = Math.max(a, b);
        return hi + 3 * lo / 32 + Math.max(0, 2 * lo - hi) / 8 + Math.max(0, 4 * lo - hi) / 16;
    }
};

class EventEmitter {
    static wrap(entity) {
        let emitter = new EventEmitter();

        entity.on = emitter.on;
        entity.emit = emitter.emit;
        entity.listeners = emitter.listeners;
    }

    constructor() {
        this.listeners = {};
    }

    on(event, listener) {
        this.listeners[event] = this.listeners[event] || [];
        this.listeners[event].push(listener);
    }

    emit(event, ...args) {
        if (this.listeners[event]) {
            this.listeners[event].forEach(listener => {
                try {
                    listener(...args);
                } catch(e) {
                    console.error(`EventEmitter: event ${event} failed:`, e);
                }
            });
        }
    }
}

class Boids {
    constructor(opts = {}) {
        EventEmitter.wrap(this);

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
        this.cb = opts.cb || (() => {});

        this.boids = Array(opts.boids || 50)
            .fill(0)
            .map(() => ({
                speed: [utils.rand(-25, 0), utils.rand(-25, 0)],
                position: [utils.rand(-window.innerWidth / 4, window.innerWidth / 4), utils.rand(-window.innerHeight / 4, window.innerHeight / 4)],
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

                if (dist < radius) {
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

        this.cb(this.boids);
    }
}

////////////////////////////////////////////////////////////////////////

const boids = new Boids({
    boids: 100,              // The amount of boids to use
    speedLimit: 2,          // Max steps to take per tick
    accelerationLimit: .05,   // Max acceleration per tick
    separationDistance: 60, // Radius at which boids avoid others
    alignmentDistance: 180, // Radius at which boids align with others
    cohesionDistance: 300,  // Radius at which boids approach others
    separationForce: .15,  // Speed to avoid at
    alignmentForce: .25,   // Speed to align with other boids
    cohesionForce: .1,     // Speed to move towards other boids
    attractors: [
        [
            Infinity,
            Infinity,
            50000,
            .1
        ]
    ]
});

const ctx = flow.getContext('2d');

window.addEventListener('resize', onResize);
document.addEventListener('mousemove', onMousemove);

onResize();
run();

////////////////////////////////////////////////////////////////////////

function run() {
    ctx.clearRect(0, 0, flow.width, flow.height);
    ctx.fillStyle = 'white';

    ctx.save();
    ctx.translate(flow.width/2, flow.height/2);

    boids.tick();
    boids.boids.forEach((boid) => {
        let [x, y] = boid.position;
        let halves = [
            flow.width / 2,
            flow.height / 2
        ];

        boid.position[0] = x > halves[0] ? -halves[0] : -x > halves[0] ? halves[0] : x;
        boid.position[1] = y > halves[1] ? -halves[1] : -y > halves[1] ? halves[1] : y;

        drawCursor(boid);
    });

    ctx.restore();

    window.requestAnimationFrame(run);
}

function onResize() {
    flow.width = window.innerWidth;
    flow.height = window.innerHeight;
}

function onMousemove(e) {
    boids.attractors[0][0] = e.x - (flow.width / 2);
    boids.attractors[0][1] = e.y - (flow.height / 2);
}

function drawCursor(boid) {
    let position = boid.position;
    let direction = normalize(boid.speed);
    const cursor = [
        [-14, 6],
        [-12, 1],
        [-22, 1],
        [-22, -1],
        [-12, -1],
        [-14, -6],
        [0, 0]
    ];

    direction = Math.atan2(direction[0], direction[1]);

    ctx.strokeStyle = 'black';
    ctx.strokeWidth = '2px';

    ctx.fillStyle = 'white';

    ctx.save();
    ctx.beginPath();

    let cos = Math.cos(direction);
    let sin = Math.sin(direction);

    let coords = cursor.map(([x, y]) => ([x * sin - y * cos, x * cos + y * sin]));

    ctx.moveTo(
        position[0],
        position[1]
    );

    coords.forEach(([x, y]) => {
        ctx.lineTo(position[0] + x, position[1] + y);
    });

    ctx.fill();
    ctx.stroke();

    ctx.restore()
}

function normalize([x, y]) {
    let length = Math.sqrt(x * x + y * y);

    return [
        x / length,
        y / length
    ];
}
