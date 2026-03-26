//--------------------------------------------------------------------------------------------------
// q3Box.js (Safe & Optimized JS Port)
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Box triangle indices
//--------------------------------------------------------------------------------------------------
const kBoxIndices = [
    0,6,4, 0,2,6,
    0,3,2, 0,1,3,
    2,7,6, 2,3,7,
    4,6,7, 4,7,5,
    0,4,5, 0,5,1,
    1,5,7, 1,7,3
];

//--------------------------------------------------------------------------------------------------
// q3BoxDef.js
//--------------------------------------------------------------------------------------------------

class q3BoxDef {
    constructor() {
        this.m_tx = new q3Transform();
        this.m_e = new q3Vec3(0.5,0.5,0.5);
        
        this.m_tx.position = new q3Vec3(0,0,0);
        this.m_tx.rotation = q3IdentityMat3();

        this.m_friction = 0.04;
        this.m_restitution = 0.02;
        this.m_density = 1.0;
        this.m_sensor = false;
    }

    set(tx, extents) {
        this.m_tx = tx;
        this.m_e = extents.mul(0.5);
    }

    setFriction(friction) { this.m_friction = friction; }
    setRestitution(restitution) { this.m_restitution = restitution; }
    setDensity(density) { this.m_density = density; }
    setSensor(sensor) { this.m_sensor = sensor; }
}

class q3MassData {
    constructor() {
        this.inertia = new q3Mat3();
        this.center = new q3Vec3();
        this.mass = 0.0;
    }
}

class q3Box {
    constructor() {
        this.local = new q3Transform();
        this.e = new q3Vec3(0.5, 0.5, 0.5); // half extents by default

        this.next = null;
        this.body = null;
        this.friction = 0.4;
        this.restitution = 0.2;
        this.density = 1.0;
        this.broadPhaseIndex = -1;
        this.userData = null;
        this.sensor = false;
    }

    setUserData(data) {
        this.userData = data;
    }

    getUserData() {
        return this.userData;
    }

    setSensor(isSensor) {
        this.sensor = isSensor;
    }

    testPoint(tx, p) {
        const world = q3MulTransformTransform(tx, this.local);
        const p0 = q3MulT(world, p);

        return Math.abs(p0.x) <= this.e.x &&
               Math.abs(p0.y) <= this.e.y &&
               Math.abs(p0.z) <= this.e.z;
    }

    computeAABB(tx, aabb) {
        const world = q3MulTransformTransform(tx, this.local);

        const verts = [
            new q3Vec3(-this.e.x, -this.e.y, -this.e.z),
            new q3Vec3(-this.e.x, -this.e.y,  this.e.z),
            new q3Vec3(-this.e.x,  this.e.y, -this.e.z),
            new q3Vec3(-this.e.x,  this.e.y,  this.e.z),
            new q3Vec3( this.e.x, -this.e.y, -this.e.z),
            new q3Vec3( this.e.x, -this.e.y,  this.e.z),
            new q3Vec3( this.e.x,  this.e.y, -this.e.z),
            new q3Vec3( this.e.x,  this.e.y,  this.e.z)
        ];

        for (let i = 0; i < 8; ++i)
            verts[i] = q3Mul(world, verts[i]);

        let min = new q3Vec3(Q3_R32_MAX, Q3_R32_MAX, Q3_R32_MAX);
        let max = new q3Vec3(-Q3_R32_MAX, -Q3_R32_MAX, -Q3_R32_MAX);

        for (let i = 0; i < 8; ++i) {
            min = q3MinVec3(min, verts[i]);
            max = q3MaxVec3(max, verts[i]);
        }

        aabb.min = min;
        aabb.max = max;
    }

    computeMass(md) {
        const ex2 = 4.0 * this.e.x * this.e.x;
        const ey2 = 4.0 * this.e.y * this.e.y;
        const ez2 = 4.0 * this.e.z * this.e.z;

        const mass = 8.0 * this.e.x * this.e.y * this.e.z * this.density;

        const x = (1/12) * mass * (ey2 + ez2);
        const y = (1/12) * mass * (ex2 + ez2);
        const z = (1/12) * mass * (ex2 + ey2);

        let I = q3Diagonal(x, y, z);
        I = q3MulMat3(this.local.rotation, I, q3Transpose(this.local.rotation));

        const identity = q3IdentityMat3();
        const outer = q3OuterProduct(this.local.position, this.local.position);
        I = q3AddMat3(I, q3SubMat3(q3MulScalar(identity, mass), outer));

        md.center = this.local.position.clone();
        md.inertia = I;
        md.mass = mass;
    }

    render(tx, awake, render) {
        const world = q3MulTransformTransform(tx, this.local);
        const verts = [
            new q3Vec3(-this.e.x, -this.e.y, -this.e.z),
            new q3Vec3(-this.e.x, -this.e.y,  this.e.z),
            new q3Vec3(-this.e.x,  this.e.y, -this.e.z),
            new q3Vec3(-this.e.x,  this.e.y,  this.e.z),
            new q3Vec3( this.e.x, -this.e.y, -this.e.z),
            new q3Vec3( this.e.x, -this.e.y,  this.e.z),
            new q3Vec3( this.e.x,  this.e.y, -this.e.z),
            new q3Vec3( this.e.x,  this.e.y,  this.e.z)
        ];

        for (let i = 0; i < 36; i += 3) {
            const a = q3Mul(world, verts[kBoxIndices[i]]);
            const b = q3Mul(world, verts[kBoxIndices[i+1]]);
            const c = q3Mul(world, verts[kBoxIndices[i+2]]);

            const n = q3Normalize(q3Cross(b.sub(a), c.sub(a)));

            render.setTriNormal(n.x, n.y, n.z);
            render.triangle(
                a.x,a.y,a.z,
                b.x,b.y,b.z,
                c.x,c.y,c.z
            );
        }
    }
}



//--------------------------------------------------------------------------------------------------
// q3Box inline helpers (from q3Box.inl)
//--------------------------------------------------------------------------------------------------

q3Box.prototype.setUserData = function(data) {
    this.userData = data;
};

q3Box.prototype.getUserData = function() {
    return this.userData;
};

//--------------------------------------------------------------------------------------------------
q3BoxDef.prototype.set = function(tx, extents) {
    this.m_tx = tx;
    this.m_e = extents.mul(0.5);
};

q3BoxDef.prototype.setRestitution = function(restitution) {
    this.m_restitution = restitution;
};

q3BoxDef.prototype.setFriction = function(friction) {
    this.m_friction = friction;
};

q3BoxDef.prototype.setDensity = function(density) {
    this.m_density = density;
};

q3BoxDef.prototype.setSensor = function(sensor) {
    this.m_sensor = sensor;
};



q3Box.prototype.raycast = function(tx, raycast) {
    // Combine the box local transform with the given transform
    const world = q3MulTransformTransform(tx, this.local);

    // Transform ray direction and start into the box's local space
    const d = q3MulTTransformVec3({ position: new q3Vec3(0,0,0), rotation: world.rotation }, raycast.dir);
    
    const p = q3MulTTransformVec3(world, raycast.start);

    const epsilon = 1e-8;
    let tmin = 0.0;
    let tmax = raycast.t;
    let n0 = new q3Vec3();

    for (const axis of ['x','y','z']) {
        if (Math.abs(d[axis]) < epsilon) {
            if (p[axis] < -this.e[axis] || p[axis] > this.e[axis])
                return false;
        } else {
            const invD = 1.0 / d[axis];
            const s = q3Sign(d[axis]);
            const ei = this.e[axis] * s;
            const n = new q3Vec3();
            n[axis] = -s;

            const t0 = -(ei + p[axis]) * invD;
            const t1 = (ei - p[axis]) * invD;

            if (t0 > tmin) {
                tmin = t0;
                n0 = n;
            }

            tmax = Math.min(tmax, t1);
            if (tmin > tmax) return false;
        }
    }

    // Transform normal back into world space
    raycast.normal = q3MulMat3Vec3(world.rotation, n0);
    raycast.toi = tmin;
    return true;
};


