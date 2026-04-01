//--------------------------------------------------------------------------------------------------
// q3Geometry.js (Merged, 1:1 JS port)
//--------------------------------------------------------------------------------------------------

// Vector math helpers
function q3Dot(a, b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

function q3Cross(a, b) {
    return new q3Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

function q3LengthSqVec3(v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

function q3LengthVec3(v) {
    return Math.sqrt(q3LengthSqVec3(v));
}

function q3DistanceSqVec3(a,b){
    return q3LengthSqVec3(a.sub(b));
}

function q3DistanceVec3(a,b){
    return Math.sqrt(q3DistanceSqVec3(a,b));
}

function q3Clamp(val, min, max) {
    return val < min ? min : (val > max ? max : val);
}

function q3Clamp01(val) {
    return q3Clamp(val, 0.0, 1.0);
}

function q3Abs(a) {
    return Math.abs(a);
}

function q3Min(a,b) {
    return Math.min(a,b);
}

function q3Max(a,b) {
    return Math.max(a,b);
}

// Vector min/max helpers
function q3MinVec3(a, b) {
    return new q3Vec3(
        Math.min(a.x, b.x),
        Math.min(a.y, b.y),
        Math.min(a.z, b.z)
    );
}

function q3MaxVec3(a, b) {
    return new q3Vec3(
        Math.max(a.x, b.x),
        Math.max(a.y, b.y),
        Math.max(a.z, b.z)
    );
}

//--------------------------------------------------------------------------------------------------
// q3AABB class
//--------------------------------------------------------------------------------------------------
class q3AABB {
    constructor(min = new q3Vec3(), max = new q3Vec3()) {
        this.min = min;
        this.max = max;
    }

    ContainsAABB(other) {
        return (
            this.min.x <= other.min.x &&
            this.min.y <= other.min.y &&
            this.min.z <= other.min.z &&
            this.max.x >= other.max.x &&
            this.max.y >= other.max.y &&
            this.max.z >= other.max.z
        );
    }

    ContainsPoint(point) {
        return (
            this.min.x <= point.x &&
            this.min.y <= point.y &&
            this.min.z <= point.z &&
            this.max.x >= point.x &&
            this.max.y >= point.y &&
            this.max.z >= point.z
        );
    }

    SurfaceArea() {
        const x = this.max.x - this.min.x;
        const y = this.max.y - this.min.y;
        const z = this.max.z - this.min.z;
        return 2.0 * (x*y + x*z + y*z);
    }

    expandToInclude(other) {
        this.min = q3MinVec3(this.min, other.min);
        this.max = q3MaxVec3(this.max, other.max);
    }

    clone(){
        return new q3AABB(this.min.clone(), this.max.clone());
    }
}

function q3AABBtoAABB(a, b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

function q3Combine(a, b) {
    let out = new q3AABB();
    out.min.x = Math.min(a.min.x, b.min.x);
    out.min.y = Math.min(a.min.y, b.min.y);
    out.min.z = Math.min(a.min.z, b.min.z);

    out.max.x = Math.max(a.max.x, b.max.x);
    out.max.y = Math.max(a.max.y, b.max.y);
    out.max.z = Math.max(a.max.z, b.max.z);

    return out;
}
/*
function q3Combine(a, b) {
    return new q3AABB(q3MinVec3(a.min, b.min), q3MaxVec3(a.max, b.max));
}
*/

function q3ClosestPointOnAABB(point, aabbMin, aabbMax) {
    return new q3Vec3(
        Math.max(aabbMin.x, Math.min(aabbMax.x, point.x)),
        Math.max(aabbMin.y, Math.min(aabbMax.y, point.y)),
        Math.max(aabbMin.z, Math.min(aabbMax.z, point.z))
    );
}

//--------------------------------------------------------------------------------------------------
// q3Closest point on line
//--------------------------------------------------------------------------------------------------
function q3ClosestPointLineSegment(a, b, p) {
    const ab = b.clone().sub(a);
    const lenSq = q3LengthSqVec3(ab);
    let t = 0.0;
    if(lenSq>0) t = q3Dot(ab, p.clone().sub(a)) / lenSq;
    t = q3Clamp01(t);
    return a.clone().add(ab.clone().mul(t));
}

//--------------------------------------------------------------------------------------------------
// q3RaycastData
//--------------------------------------------------------------------------------------------------
class q3RaycastData {
    constructor(start = new q3Vec3(), dir = new q3Vec3(), t = 0.0) {
        this.start = start;
        this.dir = dir;
        this.t = t;
        this.toi = 0.0;
        this.normal = new q3Vec3();
    }

    Set(startPoint, direction, endPointTime) {
        this.start = startPoint;
        this.dir = direction;
        this.t = endPointTime;
        this.toi = 0.0;
        this.normal = new q3Vec3();
    }

    GetImpactPoint() {
        return this.start.add(this.dir.mul(this.toi));
    }
}

//--------------------------------------------------------------------------------------------------
// Ray vs AABB test
//--------------------------------------------------------------------------------------------------
function q3RayAABB(rayOrigin, rayDir, aabbMin, aabbMax) {
    let tmin = -Infinity, tmax = Infinity;

    for (const axis of ['x','y','z']) {
        if(rayDir[axis] !== 0.0) {
            let t1 = (aabbMin[axis]-rayOrigin[axis])/rayDir[axis];
            let t2 = (aabbMax[axis]-rayOrigin[axis])/rayDir[axis];
            if(t1>t2) [t1,t2] = [t2,t1];
            tmin = Math.max(tmin,t1);
            tmax = Math.min(tmax,t2);
            if(tmin>tmax) return [false,0,0];
        } else if(rayOrigin[axis]<aabbMin[axis]||rayOrigin[axis]>aabbMax[axis]) return [false,0,0];
    }
    return [true,tmin,tmax];
}


// a = normal, b/c = tangentVectors[0/1]
function q3ComputeBasis(a, tangentVectors) {
    const k = 0.57735027;
    let b = tangentVectors[0];
    let c = tangentVectors[1];

    if (Math.abs(a.x) >= k) {
        b.Set(a.y, -a.x, 0.0);
    } else {
        b.Set(0.0, a.z, -a.y);
    }

    //a.normalizeEq();
    //b.normalizeEq(); // modify b -- does not clone()

    const cross = q3Cross(a, b);
    c.Set(cross.x, cross.y, cross.z);
    //c.normalizeEq();
}
