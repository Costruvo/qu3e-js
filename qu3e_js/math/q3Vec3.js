//--------------------------------------------------------------------------------------------------
// q3Vec3.js (fixed + safety fallbacks)
//--------------------------------------------------------------------------------------------------

class q3Vec3 {

    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    Set(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    SetAll(a) {
        this.x = a;
        this.y = a;
        this.z = a;
    }

    clone()
    {
        return new q3Vec3(this.x, this.y, this.z);
    }

    // In-place arithmetic
    addEq(rhs) { this.x += rhs.x; this.y += rhs.y; this.z += rhs.z; return this; }
    subEq(rhs) { this.x -= rhs.x; this.y -= rhs.y; this.z -= rhs.z; return this; }
    mulEq(f)   { this.x *= f; this.y *= f; this.z *= f; return this; }
    divEq(f)   { this.x /= (f !== 0 ? f : 1); this.y /= (f !== 0 ? f : 1); this.z /= (f !== 0 ? f : 1); return this; }

    // Index access
    get(i) {
        if (i === 0) return this.x;
        if (i === 1) return this.y;
        if (i === 2) return this.z;
        throw new Error("q3Vec3 index out of range");
    }

    setIndex(i, value) {
        if (i === 0) this.x = value;
        else if (i === 1) this.y = value;
        else if (i === 2) this.z = value;
        else throw new Error("q3Vec3 index out of range");
    }

    // Unary -
    neg() { return new q3Vec3(-this.x, -this.y, -this.z); }

    // Arithmetic returning new vector
    add(rhs) { return new q3Vec3(this.x + rhs.x, this.y + rhs.y, this.z + rhs.z); }
    sub(rhs) { return new q3Vec3(this.x - rhs.x, this.y - rhs.y, this.z - rhs.z); }
    mul(f)   { return new q3Vec3(this.x * f, this.y * f, this.z * f); }
    div(f)   { let inv = f !== 0 ? 1.0 / f : 1.0; return new q3Vec3(this.x * inv, this.y * inv, this.z * inv); }

    // In-place normalize
    normalizeEq() {
        let len = q3Length(this);
        if (len > 0) {
            let inv = 1.0 / len;
            this.mulEq(inv);
        }
        return this;
    }

    
}

//--------------------------------------------------------------------------------------------------
// Scalar helpers (consolidated)
//--------------------------------------------------------------------------------------------------

function q3Abs(a) { return Math.abs(a); }
function q3Min(a, b) { return Math.min(a, b); }
function q3Max(a, b) { return Math.max(a, b); }
function q3Invert(a) { return a !== 0 ? 1.0 / a : 0.0; }
function q3Sign(a) { return a >= 0 ? 1 : -1; }
function q3Clamp(val, min, max) { return val < min ? min : (val > max ? max : val); }
function q3Clamp01(val) { return q3Clamp(val, 0.0, 1.0); }

//--------------------------------------------------------------------------------------------------
// q3Vec3 utility functions
//--------------------------------------------------------------------------------------------------

function q3Identity(obj)
{
    if (obj instanceof q3Vec3)
    {
        obj.SetAll(0);
    }
    else if (obj instanceof q3Mat3)
    {
        obj.ex.Set(1, 0, 0);
        obj.ey.Set(0, 1, 0);
        obj.ez.Set(0, 0, 1);
    }
}

function q3Dot(a, b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

function q3Cross(a, b) {
    return new q3Vec3(
        ((a.y*b.z) - (b.y*a.z)),
        ((b.x*a.z) - (a.x*b.z)),
        ((a.x*b.y) - (b.x*a.y))
    );
}

function q3MulScalarVec3(f, rhs) {
    return new q3Vec3(rhs.x * f, rhs.y * f, rhs.z * f);
}

function q3Length(v) {
    return Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

function q3LengthSq(v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

// Modification: avoids returning (1,0,0) blindly if the vector is nearly zero but has small components.
// This prevents NaNs in rotations or cross products.
function q3Normalize(v){
    return v.clone().normalizeEq();
}
/*
function q3Normalize(v)
{
    let len = Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

    if (len > 1e-9) // small epsilon instead of 0
        return new q3Vec3(v.x/len, v.y/len, v.z/len);

    // fallback: pick the axis with largest absolute value
    if (Math.abs(v.x) >= Math.abs(v.y) && Math.abs(v.x) >= Math.abs(v.z)) return new q3Vec3(1,0,0);
    if (Math.abs(v.y) >= Math.abs(v.z)) return new q3Vec3(0,1,0);
    return new q3Vec3(0,0,1);
}*/



function q3Distance(a, b) { return Math.sqrt(q3DistanceSq(a, b)); }
function q3DistanceSq(a, b) { let dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z; return dx*dx+dy*dy+dz*dz; }
function q3AbsVec3(v) { return new q3Vec3(q3Abs(v.x), q3Abs(v.y), q3Abs(v.z)); }
function q3MinVec3(a, b) { return new q3Vec3(q3Min(a.x,b.x), q3Min(a.y,b.y), q3Min(a.z,b.z)); }
function q3MaxVec3(a, b) { return new q3Vec3(q3Max(a.x,b.x), q3Max(a.y,b.y), q3Max(a.z,b.z)); }
function q3MinPerElem(a) { return q3Min(a.x, q3Min(a.y,a.z)); }
function q3MaxPerElem(a) { return q3Max(a.x, q3Max(a.y,a.z)); }

//--------------------------------------------------------------------------------------------------
// q3Add
//--------------------------------------------------------------------------------------------------

function q3Add(a, b)
{
    return new q3Vec3(
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    );
}

//--------------------------------------------------------------------------------------------------
// q3Sub
//--------------------------------------------------------------------------------------------------

function q3Sub(a, b)
{
    return new q3Vec3(
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    );
}

q3Vec3.prototype.scale = function(s)
{
    return new q3Vec3(
        this.x * s,
        this.y * s,
        this.z * s
    );
};

q3Vec3.prototype.scaleEq = function(s)
{
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
};

q3Vec3.prototype.mulScalar = function(s) {
    return new q3Vec3(this.x * s, this.y * s, this.z * s);
};
