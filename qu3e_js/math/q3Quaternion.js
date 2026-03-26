//--------------------------------------------------------------------------------------------------
// q3Quaternion.js
// JavaScript port of Randy Gaul's q3Quaternion
//--------------------------------------------------------------------------------------------------

class q3Quaternion
{
    constructor(x = 0, y = 0, z = 0, w = 1)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    Set(x, y, z, w)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
  
    SetFromAxisAngle(axis, angle)
    {
        const half = angle * 0.5;
        const s = Math.sin(half);
    
        this.x = axis.x * s;
        this.y = axis.y * s;
        this.z = axis.z * s;
        this.w = Math.cos(half);
    }

    SetIdentity()
    {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 1;
    }

    // +=
    addEq(q)
    {
        this.x += q.x;
        this.y += q.y;
        this.z += q.z;
        this.w += q.w;
        return this;
    }

    // -=
    subEq(q)
    {
        this.x -= q.x;
        this.y -= q.y;
        this.z -= q.z;
        this.w -= q.w;
        return this;
    }

    // *= scalar
    mulEqScalar(s)
    {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        this.w *= s;
        return this;
    }

    // /= scalar
    divEqScalar(s)
    {
        this.x /= s;
        this.y /= s;
        this.z /= s;
        this.w /= s;
        return this;
    }

    // +
    add(q)
    {
        return new q3Quaternion(
            this.x + q.x,
            this.y + q.y,
            this.z + q.z,
            this.w + q.w
        );
    }

    // -
    sub(q)
    {
        return new q3Quaternion(
            this.x - q.x,
            this.y - q.y,
            this.z - q.z,
            this.w - q.w
        );
    }

    // * scalar
    mulScalar(s)
    {
        return new q3Quaternion(
            this.x * s,
            this.y * s,
            this.z * s,
            this.w * s
        );
    }

    // / scalar
    divScalar(s)
    {
        return new q3Quaternion(
            this.x / s,
            this.y / s,
            this.z / s,
            this.w / s
        );
    }
  
    ToMat3 = function()
    {
        let qx2 = this.x + this.x;
        let qy2 = this.y + this.y;
        let qz2 = this.z + this.z;
        
        let qxqx2 = this.x * qx2;
        let qxqy2 = this.x * qy2;
        let qxqz2 = this.x * qz2;
        let qxqw2 = this.w * qx2;
        
        let qyqy2 = this.y * qy2;
        let qyqz2 = this.y * qz2;
        let qyqw2 = this.w * qy2;
        
        let qzqz2 = this.z * qz2;
        let qzqw2 = this.w * qz2;
        
        return new q3Mat3(
            // Column 0
            1.0 - qyqy2 - qzqz2,
            qxqy2 + qzqw2,
            qxqz2 - qyqw2,
            
            // Column 1
            qxqy2 - qzqw2,
            1.0 - qxqx2 - qzqz2,
            qyqz2 + qxqw2,
            
            // Column 2
            qxqz2 + qyqw2,
            qyqz2 - qxqw2,
            1.0 - qxqx2 - qyqy2
        );
    };
}

q3Quaternion.prototype.integrate = function(dv, dt)
{
    // q = (dv * dt, 0)
    const q = new q3Quaternion(
        dv.x * dt,
        dv.y * dt,
        dv.z * dt,
        0.0
    );

    // q *= *this  (quat multiply: q = q * this)
    const x = q.w * this.x + q.x * this.w + q.y * this.z - q.z * this.y;
    const y = q.w * this.y - q.x * this.z + q.y * this.w + q.z * this.x;
    const z = q.w * this.z + q.x * this.y - q.y * this.x + q.z * this.w;
    const w = q.w * this.w - q.x * this.x - q.y * this.y - q.z * this.z;

    // integrate
    this.x += x * 0.5;
    this.y += y * 0.5;
    this.z += z * 0.5;
    this.w += w * 0.5;

    // normalize (critical!)
    const len = Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z + this.w*this.w);
    if (len > 0) {
        const inv = 1 / len;
        this.x *= inv;
        this.y *= inv;
        this.z *= inv;
        this.w *= inv;
    }
};

function q3NormalizeQuat(q)
{
    const len = Math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    if (len > 1e-9)
    {
        const inv = 1 / len;
        return new q3Quaternion(
            q.x * inv,
            q.y * inv,
            q.z * inv,
            q.w * inv
        );
    }

    // fallback = identity quaternion (NOT axis!)
    return new q3Quaternion(0, 0, 0, 1);
}

//--------------------------------------------------------------------------------------------------

function q3DotQuaternion(a, b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

//--------------------------------------------------------------------------------------------------

function q3NormalizeQuaternion(q)
{
    let len = Math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (len > 1e-9)
    {
        let inv = 1.0 / len;
        return new q3Quaternion(q.x*inv, q.y*inv, q.z*inv, q.w*inv);
    }

    // fallback to identity quaternion
    return new q3Quaternion(0,0,0,1);
}

//--------------------------------------------------------------------------------------------------

function q3MulQuaternion(a, b)
{
    return new q3Quaternion(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}

//--------------------------------------------------------------------------------------------------

function q3Conjugate(q)
{
    return new q3Quaternion(
        -q.x,
        -q.y,
        -q.z,
        q.w
    );
}
