//--------------------------------------------------------------------------------------------------
// q3Mat3.js
// JavaScript port of Randy Gaul's q3Mat3
//--------------------------------------------------------------------------------------------------

class q3Mat3 {

    constructor(
        m00 = 0, m01 = 0, m02 = 0,
        m10 = 0, m11 = 0, m12 = 0,
        m20 = 0, m21 = 0, m22 = 0
    )
    {
        this.ex = new q3Vec3(m00, m01, m02);
        this.ey = new q3Vec3(m10, m11, m12);
        this.ez = new q3Vec3(m20, m21, m22);
    }

    Set(
        m00, m01, m02,
        m10, m11, m12,
        m20, m21, m22
    )
    {
        this.ex.Set(m00, m01, m02);
        this.ey.Set(m10, m11, m12);
        this.ez.Set(m20, m21, m22);
    }

    SetIdentity()
    {
        this.ex.Set(1,0,0);
        this.ey.Set(0,1,0);
        this.ez.Set(0,0,1);
    }

    SetZero()
    {
        this.ex.SetAll(0);
        this.ey.SetAll(0);
        this.ez.SetAll(0);
    }
  
    SetRows(r0, r1, r2) {
        this.ex = r0.clone(); // column 0
        this.ey = r1.clone(); // column 1
        this.ez = r2.clone(); // column 2
    }
  
    Column0()
    {
        return new q3Vec3(this.ex.x, this.ey.x, this.ez.x);
    }
  
    Column1()
    {
        return new q3Vec3(this.ex.y, this.ey.y, this.ez.y);
    }
  
    Column2()
    {
        return new q3Vec3(this.ex.z, this.ey.z, this.ez.z);
    }

    // matrix addition
    add(rhs)
    {
        return new q3Mat3(
            this.ex.x + rhs.ex.x, this.ex.y + rhs.ex.y, this.ex.z + rhs.ex.z,
            this.ey.x + rhs.ey.x, this.ey.y + rhs.ey.y, this.ey.z + rhs.ey.z,
            this.ez.x + rhs.ez.x, this.ez.y + rhs.ez.y, this.ez.z + rhs.ez.z
        );
    }

    // matrix subtraction
    sub(rhs)
    {
        return new q3Mat3(
            this.ex.x - rhs.ex.x, this.ex.y - rhs.ex.y, this.ex.z - rhs.ex.z,
            this.ey.x - rhs.ey.x, this.ey.y - rhs.ey.y, this.ey.z - rhs.ey.z,
            this.ez.x - rhs.ez.x, this.ez.y - rhs.ez.y, this.ez.z - rhs.ez.z
        );
    }

    // scalar multiply
    mulScalar(s)
    {
        return new q3Mat3(
            this.ex.x*s, this.ex.y*s, this.ex.z*s,
            this.ey.x*s, this.ey.y*s, this.ey.z*s,
            this.ez.x*s, this.ez.y*s, this.ez.z*s
        );
    }

    clone()
    {
        return new q3Mat3(this.ex.x, this.ex.y, this.ex.z,
                          this.ey.x, this.ey.y, this.ey.z,
                          this.ez.x, this.ez.y, this.ez.z);
    }
}

//--------------------------------------------------------------------------------------------------
// Matrix operations
//--------------------------------------------------------------------------------------------------
// This is correct
function q3MulMat3Vec3(m, rhs)
{
    return new q3Vec3(
        m.ex.x * rhs.x + m.ey.x * rhs.y + m.ez.x * rhs.z,
        m.ex.y * rhs.x + m.ey.y * rhs.y + m.ez.y * rhs.z,
        m.ex.z * rhs.x + m.ey.z * rhs.y + m.ez.z * rhs.z
    );
}

q3Mat3.prototype.mulVec3 = function(rhs) {
    return q3MulMat3Vec3(this, rhs);
};

function q3MulMat3(A, B, C = null) {
    // If a third argument is provided, assume it's a "post-multiply" or transform
    if (C) {
        // Do A * B * C
        const AB = q3MulMat3(A, B);  // reuse the same function for first two
        return q3MulMat3(AB, C);     // multiply result by C
    }

    // Default 2-matrix multiply (column-major)
    const col0 = q3MulMat3Vec3(A, B.ex);
    const col1 = q3MulMat3Vec3(A, B.ey);
    const col2 = q3MulMat3Vec3(A, B.ez);

    let m = new q3Mat3();
    m.SetRows(col0, col1, col2); // or SetColumns if your Mat3 uses column vectors
    return m;
}

/*function q3MulMat3(A, B)
{
    return new q3Mat3(
        // Column 0
        A.ex.x * B.ex.x + A.ey.x * B.ex.y + A.ez.x * B.ex.z,
        A.ex.y * B.ex.x + A.ey.y * B.ex.y + A.ez.y * B.ex.z,
        A.ex.z * B.ex.x + A.ey.z * B.ex.y + A.ez.z * B.ex.z,

        // Column 1
        A.ex.x * B.ey.x + A.ey.x * B.ey.y + A.ez.x * B.ey.z,
        A.ex.y * B.ey.x + A.ey.y * B.ey.y + A.ez.y * B.ey.z,
        A.ex.z * B.ey.x + A.ey.z * B.ey.y + A.ez.z * B.ey.z,

        // Column 2
        A.ex.x * B.ez.x + A.ey.x * B.ez.y + A.ez.x * B.ez.z,
        A.ex.y * B.ez.x + A.ey.y * B.ez.y + A.ez.y * B.ez.z,
        A.ex.z * B.ez.x + A.ey.z * B.ez.y + A.ez.z * B.ez.z
    );
}*/


//--------------------------------------------------------------------------------------------------
// This is correct (qu3e 1:1)
function q3Transpose(m)
{
    return new q3Mat3(
        m.ex.x, m.ey.x, m.ez.x,
        m.ex.y, m.ey.y, m.ez.y,
        m.ex.z, m.ey.z, m.ez.z
    );
}

//--------------------------------------------------------------------------------------------------

function q3AbsMat3(m)
{
    return new q3Mat3(
        q3Abs(m.ex.x), q3Abs(m.ex.y), q3Abs(m.ex.z),
        q3Abs(m.ey.x), q3Abs(m.ey.y), q3Abs(m.ey.z),
        q3Abs(m.ez.x), q3Abs(m.ez.y), q3Abs(m.ez.z)
    );
}

//--------------------------------------------------------------------------------------------------

function q3Diagonal(x=0,y=x,z=x)
{
    let m = new q3Mat3();
    m.SetZero();

    m.ex.x = x;
    m.ey.y = y;
    m.ez.z = z;

    return m;
}


//--------------------------------------------------------------------------------------------------
// q3OuterProduct
//--------------------------------------------------------------------------------------------------

function q3OuterProduct(u, v)
{
    // v * u.x, v * u.y, v * u.z
    const a = v.scale(u.x);
    const b = v.scale(u.y);
    const c = v.scale(u.z);

    return new q3Mat3(
        a.x, a.y, a.z,
        b.x, b.y, b.z,
        c.x, c.y, c.z
    );
}

//--------------------------------------------------------------------------------------------------
// q3AddMat3
//--------------------------------------------------------------------------------------------------

function q3AddMat3(a, b)
{
    return new q3Mat3(
        a.ex.x + b.ex.x, a.ex.y + b.ex.y, a.ex.z + b.ex.z,
        a.ey.x + b.ey.x, a.ey.y + b.ey.y, a.ey.z + b.ey.z,
        a.ez.x + b.ez.x, a.ez.y + b.ez.y, a.ez.z + b.ez.z
    );
}


//--------------------------------------------------------------------------------------------------
// q3SubMat3
//--------------------------------------------------------------------------------------------------

function q3SubMat3(a, b)
{
    return new q3Mat3(
        a.ex.x - b.ex.x, a.ex.y - b.ex.y, a.ex.z - b.ex.z,
        a.ey.x - b.ey.x, a.ey.y - b.ey.y, a.ey.z - b.ey.z,
        a.ez.x - b.ez.x, a.ez.y - b.ez.y, a.ez.z - b.ez.z
    );
}

//--------------------------------------------------------------------------------------------------
// q3MulScalar
//--------------------------------------------------------------------------------------------------

function q3MulScalar(m, s)
{
    return new q3Mat3(
        m.ex.x * s, m.ex.y * s, m.ex.z * s,
        m.ey.x * s, m.ey.y * s, m.ey.z * s,
        m.ez.x * s, m.ez.y * s, m.ez.z * s
    );
}


function q3IdentityMat3(){
    let m = new q3Mat3();
    m.SetIdentity();
    return m;
}


// Add missing SetIdentity to q3Mat3 prototype for convenience
q3Mat3.prototype.SetIdentity = function() {
    this.ex.Set(1,0,0);
    this.ey.Set(0,1,0);
    this.ez.Set(0,0,1);
};

//--------------------------------------------------------------------------------------------------
// q3Mat3 helpers: getRow / getCol
//--------------------------------------------------------------------------------------------------

q3Mat3.prototype.getRow = function(i)
{
    return new q3Vec3(
        this.ex.get(i),
        this.ey.get(i),
        this.ez.get(i)
    );
};

q3Mat3.prototype.setRow = function(i, rowVec) {
    this.ex.setIndex(i, rowVec.x);
    this.ey.setIndex(i, rowVec.y);
    this.ez.setIndex(i, rowVec.z);
};

q3Mat3.prototype.getCol = function(i)
{
    if (i === 0) return new q3Vec3(this.ex.x, this.ey.x, this.ez.x);
    if (i === 1) return new q3Vec3(this.ex.y, this.ey.y, this.ez.y);
    if (i === 2) return new q3Vec3(this.ex.z, this.ey.z, this.ez.z);
    throw new Error("bad column");
};


q3Mat3.prototype.set = function(row, col, value)
{
    if (col === 0) this.ex.setIndex(row, value);
    else if (col === 1) this.ey.setIndex(row, value);
    else if (col === 2) this.ez.setIndex(row, value);
    else throw new Error("q3Mat3 column index out of range");
};


q3Mat3.prototype.get = function(row, col)
{
    if (col === 0) return this.ex.get(row);
    if (col === 1) return this.ey.get(row);
    if (col === 2) return this.ez.get(row);

    throw new Error("q3Mat3.get column out of range");
};

function q3Mat3RotationX(angle)
{
    let c = Math.cos(angle);
    let s = Math.sin(angle);

    let m = new q3Mat3();

    m.ex.Set(1, 0, 0);
    m.ey.Set(0, c, s);
    m.ez.Set(0, -s, c);

    return m;
}

function q3Mat3RotationY(angle)
{
    let c = Math.cos(angle);
    let s = Math.sin(angle);

    let m = new q3Mat3();

    m.ex.Set(c, 0, -s);
    m.ey.Set(0, 1, 0);
    m.ez.Set(s, 0, c);

    return m;
}

function q3Mat3RotationZ(angle)
{
    let c = Math.cos(angle);
    let s = Math.sin(angle);

    let m = new q3Mat3();

    m.ex.Set(c, s, 0);
    m.ey.Set(-s, c, 0);
    m.ez.Set(0, 0, 1);

    return m;
}


function q3Mul(a, b)
{
    // Transform * Vec3
    if (a instanceof q3Transform && b instanceof q3Vec3)
    {
        let r = q3MulMat3Vec3(a.rotation, b);
        return new q3Vec3(
            r.x + a.position.x,
            r.y + a.position.y,
            r.z + a.position.z
        );
    }

    // Mat3 * Vec3
    if (a instanceof q3Mat3 && b instanceof q3Vec3)
    {
        return q3MulMat3Vec3(a, b);
    }

    // Mat3 * Mat3
    if (a instanceof q3Mat3 && b instanceof q3Mat3)
    {
        return q3MulMat3(a, b);
    }

    throw new Error("q3Mul: unsupported types");
}
