//--------------------------------------------------------------------------------------------------
// q3Transform.js
// JavaScript port of Randy Gaul's q3Transform
//--------------------------------------------------------------------------------------------------

class q3Transform
{
    constructor(position = new q3Vec3(0, 0, 0), rotation = q3IdentityMat3())
    {
        this.position = position;
        this.rotation = rotation;
    }

    set(position, rotation)
    {
        this.position = position;
        this.rotation = rotation;
    }

    setIdentity()
    {
        this.position.SetAll(0);
        this.rotation = q3IdentityMat3();
    }
  
    clone()
    {
        return new q3Transform(this.position.clone(), this.rotation.clone());
    }
}

//--------------------------------------------------------------------------------------------------
// Identity helper
//--------------------------------------------------------------------------------------------------
function q3IdentityTransform(tx)
{
    if (tx == null){
        return new q3Transform();
    }

    tx.setIdentity();
}

//--------------------------------------------------------------------------------------------------
// Transform * Vec3 *** same thing...
//--------------------------------------------------------------------------------------------------
function q3MulTransformVec3(tx, v)
{
    const r = q3MulMat3Vec3(tx.rotation, v);
    return r.add(tx.position);
}

//--------------------------------------------------------------------------------------------------
// Transform * Vec3
//--------------------------------------------------------------------------------------------------
function q3MulTransform(tx, v)
{
    const r = q3MulMat3Vec3(tx.rotation, v);
    return r.add(tx.position);
}

//--------------------------------------------------------------------------------------------------
// Transform * Transform
//--------------------------------------------------------------------------------------------------
function q3MulTransformTransform(a, b)
{
    const rotation = q3MulMat3(a.rotation, b.rotation);
    const position = q3MulMat3Vec3(a.rotation, b.position).add(a.position);
    return new q3Transform(position, rotation);
}

//--------------------------------------------------------------------------------------------------
// Inverse Transform * Vec3
//--------------------------------------------------------------------------------------------------
function q3MulT(tx, v)
{
    if (tx instanceof q3Transform)
    {
        const local = v.sub(tx.position);
        const rotT = q3Transpose(tx.rotation);
        return q3MulMat3Vec3(rotT, local);
    }
    else if (tx instanceof q3Mat3)
    {
        let M = tx;
        return new q3Vec3(
            q3Dot(M.ex, v),
            q3Dot(M.ey, v),
            q3Dot(M.ez, v)
        );
    }
}


//--------------------------------------------------------------------------------------------------
// Inverse Transform * Transform
//--------------------------------------------------------------------------------------------------
function q3MulTTransform(a, b)
{
    const rotT = q3Transpose(a.rotation);
    const rotation = q3MulMat3(rotT, b.rotation);
    const position = q3MulMat3Vec3(rotT, b.position.sub(a.position));
    return new q3Transform(position, rotation);
}





//--------------------------------------------------------------------------------------------------
// Inverse Transform * Vec3
//--------------------------------------------------------------------------------------------------

function q3MulTTransformVec3(tx, v)
{
    const local = v.sub(tx.position);

    // Avoid NaNs if rotation matrix is malformed
    const rotT = q3Transpose(tx.rotation) || q3IdentityMat3();

    return q3MulMat3Vec3(rotT, local);
}

// Modifications: added safety fallback
function q3Inverse(m)
{
    const tmp0 = q3Cross(m.ey, m.ez);
    const tmp1 = q3Cross(m.ez, m.ex);
    const tmp2 = q3Cross(m.ex, m.ey);

    const det = q3Dot(m.ez, tmp2);

    if (Math.abs(det) < 1e-9)
    {
        let fallback = new q3Mat3();
        fallback.SetIdentity();
        return fallback;
    }

    const invDet = 1 / det;

    return new q3Mat3(
        tmp0.x * invDet, tmp1.x * invDet, tmp2.x * invDet,
        tmp0.y * invDet, tmp1.y * invDet, tmp2.y * invDet,
        tmp0.z * invDet, tmp1.z * invDet, tmp2.z * invDet
    );
}