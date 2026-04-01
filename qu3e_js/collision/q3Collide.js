//--------------------------------------------------------------------------------------------------
// q3Collide.js
// Direct JS port of qu3e q3Collide.cpp
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Macros
//--------------------------------------------------------------------------------------------------

function InFront(a){ return a < 0 }
function Behind(a){ return a >= 0 }
function On(a){ return a < 0.005 && a > -0.005 }

function q3TrackFaceAxis(axis, n, s, sMax, normal, axisNormal)
{
    if (s > 0) return true;

    if (s > sMax.value)
    {
        sMax.value = s;
        axis.value = n;
        axisNormal.Set(normal.x, normal.y, normal.z);
    }

    return false;
}

function q3TrackEdgeAxis(axis, n, s, sMax, normal, axisNormal)
{
    if (s > 0) return true;

    let len = q3Length(normal);
    //if (len === 0) return false;

    let l = 1.0 / len;
    s *= l;

    if (s > sMax.value)
    {
        sMax.value = s;
        axis.value = n;
        axisNormal.Set(normal.x * l, normal.y * l, normal.z * l);
    }

    return false;
}

//--------------------------------------------------------------------------------------------------
// q3ClipVertex
//--------------------------------------------------------------------------------------------------

class q3ClipVertex
{
    constructor()
    {
        this.v = new q3Vec3();
        
        this.f = {
            inR: 0,
            outR: 0,
            inI: 0,
            outI: 0,
            key: 0
        };
    }

    clone() {
        const cv = new q3ClipVertex();
        cv.v = this.v.clone();
        cv.f.inR = this.f.inR;
        cv.f.outR = this.f.outR;
        cv.f.inI = this.f.inI;
        cv.f.outI = this.f.outI;
        cv.f.key = this.f.key;
        return cv;
    }
}


//--------------------------------------------------------------------------------------------------
// q3ComputeReferenceEdgesAndBasis
//--------------------------------------------------------------------------------------------------
function q3ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, clipEdges, basis, e)
{
    let norm = q3MulMat3Vec3(q3Transpose(rtx.rotation), n);

    if (axis >= 3) axis -= 3;

    let r = rtx.rotation;

    switch (axis)
    {
        case 0:
            if (norm.x > 0)
            {
                clipEdges[0] = 1;
                clipEdges[1] = 8;
                clipEdges[2] = 7;
                clipEdges[3] = 9;

                e.Set(eR.y, eR.z, eR.x);
                basis.SetRows(r.ey, r.ez, r.ex);
            }
            else
            {
                clipEdges[0] = 11;
                clipEdges[1] = 3;
                clipEdges[2] = 10;
                clipEdges[3] = 5;

                e.Set(eR.z, eR.y, eR.x);
                basis.SetRows(r.ez, r.ey, r.ex.neg());
            }
            break;

        case 1:
            if (norm.y > 0)
            {
                clipEdges[0] = 0;
                clipEdges[1] = 1;
                clipEdges[2] = 2;
                clipEdges[3] = 3;

                e.Set(eR.z, eR.x, eR.y);
                basis.SetRows(r.ez, r.ex, r.ey);
            }
            else
            {
                clipEdges[0] = 4;
                clipEdges[1] = 5;
                clipEdges[2] = 6;
                clipEdges[3] = 7;

                e.Set(eR.z, eR.x, eR.y);
                basis.SetRows(r.ez, r.ex.neg(), r.ey.neg());
            }
            break;

        case 2:
            if (norm.z > 0)
            {
                clipEdges[0] = 11;
                clipEdges[1] = 4;
                clipEdges[2] = 8;
                clipEdges[3] = 0;

                e.Set(eR.y, eR.x, eR.z);
                basis.SetRows(r.ey.neg(), r.ex, r.ez);
            }
            else
            {
                clipEdges[0] = 6;
                clipEdges[1] = 10;
                clipEdges[2] = 2;
                clipEdges[3] = 9;

                e.Set(eR.y, eR.x, eR.z);
                basis.SetRows(r.ey.neg(), r.ex.neg(), r.ez.neg());
            }
            break;
    }
}

//--------------------------------------------------------------------------------------------------
// q3ComputeIncidentFace
//--------------------------------------------------------------------------------------------------

function q3ComputeIncidentFace(out, e, tx, n)
{
    // EXACT: n = -q3MulT(itx.rotation, n)
    let norm = q3MulMat3Vec3(q3Transpose(tx.rotation), n).neg();

    let absN = q3AbsVec3(norm);

    if (absN.x > absN.y && absN.x > absN.z)
    {
        if (norm.x > 0)
        {
            out[0].v.Set( e.x,  e.y, -e.z );
            out[1].v.Set( e.x,  e.y,  e.z );
            out[2].v.Set( e.x, -e.y,  e.z );
            out[3].v.Set( e.x, -e.y, -e.z );

            out[0].f.inI = 9;  out[0].f.outI = 1;
            out[1].f.inI = 1;  out[1].f.outI = 8;
            out[2].f.inI = 8;  out[2].f.outI = 7;
            out[3].f.inI = 7;  out[3].f.outI = 9;
        }
        else
        {
            out[0].v.Set( -e.x, -e.y,  e.z );
            out[1].v.Set( -e.x,  e.y,  e.z );
            out[2].v.Set( -e.x,  e.y, -e.z );
            out[3].v.Set( -e.x, -e.y, -e.z );

            out[0].f.inI = 5;  out[0].f.outI = 11;
            out[1].f.inI = 11; out[1].f.outI = 3;
            out[2].f.inI = 3;  out[2].f.outI = 10;
            out[3].f.inI = 10; out[3].f.outI = 5;
        }
    }
    else if (absN.y > absN.x && absN.y > absN.z)
    {
        if (norm.y > 0)
        {
            out[0].v.Set(-e.x,  e.y,  e.z);
            out[1].v.Set( e.x,  e.y,  e.z);
            out[2].v.Set( e.x,  e.y, -e.z);
            out[3].v.Set(-e.x,  e.y, -e.z);

            out[0].f.inI = 3; out[0].f.outI = 0;
            out[1].f.inI = 0; out[1].f.outI = 1;
            out[2].f.inI = 1; out[2].f.outI = 2;
            out[3].f.inI = 2; out[3].f.outI = 3;
        }
        else
        {
            out[0].v.Set( e.x, -e.y,  e.z);
            out[1].v.Set(-e.x, -e.y,  e.z);
            out[2].v.Set(-e.x, -e.y, -e.z);
            out[3].v.Set( e.x, -e.y, -e.z);

            out[0].f.inI = 7; out[0].f.outI = 4;
            out[1].f.inI = 4; out[1].f.outI = 5;
            out[2].f.inI = 5; out[2].f.outI = 6;
            out[3].f.inI = 6; out[3].f.outI = 7;
        }
    }
    else
    {
        if (norm.z > 0)
        {
            out[0].v.Set(-e.x,  e.y,  e.z);
            out[1].v.Set(-e.x, -e.y,  e.z);
            out[2].v.Set( e.x, -e.y,  e.z);
            out[3].v.Set( e.x,  e.y,  e.z);

            out[0].f.inI = 0;  out[0].f.outI = 11;
            out[1].f.inI = 11; out[1].f.outI = 4;
            out[2].f.inI = 4;  out[2].f.outI = 8;
            out[3].f.inI = 8;  out[3].f.outI = 0;
        }
        else
        {
            out[0].v.Set( e.x, -e.y, -e.z);
            out[1].v.Set(-e.x, -e.y, -e.z);
            out[2].v.Set(-e.x,  e.y, -e.z);
            out[3].v.Set( e.x,  e.y, -e.z);

            out[0].f.inI = 9;  out[0].f.outI = 6;
            out[1].f.inI = 6;  out[1].f.outI = 10;
            out[2].f.inI = 10; out[2].f.outI = 2;
            out[3].f.inI = 2;  out[3].f.outI = 9;
        }
    }

    // transform to world
    for (let i = 0; i < 4; ++i)
    {
        out[i].v = q3MulTransformVec3(tx, out[i].v);
    }
}


//--------------------------------------------------------------------------------------------------
// q3Orthographic
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
// q3Orthographic (faithful JS port with feature pair preservation)
//--------------------------------------------------------------------------------------------------
function q3Orthographic(sign, e, axis, clipEdge, input, inCount, output) {
    let outCount = 0;
    let a = input[inCount - 1];

    for (let i = 0; i < inCount; ++i) {
        let b = input[i];

        let da = sign * a.v.get(axis) - e;
        let db = sign * b.v.get(axis) - e;

        let cv = new q3ClipVertex();

        // Both in front or on the plane -> keep b
        if ((InFront(da) && InFront(db)) || On(da) || On(db)) {
            output[outCount++] = b;
        }
        // a in front, b behind -> intersection only
        else if (InFront(da) && Behind(db)) {
            // Copy feature data from b
            cv.f.inR  = b.f.inR;
            cv.f.outR = b.f.outR;
            cv.f.inI  = b.f.inI;
            cv.f.outI = b.f.outI;
            cv.f.key  = b.f.key;

            // Interpolate vertex position
            let t = da / (da - db);
            cv.v = q3Add(a.v, q3MulScalarVec3(t, q3Sub(b.v, a.v)));

            // Set clipEdge feature
            cv.f.outR = clipEdge;
            cv.f.outI = 0;

            output[outCount++] = cv;
        }
        // a behind, b in front -> intersection + b
        else if (Behind(da) && InFront(db)) {
            // Copy feature data from a
            cv.f.inR  = a.f.inR;
            cv.f.outR = a.f.outR;
            cv.f.inI  = a.f.inI;
            cv.f.outI = a.f.outI;
            cv.f.key  = a.f.key;

            // Interpolate vertex position
            let t = da / (da - db);
            cv.v = q3Add(a.v, q3MulScalarVec3(t, q3Sub(b.v, a.v)));

            // Set clipEdge feature
            cv.f.inR = clipEdge;
            cv.f.inI = 0;

            output[outCount++] = cv;
            output[outCount++] = b; // add the original vertex behind the plane
        }

        a = b;
    }

    return outCount;
}



//--------------------------------------------------------------------------------------------------
// q3Clip
//--------------------------------------------------------------------------------------------------

function q3Clip(rPos, e, clipEdges, basis, incident, outVerts, outDepths)
{
    let inCount = 4;
    let outCount;

    let input = new Array(8);
    let output = new Array(8);

    for (let i = 0; i < 4; ++i)
    {
        input[i] = new q3ClipVertex();
        input[i].v = q3MulMat3Vec3(q3Transpose(basis), q3Sub(incident[i].v, rPos));
    }

    outCount = q3Orthographic(1.0, e.x, 0, clipEdges[0], input, inCount, output);
    if (!outCount) return 0;

    inCount = q3Orthographic(1.0, e.y, 1, clipEdges[1], output, outCount, input);
    if (!inCount) return 0;

    outCount = q3Orthographic(-1.0, e.x, 0, clipEdges[2], input, inCount, output);
    if (!outCount) return 0;

    inCount = q3Orthographic(-1.0, e.y, 1, clipEdges[3], output, outCount, input);

    // final pass
    outCount = 0;

    for (let i = 0; i < inCount; ++i)
    {
        let d = input[i].v.z - e.z;

        if (d <= 0)
        {
            outVerts[outCount] = new q3ClipVertex();
            outVerts[outCount].v = q3Add(q3MulMat3Vec3(basis, input[i].v), rPos);
            outVerts[outCount].f = input[i].clone(); // use clone to preserve key
            outDepths[outCount++] = d;
        }
    }

    return outCount;
}

//--------------------------------------------------------------------------------------------------
// q3EdgesContact
//--------------------------------------------------------------------------------------------------
function q3EdgesContact(CA, CB, PA, QA, PB, QB)
{
    const DA = q3Sub(QA, PA);
    const DB = q3Sub(QB, PB);
    const r = q3Sub(PA, PB);

    const a = q3Dot(DA, DA);
    const e = q3Dot(DB, DB);
    const f = q3Dot(DB, r);
    const c = q3Dot(DA, r);
    const b = q3Dot(DA, DB);

    const denom = a * e - b * b;

    let TA, TB;

    /*if (Math.abs(denom) < 1e-8)
    {
        // Parallel fallback
        TA = 0;
        TB = (b > e ? c / b : f / e);
    }
    else
    {*/
        TA = (b * f - c * e) / denom;
        TB = (b * TA + f) / e;
    //}

    CA.Set(
        PA.x + DA.x * TA,
        PA.y + DA.y * TA,
        PA.z + DA.z * TA
    );

    CB.Set(
        PB.x + DB.x * TB,
        PB.y + DB.y * TB,
        PB.z + DB.z * TB
    );
}




//--------------------------------------------------------------------------------------------------
// q3SupportEdge
//--------------------------------------------------------------------------------------------------
function q3SupportEdge(tx, e, n, aOut, bOut)
{
    n = q3MulMat3Vec3(q3Transpose(tx.rotation), n);

    const absN = q3AbsVec3(n);

    const a = new q3Vec3(e.x, e.y, e.z);
    const b = new q3Vec3();

    if (absN.x > absN.y)
    {
        if (absN.y > absN.z)
            b.Set(e.x, e.y, -e.z);
        else
            b.Set(e.x, -e.y, e.z);
    }
    else
    {
        if (absN.x > absN.z)
            b.Set(e.x, e.y, -e.z);
        else
            b.Set(-e.x, e.y, e.z);
    }

    const sx = Math.sign(n.x) || 1;
    const sy = Math.sign(n.y) || 1;
    const sz = Math.sign(n.z) || 1;

    a.x *= sx; a.y *= sy; a.z *= sz;
    b.x *= sx; b.y *= sy; b.z *= sz;

    const wa = q3MulTransformVec3(tx, a);
    const wb = q3MulTransformVec3(tx, b);

    aOut.Set(wa.x, wa.y, wa.z);
    bOut.Set(wb.x, wb.y, wb.z);
}


function q3BoxtoBox(m, A, B)
{
    m.contactCount = 0;
    
    let atx = q3MulTransformTransform(A.body.m_tx, A.local);
    let btx = q3MulTransformTransform(B.body.m_tx, B.local);
    
    let eA = A.e;
    let eB = B.e;

    // --------------------------------------------------
    // C = transpose(A) * B
    // --------------------------------------------------

    // B's frame in A's space
    let C = q3MulMat3(q3Transpose(atx.rotation), btx.rotation);
    let absC = new q3Mat3();
    let parallel = false;
    
    const kCosTol = 1e-6;

    for (let i = 0; i < 3; ++i)
    {
        for (let j = 0; j < 3; ++j)
        {
            let val = Math.abs(C.get(i, j));
            
            absC.set(i, j, val);
            
            if (val + kCosTol >= 1.0)
                parallel = true;
        }
    }

    // --------------------------------------------------
    // t = A^T (b.pos - a.pos)
    // --------------------------------------------------
    let t = q3MulMat3Vec3(
        q3Transpose(atx.rotation),
        q3Sub(btx.position, atx.position)
    );

    // --------------------------------------------------
    // SAT tracking
    // --------------------------------------------------

    let s;
    let aMax = { value: -Infinity };
    let bMax = { value: -Infinity };
    let eMax = { value: -Infinity };

    let aAxis = { value: -1 };
    let bAxis = { value: -1 };
    let eAxis = { value: -1 };

    let nA = new q3Vec3();
    let nB = new q3Vec3();
    let nE = new q3Vec3();
  
    // --------------------------------------------------
    // A axes
    // --------------------------------------------------

    s = Math.abs(t.x) - (eA.x + q3Dot(absC.Column0(), eB));
    if (q3TrackFaceAxis(aAxis, 0, s, aMax, atx.rotation.ex, nA)) return;

    s = Math.abs(t.y) - (eA.y + q3Dot(absC.Column1(), eB));
    if (q3TrackFaceAxis(aAxis, 1, s, aMax, atx.rotation.ey, nA)) return;

    s = Math.abs(t.z) - (eA.z + q3Dot(absC.Column2(), eB));
    if (q3TrackFaceAxis(aAxis, 2, s, aMax, atx.rotation.ez, nA)) return;

    // --------------------------------------------------
    // B axes
    // --------------------------------------------------

    s = Math.abs(q3Dot(t, C.ex)) - (eB.x + q3Dot(absC.ex, eA));
    if (q3TrackFaceAxis(bAxis, 3, s, bMax, btx.rotation.ex, nB)) return;

    s = Math.abs(q3Dot(t, C.ey)) - (eB.y + q3Dot(absC.ey, eA));
    if (q3TrackFaceAxis(bAxis, 4, s, bMax, btx.rotation.ey, nB)) return;

    s = Math.abs(q3Dot(t, C.ez)) - (eB.z + q3Dot(absC.ez, eA));
    if (q3TrackFaceAxis(bAxis, 5, s, bMax, btx.rotation.ez, nB)) return;
  
    // --------------------------------------------------
    // Edge axes (ONLY if not parallel)
    // --------------------------------------------------
    
    // Collision is 100% stable when parallel is always true...
    // I don't know what's wrong with tracking edge axes
    parallel = true;
    
    if (!parallel)
    {
        let rA, rB;

        // Cross(a.x, b.x)
        rA = eA.y * absC.get(0,2) + eA.z * absC.get(0,1);
        rB = eB.y * absC.get(2,0) + eB.z * absC.get(1,0);
        s = Math.abs(t.z * C.get(0,1) - t.y * C.get(0,2)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 6, s, eMax, new q3Vec3(0, -C.get(0,2), C.get(0,1)), nE)) return;

        // Cross(a.x, b.y)
        rA = eA.y * absC.get(1,2) + eA.z * absC.get(1,1);
        rB = eB.x * absC.get(2,0) + eB.z * absC.get(0,0);
        s = Math.abs(t.z * C.get(1,1) - t.y * C.get(1,2)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 7, s, eMax, new q3Vec3(0, -C.get(1,2), C.get(1,1)), nE)) return;

        // Cross(a.x, b.z)
        rA = eA.y * absC.get(2,2) + eA.z * absC.get(2,1);
        rB = eB.x * absC.get(1,0) + eB.y * absC.get(0,0);
        s = Math.abs(t.z * C.get(2,1) - t.y * C.get(2,2)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 8, s, eMax, new q3Vec3(0, -C.get(2,2), C.get(2,1)), nE)) return;

        // Cross(a.y, b.x)
        rA = eA.x * absC.get(0,2) + eA.z * absC.get(0,0);
        rB = eB.y * absC.get(2,1) + eB.z * absC.get(1,1);
        s = Math.abs(t.x * C.get(0,2) - t.z * C.get(0,0)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 9, s, eMax, new q3Vec3(C.get(0,2), 0, -C.get(0,0)), nE)) return;

        // Cross(a.y, b.y)
        rA = eA.x * absC.get(1,2) + eA.z * absC.get(1,0);
        rB = eB.x * absC.get(2,1) + eB.z * absC.get(0,1);
        s = Math.abs(t.x * C.get(1,2) - t.z * C.get(1,0)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 10, s, eMax, new q3Vec3(C.get(1,2), 0, -C.get(1,0)), nE)) return;

        // Cross(a.y, b.z)
        rA = eA.x * absC.get(2,2) + eA.z * absC.get(2,0);
        rB = eB.x * absC.get(1,1) + eB.y * absC.get(0,1);
        s = Math.abs(t.x * C.get(2,2) - t.z * C.get(2,0)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 11, s, eMax, new q3Vec3(C.get(2,2), 0, -C.get(2,0)), nE)) return;

        // Cross(a.z, b.x)
        rA = eA.x * absC.get(0,1) + eA.y * absC.get(0,0);
        rB = eB.y * absC.get(2,2) + eB.z * absC.get(1,2);
        s = Math.abs(t.y * C.get(0,0) - t.x * C.get(0,1)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 12, s, eMax, new q3Vec3(-C.get(0,1), C.get(0,0), 0), nE)) return;

        // Cross(a.z, b.y)
        rA = eA.x * absC.get(1,1) + eA.y * absC.get(1,0);
        rB = eB.x * absC.get(2,2) + eB.z * absC.get(0,2);
        s = Math.abs(t.y * C.get(1,0) - t.x * C.get(1,1)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 13, s, eMax, new q3Vec3(-C.get(1,1), C.get(1,0), 0), nE)) return;

        // Cross(a.z, b.z)
        rA = eA.x * absC.get(2,1) + eA.y * absC.get(2,0);
        rB = eB.x * absC.get(1,2) + eB.y * absC.get(0,2);
        s = Math.abs(t.y * C.get(2,0) - t.x * C.get(2,1)) - (rA + rB);
        if (q3TrackEdgeAxis(eAxis, 14, s, eMax, new q3Vec3(-C.get(2,1), C.get(2,0), 0), nE)) return;
    }

    // --------------------------------------------------
    // Axis selection (UNCHANGED)
    // --------------------------------------------------

    const kRelTol = 0.95;
    const kAbsTol = 0.01;

    let axis, sMax, n;

    let faceMax = Math.max(aMax.value, bMax.value);

    if (kRelTol * eMax.value > faceMax + kAbsTol)
    {
        axis = eAxis.value;
        sMax = eMax.value;
        n = nE;
    }
    else
    {
        if (kRelTol * bMax.value > aMax.value + kAbsTol)
        {
            axis = bAxis.value;
            sMax = bMax.value;
            n = nB;
        }
        else
        {
            axis = aAxis.value;
            sMax = aMax.value;
            n = nA;
        }
    }
  
    // (optional?) normalize product
    //n = q3Normalize(n);
    //if (q3Dot(n, q3Normalize(q3Sub(btx.position, atx.position))) < 0)
    if (q3Dot(n, q3Sub(btx.position, atx.position)) < 0)
        n = n.neg();

    if (axis === -1)
        return;

    if (axis < 6)
    {
        let rtx, itx, eR, eI, flip;

        if (axis < 3)
        {
            rtx = atx;
            itx = btx;
            eR = eA;
            eI = eB;
            flip = false;
        }
        else
        {
            rtx = btx;
            itx = atx;
            eR = eB;
            eI = eA;
            flip = true;
            n = n.neg();
        }

        let incident = [
            new q3ClipVertex(),
            new q3ClipVertex(),
            new q3ClipVertex(),
            new q3ClipVertex()
        ];
        
        q3ComputeIncidentFace(incident, eI, itx, n);
        
        let clipEdges = [0,0,0,0];
        let basis = new q3Mat3();
        let e = new q3Vec3();

        q3ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, clipEdges, basis, e);
      
        let outVerts = [];
        let depths = [];
      
        // clip to reference point instead
        //let refPos = q3Add(rtx.position, q3MulScalarVec3(e.z, n));

        let outNum = q3Clip(
            rtx.position, // refPos,
            e,
            clipEdges,
            basis,
            incident,
            outVerts,
            depths
        );
        
        if (outNum)
        {
            m.contactCount = outNum;
            m.normal = flip ? n.neg() : n;
            
            for (let i = 0; i < outNum; ++i)
            {
                let c = m.contacts[i];
                let pair = outVerts[i].f;

                if (flip)
                {
                    let tmp;
                    tmp = pair.inI; pair.inI = pair.inR; pair.inR = tmp;
                    tmp = pair.outI; pair.outI = pair.outR; pair.outR = tmp;
                }

                c.fp = pair.clone();
                c.position = outVerts[i].v.clone();
                c.penetration = depths[i];
            }
            
        }
    }
    else
    {
        n = q3MulMat3Vec3(atx.rotation, n);

        // (optional?) normalize product
        //n = q3Normalize(n);
        //if (q3Dot(n, q3Normalize(q3Sub(btx.position, atx.position))) < 0)
        if (q3Dot(n, q3Sub(btx.position, atx.position)) < 0)
            n = n.neg();

        let PA, QA, PB, QB;
        q3SupportEdge(atx, eA, n, PA = new q3Vec3(), QA = new q3Vec3());
        q3SupportEdge(btx, eB, n.neg(), PB = new q3Vec3(), QB = new q3Vec3());

        let CA = new q3Vec3();
        let CB = new q3Vec3();

        q3EdgesContact(CA, CB, PA, QA, PB, QB);

        m.normal = n;
        m.contactCount = 1;

        let c = m.contacts[0];
        c.fp.key = axis;
        c.position = q3MulScalarVec3(0.5, q3Add(CA, CB));
        c.penetration = sMax;
    }
}