
class q3ContactConstraintState {
    constructor() {
        this.contacts = [];
        
        for (let i = 0; i < 8; i++)
        {
            this.contacts[i] = {
                ra: new q3Vec3(),
                rb: new q3Vec3(),
                penetration: 0,

                normalImpulse: 0,
                tangentImpulse: [0, 0],

                normalMass: 0,
                tangentMass: [0, 0],

                bias: 0
            };
        }

        this.normal = new q3Vec3();

        this.tangentVectors = [
            new q3Vec3(),
            new q3Vec3()
        ];

        this.centerA = new q3Vec3();
        this.centerB = new q3Vec3();

        this.mA = 0;
        this.mB = 0;

        this.iA = new q3Mat3();
        this.iB = new q3Mat3();

        this.friction = 0;
        this.restitution = 0;

        this.contactCount = 0;

        this.indexA = 0;
        this.indexB = 0;
    }
}


class q3ContactSolver {
    constructor() {
        this.m_island = null;
        this.m_contacts = null;
        this.m_contactCount = 0;
        this.m_velocities = null;
        this.m_enableFriction = false;
    }

    Initialize(island) {
        this.m_island = island;
        this.m_contactCount = island.m_contactCount;
        this.m_contacts = island.m_contactStates;
        this.m_velocities = island.m_velocities;
        this.m_enableFriction = island.m_enableFriction;
    }

    ShutDown() {
        for (let i = 0; i < this.m_contactCount; ++i) {
            let cState = this.m_contacts[i];
            let cc = this.m_island.m_contacts[i];

            for (let j = 0; j < cState.contactCount; ++j) {
                let oc = cc.manifold.contacts[j];
                let cs = cState.contacts[j];

                oc.normalImpulse = cs.normalImpulse;
                oc.tangentImpulse[0] = cs.tangentImpulse[0];
                oc.tangentImpulse[1] = cs.tangentImpulse[1];
            }
        }
    }

    PreSolve(dt) {
        for (let i = 0; i < this.m_contactCount; ++i) {
            let cs = this.m_contacts[i]; // cs = q3ContactConstraintState*
            let vA = this.m_velocities[cs.indexA].v;
            let wA = this.m_velocities[cs.indexA].w;
            let vB = this.m_velocities[cs.indexB].v;
            let wB = this.m_velocities[cs.indexB].w;

            for (let j = 0; j < cs.contactCount; ++j) {
                let c = cs.contacts[j];

                // Precompute normalMass
                let raCn = q3Cross(c.ra, cs.normal);
                let rbCn = q3Cross(c.rb, cs.normal);
                let nm = cs.mA + cs.mB;
                let tm = new Array(2);
                tm[0] = nm;
                tm[1] = nm;
                nm += q3Dot(raCn, q3MulMat3Vec3(cs.iA, raCn)) + q3Dot(rbCn, q3MulMat3Vec3(cs.iB, rbCn));
                c.normalMass = q3Invert(nm);

                // Precompute tangentMass
                for (let k = 0; k < 2; ++k) {
                    let raCt = q3Cross(cs.tangentVectors[k], c.ra);
                    let rbCt = q3Cross(cs.tangentVectors[k], c.rb);
                    tm[k] += q3Dot(raCt, q3MulMat3Vec3(cs.iA, raCt)) + q3Dot(rbCt, q3MulMat3Vec3(cs.iB, rbCt));
                    c.tangentMass[k] = q3Invert(tm[k]);
                }

                // Bias
                c.bias = -Q3_BAUMGARTE * (1.0 / dt) * Math.min(0.0, c.penetration + Q3_PENETRATION_SLOP);

                // Warm start
                let P = q3MulScalarVec3(c.normalImpulse, cs.normal);

                if (this.m_enableFriction) {
                    P = q3Add(P, q3MulScalarVec3(c.tangentImpulse[0], cs.tangentVectors[0]));
                    P = q3Add(P, q3MulScalarVec3(c.tangentImpulse[1], cs.tangentVectors[1]));
                }

                vA = vA.sub(P.mulScalar(cs.mA));
                wA = wA.sub(q3MulMat3Vec3(cs.iA, q3Cross(c.ra, P)));

                vB = vB.add(P.mulScalar(cs.mB));
                wB = wB.add(q3MulMat3Vec3(cs.iB, q3Cross(c.rb, P)));

                // Restitution bias
                // r32 dv = q3Dot( vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra ), cs->normal );
                let dv = q3Dot(q3Sub(
                    q3Add(vB, q3Cross(wB, c.rb)),
                    q3Add(vA, q3Cross(wA, c.ra))
                ), cs.normal);
                
                if (dv < -1.0) {
                    c.bias += -(cs.restitution) * dv;
                }
            }

            this.m_velocities[cs.indexA].v = vA;
            this.m_velocities[cs.indexA].w = wA;
            this.m_velocities[cs.indexB].v = vB;
            this.m_velocities[cs.indexB].w = wB;
        }
    }

    Solve() {
        for (let i = 0; i < this.m_contactCount; ++i) {
            let cs = this.m_contacts[i];
            let vA = this.m_velocities[cs.indexA].v;
            let wA = this.m_velocities[cs.indexA].w;
            let vB = this.m_velocities[cs.indexB].v;
            let wB = this.m_velocities[cs.indexB].w;

            for (let j = 0; j < cs.contactCount; ++j) {
                let c = cs.contacts[j]; // c = q3ContactState*

                let dv = vB.add(q3Cross(wB, c.rb)).sub(vA).sub(q3Cross(wA, c.ra));

                // Friction
                if (this.m_enableFriction) {
                    for (let k = 0; k < 2; ++k) {
                        let lambda = -q3Dot(dv, cs.tangentVectors[k]) * c.tangentMass[k];
                        let maxLambda = (cs.friction * .01) * c.normalImpulse;

                        let oldPT = c.tangentImpulse[k];
                        c.tangentImpulse[k] = q3Clamp(-maxLambda, maxLambda, oldPT + lambda);
                        lambda = c.tangentImpulse[k] - oldPT;
                        
                        let impulse = cs.tangentVectors[k].mulScalar(lambda);
                        vA = vA.sub(impulse.mulScalar(cs.mA));
                        wA = wA.sub(q3MulMat3Vec3(cs.iA, q3Cross(c.ra, impulse)));

                        vB = vB.add(impulse.mulScalar(cs.mB));
                        wB = wB.add(q3MulMat3Vec3(cs.iB, q3Cross(c.rb, impulse)));
                    }
                }

                // Normal
                dv = vB.add(q3Cross(wB, c.rb)).sub(vA).sub(q3Cross(wA, c.ra));
                
                // Normal impulse
                let vn = q3Dot(dv, cs.normal);
                
                // Factor in positional bias to calculate impulse scalar j
                let lambda = c.normalMass * (-vn + c.bias);

                // Clamp impulse
                let oldPN = c.normalImpulse;
                c.normalImpulse = Math.max(oldPN + lambda, 0);
                lambda = c.normalImpulse - oldPN;

                // Apply impulse
                let impulse = cs.normal.mulScalar(lambda);


                
                vA = vA.sub(impulse.mulScalar(cs.mA));
                wA = wA.sub(q3MulMat3Vec3(cs.iA, q3Cross(c.ra, impulse)));

                vB = vB.add(impulse.mulScalar(cs.mB));
                wB = wB.add(q3MulMat3Vec3(cs.iB, q3Cross(c.rb, impulse)));
            }

            this.m_velocities[cs.indexA].v = vA;
            this.m_velocities[cs.indexA].w = wA;
            this.m_velocities[cs.indexB].v = vB;
            this.m_velocities[cs.indexB].w = wB;
        }
    }
}