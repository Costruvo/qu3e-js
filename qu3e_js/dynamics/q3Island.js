class q3VelocityState {
    constructor() {
        this.v = new q3Vec3(); // linear velocity
        this.w = new q3Vec3(); // angular velocity
    }
}

class q3Island {
    constructor(bodyCapacity = 16, contactCapacity = 16) {
        this.m_bodies = new Array(bodyCapacity);        // q3Body[]
        this.m_velocities = new Array(bodyCapacity);    // q3VelocityState[]
        this.m_bodyCapacity = bodyCapacity;
        this.m_bodyCount = 0;

        //for (let i = 0; i < bodyCapacity; ++i){
        //    this.m_velocities[i] = new q3VelocityState();
        //}

        this.m_contacts = new Array(contactCapacity);   // q3ContactConstraint[]
        this.m_contactStates = new Array(contactCapacity); // q3ContactConstraintState[]
        this.m_contactCapacity = contactCapacity;
        this.m_contactCount = 0;

        this.m_dt = 1 / 60;
        this.m_gravity = new q3Vec3(0, -9.8, 0);
        this.m_iterations = 20;

        this.m_allowSleep = true;
        this.m_enableFriction = true;
    }

    AddBody(body) {
        if (this.m_bodyCount < this.m_bodyCapacity)
            this.m_bodyCapacity *= 2;
        
        body.m_islandIndex = this.m_bodyCount;
        this.m_bodies[this.m_bodyCount] = body;
        this.m_velocities[this.m_bodyCount] = new q3VelocityState();
        this.m_bodyCount++;
    }

    AddContact(contact) {
        if (this.m_contactCount < this.m_contactCapacity)
            this.m_contactCapacity *= 2;

        this.m_contacts[this.m_contactCount] = contact;
        this.m_contactStates[this.m_contactCount] = new q3ContactConstraintState();
        this.m_contactCount++;
    }

    Initialize() {
        // Initialize velocities from bodies
        for (let i = 0; i < this.m_bodyCount; ++i) {
            let body = this.m_bodies[i];
            let vel = this.m_velocities[i];
            vel.v = body.m_linearVelocity.clone();
            vel.w = body.m_angularVelocity.clone();
        }

      
        // Initialize contacts
        for (let i = 0; i < this.m_contactCount; ++i) {
            let cs = this.m_contactStates[i]; // cs = q3ContactConstraintState*
            let cc = this.m_contacts[i]; // cc = q3ContactConstraint*

            cs.centerA = cc.bodyA.m_worldCenter.clone();
            cs.centerB = cc.bodyB.m_worldCenter.clone();
            cs.iA = cc.bodyA.m_invInertiaWorld.clone();
            cs.iB = cc.bodyB.m_invInertiaWorld.clone();
            cs.mA = cc.bodyA.m_invMass;
            cs.mB = cc.bodyB.m_invMass;
            cs.restitution = cc.restitution;
            cs.friction = cc.friction;
            cs.indexA = cc.bodyA.m_islandIndex;//this.m_bodies.indexOf(cc.bodyA);
            cs.indexB = cc.bodyB.m_islandIndex;//this.m_bodies.indexOf(cc.bodyB);
            cs.normal = cc.manifold.normal.clone();
            cs.tangentVectors[0] = cc.manifold.tangentVectors[0].clone();
            cs.tangentVectors[1] = cc.manifold.tangentVectors[1].clone();
            cs.contactCount = cc.manifold.contactCount;

            for (let j = 0; j < cs.contactCount; ++j) {
                let c = cs.contacts[j]; // c = q3ContactState*
                let mc = cc.manifold.contacts[j]; // mc = q3Contact*
                c.ra = mc.position.clone().sub(cs.centerA);
                c.rb = mc.position.clone().sub(cs.centerB);
                c.penetration = mc.penetration;
                c.normalImpulse = mc.normalImpulse;
                c.tangentImpulse[0] = mc.tangentImpulse[0];
                c.tangentImpulse[1] = mc.tangentImpulse[1];
            }

            /*
            // Compute tangent vectors here (optional)
            let n = cs.normal;
            if (Math.abs(n.x) >= 0.57735) { // 1/sqrt(3)
                cs.tangentVectors[0] = q3Normalize(new q3Vec3(n.y, -n.x, 0));//.normalize();
            } else {
                cs.tangentVectors[0] = q3Normalize(new q3Vec3(0, n.z, -n.y));//.normalize();
            }
            cs.tangentVectors[1] = q3Cross(n, cs.tangentVectors[0]);
            */
        }
    }

    Solve()
    {
        const dt = this.m_dt;

        // --------------------------------------------------
        // 1. Apply gravity + integrate velocities
        // --------------------------------------------------
        for (let i = 0; i < this.m_bodyCount; ++i)
        {
            const body = this.m_bodies[i];
            const v = this.m_velocities[i];

            if (body.m_flags & q3Body.eDynamic)
            {
                body.ApplyLinearForce(this.m_gravity.scale(body.m_gravityScale * body.m_mass));

                // --- World inertia ---
                const R = body.m_tx.rotation;
                body.m_invInertiaWorld =
                    q3MulMat3(q3MulMat3(R, body.m_invInertiaModel), q3Transpose(R));

                // --- Integrate velocity ---
                body.m_linearVelocity =
                    body.m_linearVelocity.add(body.m_force.scale(body.m_invMass * dt));

                body.m_angularVelocity = body.m_angularVelocity.add(
                    q3MulMat3Vec3(body.m_invInertiaWorld, body.m_torque).scale(dt)
                );

                // --- Damping ---
                const linDamp = 1.0 / (1.0 + dt * body.m_linearDamping);
                const angDamp = 1.0 / (1.0 + dt * body.m_angularDamping);
                
                body.m_linearVelocity = body.m_linearVelocity.scale(linDamp);
                body.m_angularVelocity = body.m_angularVelocity.scale(angDamp);
                
                
                // DRIFT PREVENTION
                const kRestTol = 0.1;
                if (Math.abs(body.m_linearVelocity.x) < kRestTol) body.m_linearVelocity.x = 0;
                if (Math.abs(body.m_linearVelocity.y) < kRestTol) body.m_linearVelocity.y = 0;
                if (Math.abs(body.m_linearVelocity.z) < kRestTol) body.m_linearVelocity.z = 0;
                
            }

            // Copy into island buffers
            v.v = body.m_linearVelocity.clone();
            v.w = body.m_angularVelocity.clone();
        }

        
        // --------------------------------------------------
        // 2. Solve contacts
        // --------------------------------------------------
        const solver = new q3ContactSolver();
        solver.Initialize(this);
        solver.PreSolve(dt);

        for (let i = 0; i < this.m_iterations; ++i)
            solver.Solve();

        solver.ShutDown();

        // --------------------------------------------------
        // 3. Integrate positions + rotation
        // --------------------------------------------------
        for (let i = 0; i < this.m_bodyCount; ++i)
        {
            const body = this.m_bodies[i];
            const v = this.m_velocities[i];

            if (body.m_flags & q3Body.eStatic)
                continue;

            // Copy velocities back
            body.m_linearVelocity = v.v.clone();
            body.m_angularVelocity = v.w.clone();

            // --- Integrate position ---
            body.m_worldCenter =
                body.m_worldCenter.add(body.m_linearVelocity.scale(dt));

            // --- Integrate rotation ---
            body.m_q.integrate(body.m_angularVelocity, dt);
            body.m_q = q3NormalizeQuat(body.m_q);

            body.m_tx.rotation = body.m_q.ToMat3();
        }
        
        if (this.m_allowSleep) {
            // Find minimum sleep time of the entire island
            let minSleepTime = Number.MAX_VALUE;

            for (let i = 0; i < this.m_bodyCount; ++i) {
                const body = this.m_bodies[i];

                // Skip static bodies
                if (body.m_flags & q3Body.eStatic)
                    continue;

                const sqrLinVel = q3Dot(body.m_linearVelocity, body.m_linearVelocity);
                const sqrAngVel = q3Dot(body.m_angularVelocity, body.m_angularVelocity);
                const linTol = Q3_SLEEP_LINEAR;   // e.g., 0.01
                const angTol = Q3_SLEEP_ANGULAR;  // e.g., 0.01

                if (sqrLinVel > linTol || sqrAngVel > angTol) {
                    minSleepTime = 0.0;
                    body.m_sleepTime = 0.0;
                } else {
                    body.m_sleepTime = (body.m_sleepTime || 0.0) + this.m_dt;
                    minSleepTime = Math.min(minSleepTime, body.m_sleepTime);
                }
            }

            // Put entire island to sleep if minimum sleep time exceeds threshold
            if (minSleepTime > Q3_SLEEP_TIME) {  // e.g., 0.5 seconds
                for (let i = 0; i < this.m_bodyCount; ++i) {
                    this.m_bodies[i].SetToSleep();
                }
            }
        }
    }
    
}
