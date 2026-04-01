//--------------------------------------------------------------------------------------------------
// q3Scene.js
// Direct JS port of qu3e q3Scene.h / q3Scene.cpp
// Full Step() logic including islands, contacts, DFS, and broadphase updates
//--------------------------------------------------------------------------------------------------

class q3Scene {
    constructor(dt, gravity = new q3Vec3(0, -9.8, 0), iterations = 12) {
        this.m_dt = dt;
        this.m_gravity = gravity;
        this.m_iterations = iterations;

        this.m_bodyList = null;
        this.m_bodyCount = 0;

        this.m_contactManager = new q3ContactManager();
        this.m_newBox = false;

        this.m_allowSleep = true;
        this.m_enableFriction = true;

        this.m_stack = []; // used for DFS in islands
    }

    //--------------------------------------------------------------------------------------------------
    Step() {
        if (this.m_newBox) {
            this.m_contactManager.m_broadphase.UpdatePairs();
            this.m_newBox = false;
        }

        this.m_contactManager.TestCollisions();

        // Clear island flags
        for (let body = this.m_bodyList; body; body = body.next)
        {
            body.m_flags &= ~q3Body.eIsland;
            body.m_islandIndex = undefined;
        }

        // Prepare island
        let island = new q3Island();
        island.m_allowSleep = this.m_allowSleep;
        island.m_enableFriction = this.m_enableFriction;
        island.m_dt = this.m_dt;
        island.m_iterations = this.m_iterations;
        island.m_gravity = this.m_gravity;
        island.m_bodyCount = 0;
        island.m_contactCount = 0;

        const stackSize = this.m_bodyCount;
        const stack = new Array(stackSize);

        for (let seed = this.m_bodyList; seed; seed = seed.next) {
            if (seed.m_flags & q3Body.eIsland) continue; // Seed cannot be apart of an island already
            if (!(seed.m_flags & q3Body.eAwake)) continue; // Seed must be awake
            if (seed.m_flags & q3Body.eStatic) continue; // Seed cannot be a static body in order to keep islands as small as possible

            let stackCount = 0;
            stack[stackCount++] = seed;
            island.m_bodyCount = 0;
            island.m_contactCount = 0;

            seed.m_flags |= q3Body.eIsland;

            // DFS on constraint graph
            while (stackCount > 0) {
                let body = stack[--stackCount];
                island.AddBody(body);

                // Awaken all bodies connected to
                body.SetToAwake();

                // Do not search across static bodies to keep island
                // formations as small as possible, however the static
                // body itself should be apart of the island in order
                // to properly represent a full contact
                if (body.m_flags & q3Body.eStatic) continue;

                
                let edge = body.m_contactList;
                while (edge) {
                    const contact = edge.constraint;

                    if (contact.m_flags & q3ContactConstraint.eIsland) {
                        edge = edge.next;
                        continue;
                    }

                    if (!(contact.m_flags & q3ContactConstraint.eColliding)) {
                        edge = edge.next;
                        continue;
                    }

                    if (contact.A.sensor || contact.B.sensor) {
                        edge = edge.next;
                        continue;
                    }

                    contact.m_flags |= q3ContactConstraint.eIsland;
                    island.AddContact(contact);
                    
                    const other = edge.other;
                    if (other.m_flags & q3Body.eIsland) {
                        edge = edge.next;
                        continue;
                    }

                    stack[stackCount++] = other;
                    other.m_flags |= q3Body.eIsland;

                    edge = edge.next;
                }
            }

            if (island.m_bodyCount === 0) continue;

            island.Initialize();
            island.Solve();

            // Reset static body island flags
            for (let i = 0; i < island.m_bodyCount; i++) {
                let body = island.m_bodies[i];
                
                if (body.m_flags & q3Body.eStatic)
                    body.m_flags &= ~q3Body.eIsland;
            }
        }

        // Update broadphase proxies
        for (let body = this.m_bodyList; body; body = body.next) {
            if (body.m_flags & q3Body.eStatic) continue;
            body.SynchronizeProxies();
        }

        this.m_contactManager.FindNewContacts();

        // Clear forces/torques
        for (let body = this.m_bodyList; body; body = body.next) {
            q3Identity(body.m_force);
            q3Identity(body.m_torque);
        }
    }

    //--------------------------------------------------------------------------------------------------
    CreateBody(def) {
        const body = new q3Body(def, this);

        body.prev = null;
        body.next = this.m_bodyList;
        if (this.m_bodyList) this.m_bodyList.prev = body;
        this.m_bodyList = body;
        this.m_bodyCount++;
        return body;
    }

    RemoveBody(body) {
        if (this.m_bodyCount <= 0) return;

        this.m_contactManager.RemoveContactsFromBody(body);
        body.RemoveAllBoxes();

        if (body.next) body.next.prev = body.prev;
        if (body.prev) body.prev.next = body.next;
        if (body === this.m_bodyList) this.m_bodyList = body.next;

        this.m_bodyCount--;
    }

    RemoveAllBodies() {
        let body = this.m_bodyList;
        while (body) {
            let next = body.next;
            body.RemoveAllBoxes();
            body = next;
        }
        this.m_bodyList = null;
        this.m_bodyCount = 0;
    }

    SetAllowSleep(allowSleep) {
        this.m_allowSleep = allowSleep;
        if (!allowSleep) {
            for (let body = this.m_bodyList; body; body = body.next)
                body.SetToAwake();
        }
    }

    SetIterations(iterations) {
        this.m_iterations = Math.max(1, iterations);
    }

    SetEnableFriction(enabled) {
        this.m_enableFriction = enabled;
    }

    SetGravity(gravity) {
        this.m_gravity = gravity;
    }

    GetGravity() {
        return this.m_gravity;
    }

    SetContactListener(listener) {
        this.m_contactManager.contactListener = listener;
    }
}