//--------------------------------------------------------------------------------------------------
// q3Body.js
//--------------------------------------------------------------------------------------------------

const eStaticBody = 0;
const eDynamicBody = 1;
const eKinematicBody = 2;


class q3BodyDef {

constructor()
{
    this.axis = new q3Vec3(1,0,0);
    this.angle = 0;

    this.position = new q3Vec3();
    this.linearVelocity = new q3Vec3();
    this.angularVelocity = new q3Vec3();

    this.gravityScale = 1;

    this.layers = 1;
    this.userData = null;

    this.linearDamping = 0; // 0
    this.angularDamping = 0.1; // 0.1

    this.bodyType = eStaticBody;

    this.allowSleep = true;
    this.awake = true;
    this.active = true;

    this.lockAxisX = false;
    this.lockAxisY = false;
    this.lockAxisZ = false;
}
}


class q3Body {

constructor(def, scene)
{
    this.m_linearVelocity = def.linearVelocity.clone();
    this.m_angularVelocity = def.angularVelocity.clone();

    this.m_worldCenter = new q3Vec3();
    this.m_localCenter = new q3Vec3();
    
    this.m_force = new q3Vec3();
    this.m_torque = new q3Vec3();

    this.m_q = new q3Quaternion();
    
    let axis = def.axis;

    if (!axis || (axis.x === 0 && axis.y === 0 && axis.z === 0)) {
        axis = new q3Vec3(1, 0, 0);
    }

    this.m_q.SetFromAxisAngle(q3Normalize(axis), def.angle || 0);

    this.m_tx = new q3Transform();
    this.m_tx.rotation = this.m_q.ToMat3();
    this.m_tx.position = def.position.clone();
    
    this.m_sleepTime = 0;
    this.m_gravityScale = def.gravityScale;
    this.m_layers = def.layers;

    this.m_userData = def.userData;
    this.m_scene = scene;

    this.m_flags = 0;

    this.m_linearDamping = def.linearDamping;
    this.m_angularDamping = def.angularDamping;

    if (def.bodyType === eDynamicBody)
        this.m_flags |= q3Body.eDynamic;

    else
    {
        if (def.bodyType === eStaticBody)
        {
            this.m_flags |= q3Body.eStatic;
            this.m_linearVelocity = new q3Vec3();
            this.m_angularVelocity = new q3Vec3();
            this.m_force = new q3Vec3();
            this.m_torque = new q3Vec3();
        }
        else if (def.bodyType === eKinematicBody)
            this.m_flags |= q3Body.eKinematic;
    }

    if (def.allowSleep) this.m_flags |= q3Body.eAllowSleep;
    if (def.awake) this.m_flags |= q3Body.eAwake;
    if (def.active) this.m_flags |= q3Body.eActive;

    if (def.lockAxisX) this.m_flags |= q3Body.eLockAxisX;
    if (def.lockAxisY) this.m_flags |= q3Body.eLockAxisY;
    if (def.lockAxisZ) this.m_flags |= q3Body.eLockAxisZ;

    this.m_boxes = null;
    this.m_contactList = null;
}

AddBox(def)
{
    let aabb = new q3AABB();
    let box = new q3Box();

    box.local = def.m_tx.clone();
    box.e = def.m_e.clone();
    box.next = this.m_boxes;
    this.m_boxes = box;

    box.computeAABB(this.m_tx, aabb);

    box.body = this;
    box.friction = def.m_friction;
    box.restitution = def.m_restitution;
    box.density = def.m_density;
    box.sensor = def.m_sensor;

    this.CalculateMassData();

    this.m_scene.m_contactManager.m_broadphase.InsertBox(box, aabb);
    this.m_scene.m_newBox = true;

    return box;
}

RemoveBox(box)
{
    let node = this.m_boxes;
    let found = false;

    if (node === box)
    {
        this.m_boxes = node.next;
        found = true;
    }
    else
    {
        while (node)
        {
            if (node.next === box)
            {
                node.next = box.next;
                found = true;
                break;
            }

            node = node.next;
        }
    }

    let edge = this.m_contactList;

    while (edge)
    {
        let contact = edge.constraint;
        edge = edge.next;

        let A = contact.A;
        let B = contact.B;

        if (box === A || box === B)
            this.m_scene.m_contactManager.RemoveContact(contact);
    }

    this.m_scene.m_contactManager.m_broadphase.RemoveBox(box);

    this.CalculateMassData();
}

RemoveAllBoxes()
{
    while (this.m_boxes)
    {
        let next = this.m_boxes.next;

        this.m_scene.m_contactManager.m_broadphase.RemoveBox(this.m_boxes);

        this.m_boxes = next;
    }

    this.m_scene.m_contactManager.RemoveContactsFromBody(this);
}

ApplyLinearForce(force)
{
    this.m_force = this.m_force.add(force);
    this.SetToAwake();
}

ApplyForceAtWorldPoint(force, point)
{
    this.m_force = this.m_force.add(force);

    let torque = q3Cross(point.sub(this.m_worldCenter), force);
    this.m_torque = this.m_torque.add(torque);

    this.SetToAwake();
}

ApplyLinearImpulse(impulse)
{
    this.m_linearVelocity =
        this.m_linearVelocity.add(impulse.scale(this.m_invMass));

    this.SetToAwake();
}

ApplyLinearImpulseAtWorldPoint(impulse, point)
{
    this.m_linearVelocity =
        this.m_linearVelocity.add(impulse.scale(this.m_invMass));

    let ang =
        this.m_invInertiaWorld.mulVec3(
            q3Cross(point.sub(this.m_worldCenter), impulse)
        );

    this.m_angularVelocity = this.m_angularVelocity.add(ang);

    this.SetToAwake();
}

ApplyTorque(torque)
{
    this.m_torque = this.m_torque.add(torque);
}

SetToAwake()
{
    if (!(this.m_flags & q3Body.eAwake))
    {
        this.m_flags |= q3Body.eAwake;
        this.m_sleepTime = 0;
    }
}

SetToSleep()
{
    this.m_flags &= ~q3Body.eAwake;
    this.m_sleepTime = 0;

    this.m_linearVelocity = new q3Vec3();
    this.m_angularVelocity = new q3Vec3();
    this.m_force = new q3Vec3();
    this.m_torque = new q3Vec3();
}

IsAwake()
{
    return (this.m_flags & q3Body.eAwake) !== 0;
}

GetMass()
{
    return this.m_mass;
}

GetInvMass()
{
    return this.m_invMass;
}

GetGravityScale()
{
    return this.m_gravityScale;
}

SetGravityScale(scale)
{
    this.m_gravityScale = scale;
}

GetLocalPoint(p)
{
    return q3MulT(this.m_tx, p);
}

GetLocalVector(v)
{
    return q3MulT(this.m_tx.rotation, v);
}

GetWorldPoint(p)
{
    return q3Mul(this.m_tx, p);
}

GetWorldVector(v)
{
    return q3Mul(this.m_tx.rotation, v);
}

GetLinearVelocity()
{
    return this.m_linearVelocity;
}

GetVelocityAtWorldPoint(p)
{
    let dir = p.sub(this.m_worldCenter);
    let rel = q3Cross(this.m_angularVelocity, dir);
    return this.m_linearVelocity.add(rel);
}

SetLinearVelocity(v)
{
    if (this.m_flags & q3Body.eStatic)
        throw "Cannot set velocity on static body";

    if (q3Dot(v, v) > 0)
        this.SetToAwake();

    this.m_linearVelocity = v;
}

GetAngularVelocity()
{
    return this.m_angularVelocity;
}

SetAngularVelocity(v)
{
    if (this.m_flags & q3Body.eStatic)
        throw "Cannot set velocity on static body";

    if (q3Dot(v, v) > 0)
        this.SetToAwake();

    this.m_angularVelocity = v;
}

CanCollide(other)
{
    if (this === other) return false;

    if (!(this.m_flags & q3Body.eDynamic) &&
        !(other.m_flags & q3Body.eDynamic))
        return false;

    if (!(this.m_layers & other.m_layers))
        return false;

    return true;
}

GetTransform()
{
    return this.m_tx;
}

SetTransform(position)
{
    this.m_worldCenter = position.clone();
    
    this.SynchronizeProxies();
}

SetTransformAxis(position, axis, angle)
{
    this.m_worldCenter = position.clone();

    this.m_q.Set(axis, angle);
    this.m_tx.rotation = this.m_q.ToMat3();

    this.SynchronizeProxies();
}

GetFlags()
{
    return this.m_flags;
}

SetLayers(layers)
{
    this.m_layers = layers;
}

GetLayers()
{
    return this.m_layers;
}

GetQuaternion()
{
    return this.m_q;
}

GetUserData()
{
    return this.m_userData;
}

SetLinearDamping(d)
{
    this.m_linearDamping = d;
}

GetLinearDamping()
{
    return this.m_linearDamping;
}

SetAngularDamping(d)
{
    this.m_angularDamping = d;
}

GetAngularDamping()
{
    return this.m_angularDamping;
}

Render(render)
{
    let awake = this.IsAwake();
    let box = this.m_boxes;

    while (box)
    {
        box.Render(this.m_tx, awake, render);
        box = box.next;
    }
}

CalculateMassData()
{
    let inertia = q3Diagonal(0);

    this.m_invInertiaModel = q3Diagonal(0);
    this.m_invInertiaWorld = q3Diagonal(0);

    this.m_invMass = 0;
    this.m_mass = 0;

    if ((this.m_flags & q3Body.eStatic) || (this.m_flags & q3Body.eKinematic))
    {
        this.m_localCenter = new q3Vec3();
        this.m_worldCenter = this.m_tx.position.clone();
        return;
    }

    let mass = 0;
    let lc = new q3Vec3();

    for (let box = this.m_boxes; box; box = box.next)
    {
        if (box.density === 0)
            continue;

        let md = new q3MassData();
        box.computeMass(md);

        mass += md.mass;
        inertia = inertia.add(md.inertia);
        lc = lc.add(md.center.scale(md.mass));
    }

    if (mass > 0)
    {
        this.m_mass = mass;
        this.m_invMass = 1 / mass;

        lc = lc.scale(this.m_invMass);

        
        let identity = new q3Mat3();
        q3Identity(identity);

        let I = identity.mulScalar(q3Dot(lc, lc));
        let outer = q3OuterProduct(lc, lc);

        inertia = inertia.sub(
            I.sub(outer).mulScalar(mass)
        );
        

        this.m_invInertiaModel = q3Inverse(inertia);
    }
    else
    {
        this.m_invMass = 1;
        this.m_invInertiaModel = q3Diagonal(0);
        this.m_invInertiaWorld = q3Diagonal(0);
    }


    this.m_localCenter = lc;
    this.m_worldCenter = q3Mul(this.m_tx, lc);
}


SynchronizeProxies()
{
    let broadphase = this.m_scene.m_contactManager.m_broadphase;

    this.m_tx.position =
        this.m_worldCenter.sub(
            q3Mul(this.m_tx.rotation, this.m_localCenter)
        );

    let aabb = new q3AABB();
    let tx = this.m_tx;
    let box = this.m_boxes;

    while (box)
    {
        box.computeAABB(tx, aabb);
        
        if (box.broadPhaseIndex >= 0) {
            broadphase.Update(box.broadPhaseIndex, aabb);
        } else {
            box.broadPhaseIndex = broadphase.InsertBox(box, aabb);
            //broadphase.Update(box.broadPhaseIndex, aabb);
        }

        box = box.next;
    }
}

}


q3Body.eAwake = 0x001;
q3Body.eActive = 0x002;
q3Body.eAllowSleep = 0x004;
q3Body.eIsland = 0x010;
q3Body.eStatic = 0x020;
q3Body.eDynamic = 0x040;
q3Body.eKinematic = 0x080;
q3Body.eLockAxisX = 0x100;
q3Body.eLockAxisY = 0x200;
q3Body.eLockAxisZ = 0x400;

