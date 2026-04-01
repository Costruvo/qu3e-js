class q3ContactManager {

constructor(stack)
{
    this.m_stack = stack;

    this.m_allocator = new q3PagedAllocator(256); // simplified JS allocator
    this.m_broadphase = new q3BroadPhase(this);

    this.m_contactList = null;
    this.m_contactCount = 0;

    this.m_contactListener = null;
}

AddContact(A, B)
{
    let bodyA = A.body;
    let bodyB = B.body;

    if (!bodyA.CanCollide(bodyB))
        return;

    let edge = bodyA.m_contactList;

    while (edge)
    {
        if (edge.other === bodyB)
        {
            let shapeA = edge.constraint.A;
            let shapeB = edge.constraint.B;

            if (A === shapeA && B === shapeB)
                return;
        }

        edge = edge.next;
    }

    let contact = new q3ContactConstraint();

    contact.A = A;
    contact.B = B;

    contact.bodyA = bodyA;
    contact.bodyB = bodyB;

    contact.manifold.SetPair(A, B);

    contact.m_flags = 0;

    contact.friction = q3MixFriction(A, B);
    contact.restitution = q3MixRestitution(A, B);

    contact.manifold.contactCount = 0;

    for (let i = 0; i < 8; i++)
        contact.manifold.contacts[i].warmStarted = 0;

    contact.prev = null;
    contact.next = this.m_contactList;

    if (this.m_contactList)
        this.m_contactList.prev = contact;

    this.m_contactList = contact;

    // connect body A
    contact.edgeA.constraint = contact;
    contact.edgeA.other = bodyB;

    contact.edgeA.prev = null;
    contact.edgeA.next = bodyA.m_contactList;

    if (bodyA.m_contactList)
        bodyA.m_contactList.prev = contact.edgeA;

    bodyA.m_contactList = contact.edgeA;

    // connect body B
    contact.edgeB.constraint = contact;
    contact.edgeB.other = bodyA;

    contact.edgeB.prev = null;
    contact.edgeB.next = bodyB.m_contactList;

    if (bodyB.m_contactList)
        bodyB.m_contactList.prev = contact.edgeB;

    bodyB.m_contactList = contact.edgeB;

    bodyA.SetToAwake();
    bodyB.SetToAwake();

    this.m_contactCount++;
}

FindNewContacts()
{
    this.m_broadphase.UpdatePairs();
}

RemoveContact(contact)
{
    let A = contact.bodyA;
    let B = contact.bodyB;

    if (contact.edgeA.prev)
        contact.edgeA.prev.next = contact.edgeA.next;

    if (contact.edgeA.next)
        contact.edgeA.next.prev = contact.edgeA.prev;

    if (A.m_contactList === contact.edgeA)
        A.m_contactList = contact.edgeA.next;

    if (contact.edgeB.prev)
        contact.edgeB.prev.next = contact.edgeB.next;

    if (contact.edgeB.next)
        contact.edgeB.next.prev = contact.edgeB.prev;

    if (B.m_contactList === contact.edgeB)
        B.m_contactList = contact.edgeB.next;

    A.SetToAwake();
    B.SetToAwake();

    if (contact.prev)
        contact.prev.next = contact.next;

    if (contact.next)
        contact.next.prev = contact.prev;

    if (contact === this.m_contactList)
        this.m_contactList = contact.next;

    this.m_contactCount--;
}

RemoveContactsFromBody(body)
{
    let edge = body.m_contactList;

    while (edge)
    {
        let next = edge.next;
        this.RemoveContact(edge.constraint);
        edge = next;
    }
}

RemoveFromBroadphase(body)
{
    let box = body.m_boxes;

    while (box)
    {
        this.m_broadphase.RemoveBox(box);
        box = box.next;
    }
}

TestCollisions()
{
    let constraint = this.m_contactList;

    while (constraint)
    {
        let A = constraint.A;
        let B = constraint.B;
      
        let bodyA = A.body;
        let bodyB = B.body;

        constraint.m_flags &= ~q3ContactConstraint.eIsland;

        if (!bodyA.IsAwake() && !bodyB.IsAwake())
        {
            constraint = constraint.next;
            continue;
        }

        if (!bodyA.CanCollide(bodyB))
        {
            let next = constraint.next;
            this.RemoveContact(constraint);
            constraint = next;
            continue;
        }

        if (!this.m_broadphase.TestOverlap(A.broadPhaseIndex, B.broadPhaseIndex))
        {
            let next = constraint.next;
            this.RemoveContact(constraint);
            constraint = next;
            continue;
        }

        let manifold = constraint.manifold;
        let oldManifold = manifold.Clone();

        let ot0 = oldManifold.tangentVectors[0].clone();
        let ot1 = oldManifold.tangentVectors[1].clone();

        constraint.SolveCollision();
      
        q3ComputeBasis(
            manifold.normal,
            manifold.tangentVectors
        );

        for (let i = 0; i < manifold.contactCount; i++)
        {
            let c = manifold.contacts[i];

            c.tangentImpulse[0] = 0;
            c.tangentImpulse[1] = 0;
            c.normalImpulse = 0;

            let oldWarm = c.warmStarted;
            c.warmStarted = 0;

            for (let j = 0; j < oldManifold.contactCount; j++)
            {
                let oc = oldManifold.contacts[j];

                if (c.fp.key === oc.fp.key)
                {
                    c.normalImpulse = oc.normalImpulse;

                    //let friction =
                    //    ot0.scale(oc.tangentImpulse[0])
                    //    .add(ot1.scale(oc.tangentImpulse[1]));
                    let friction = q3Add(
                        q3MulScalarVec3(oc.tangentImpulse[0], ot0),
                        q3MulScalarVec3(oc.tangentImpulse[1], ot1)
                    );

                    c.tangentImpulse[0] =
                        q3Dot(friction, manifold.tangentVectors[0]);

                    c.tangentImpulse[1] =
                        q3Dot(friction, manifold.tangentVectors[1]);
                    
                    

                    c.warmStarted = Math.max(oldWarm, oldWarm + 1);

                    break;
                }
            }
        }

        if (this.m_contactListener)
        {
            let now = constraint.m_flags & q3ContactConstraint.eColliding;
            let was = constraint.m_flags & q3ContactConstraint.eWasColliding;

            if (now && !was)
                this.m_contactListener.BeginContact(constraint);

            else if (!now && was)
                this.m_contactListener.EndContact(constraint);
        }

        constraint = constraint.next;
    }
}
}