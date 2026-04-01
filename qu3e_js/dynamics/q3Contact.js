//--------------------------------------------------------------------------------------------------
// q3Contact.js
// Direct JS port of qu3e q3Contact.h / q3Contact.cpp
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Mixing Functions
//--------------------------------------------------------------------------------------------------

function q3MixRestitution(A, B)
{
    return Math.max(A.restitution, B.restitution);
}

function q3MixFriction(A, B)
{
    return Math.sqrt(A.friction * B.friction);
}


//--------------------------------------------------------------------------------------------------
// q3FeaturePair
//--------------------------------------------------------------------------------------------------

class q3FeaturePair
{
    constructor(_inR = 0, _outR = 0, _inI = 0, _outI = 0)
    {
        this.inR = _inR;
        this.outR = _outR;
        this.inI = _inI;
        this.outI = _outI;
    }

    getKey()
    {
        return (
            (this.inR  << 0)  |
            (this.outR << 8)  |
            (this.inI  << 16) |
            (this.outI << 24)
        );
    }

    setKey(key)
    {
        this.inR  = (key      ) & 0xFF;
        this.outR = (key >> 8 ) & 0xFF;
        this.inI  = (key >> 16) & 0xFF;
        this.outI = (key >> 24) & 0xFF;
    }
}


//--------------------------------------------------------------------------------------------------
// q3Contact
//--------------------------------------------------------------------------------------------------

class q3Contact
{
    constructor()
    {
        this.position = new q3Vec3();
        this.penetration = 0;

        this.normalImpulse = 0;

        this.tangentImpulse = [0,0];

        this.bias = 0;

        this.normalMass = 0;

        this.tangentMass = [0,0];

        this.fp = new q3FeaturePair();

        this.warmStarted = 0;
    }
  
    Clone = function()
    {
        const copy = new q3Contact();

        copy.position = this.position.clone();          // q3Vec3
        copy.penetration = this.penetration;
        copy.normalImpulse = this.normalImpulse;
        copy.tangentImpulse = [this.tangentImpulse[0], this.tangentImpulse[1]];
        copy.bias = this.bias;
        copy.normalMass = this.normalMass;
        copy.tangentMass = [this.tangentMass[0], this.tangentMass[1]];
        copy.fp = new q3FeaturePair(this.fp.inR, this.fp.outR, this.fp.inI, this.fp.outI);
        copy.warmStarted = this.warmStarted;

        return copy;
    };
}


//--------------------------------------------------------------------------------------------------
// q3Manifold
//--------------------------------------------------------------------------------------------------

class q3Manifold
{
    constructor()
    {
        this.A = null;
        this.B = null;

        this.normal = new q3Vec3();

        this.tangentVectors = [
            new q3Vec3(),
            new q3Vec3()
        ];

        this.contacts = new Array(8);
        for (let i = 0; i < 8; i++)
            this.contacts[i] = new q3Contact();

        this.contactCount = 0;

        this.next = null;
        this.prev = null;

        this.sensor = false;
    }

    SetPair(a, b)
    {
        this.A = a;
        this.B = b;

        this.sensor = a.sensor || b.sensor;
    }
  
    Clone() {
        const copy = new q3Manifold();

        copy.A = this.A;
        copy.B = this.B;

        copy.normal = this.normal.clone();

        copy.tangentVectors[0] = this.tangentVectors[0].clone();
        copy.tangentVectors[1] = this.tangentVectors[1].clone();

        copy.contactCount = this.contactCount;
        copy.sensor = this.sensor;

        copy.next = null;
        copy.prev = null;

        for (let i = 0; i < this.contacts.length; i++) {
            copy.contacts[i] = this.contacts[i].Clone();
        }
        
        return copy;
    }
}


//--------------------------------------------------------------------------------------------------
// q3ContactEdge
//--------------------------------------------------------------------------------------------------

class q3ContactEdge
{
    constructor()
    {
        this.other = null;
        this.constraint = null;

        this.next = null;
        this.prev = null;
    }
}


//--------------------------------------------------------------------------------------------------
// q3ContactConstraint
//--------------------------------------------------------------------------------------------------

class q3ContactConstraint
{
    constructor()
    {
        this.A = null;
        this.B = null;

        this.bodyA = null;
        this.bodyB = null;

        this.edgeA = new q3ContactEdge();
        this.edgeB = new q3ContactEdge();

        this.next = null;
        this.prev = null;

        this.friction = 0;
        this.restitution = 0;

        this.manifold = new q3Manifold();

        this.m_flags = 0;
    }

    SolveCollision()
    {
        this.manifold.contactCount = 0;

        q3BoxtoBox(this.manifold, this.A, this.B);

        if (this.manifold.contactCount > 0)
        {
            if (this.m_flags & q3ContactConstraint.eColliding)
                this.m_flags |= q3ContactConstraint.eWasColliding;
            else
                this.m_flags |= q3ContactConstraint.eColliding;
        }
        else
        {
            if (this.m_flags & q3ContactConstraint.eColliding)
            {
                this.m_flags &= ~q3ContactConstraint.eColliding;
                this.m_flags |= q3ContactConstraint.eWasColliding;
            }
            else
            {
                this.m_flags &= ~q3ContactConstraint.eWasColliding;
            }
        }
    }
}


//--------------------------------------------------------------------------------------------------
// Flags
//--------------------------------------------------------------------------------------------------

q3ContactConstraint.eColliding    = 0x00000001;
q3ContactConstraint.eWasColliding = 0x00000002;
q3ContactConstraint.eIsland       = 0x00000004;