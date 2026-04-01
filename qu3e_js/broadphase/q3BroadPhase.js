//--------------------------------------------------------------------------------------------------
// q3ContactPair
//--------------------------------------------------------------------------------------------------

function q3ContactPair() {
    this.A = 0;
    this.B = 0;
}

//--------------------------------------------------------------------------------------------------
// q3BroadPhase
//--------------------------------------------------------------------------------------------------

function q3BroadPhase(manager)
{
    this.m_manager = manager;

    this.m_pairCount = 0;
    this.m_pairCapacity = 64;
    this.m_pairBuffer = new Array(this.m_pairCapacity);
    for (let i = 0; i < this.m_pairCapacity; i++)
        this.m_pairBuffer[i] = new q3ContactPair();

    this.m_moveCount = 0;
    this.m_moveCapacity = 64;
    this.m_moveBuffer = new Array(this.m_moveCapacity);

    this.m_tree = new q3DynamicAABBTree();

    this.m_currentIndex = 0;
}

//--------------------------------------------------------------------------------------------------
// InsertBox
//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.InsertBox = function(box, aabb)
{
    if (box.broadPhaseIndex !== -1)
    {
        alert("Invalid box");
        return;
    }

    const id = this.m_tree.Insert(aabb, box);

    box.broadPhaseIndex = id;

    this.BufferMove(id);
};

//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.RemoveBox = function(box)
{
    this.m_tree.Remove(box.broadPhaseIndex);
    box.broadPhaseIndex = -1;
};

//--------------------------------------------------------------------------------------------------
// Sort must match C++ comparator
//--------------------------------------------------------------------------------------------------
function ContactPairSort(a, b)
{
    if (a.A < b.A) return -1;
    if (a.A > b.A) return 1;
    if (a.B < b.B) return -1;
    if (a.B > b.B) return 1;
    return 0;
}

//--------------------------------------------------------------------------------------------------
// UpdatePairs exact behavior
//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.UpdatePairs = function()
{
    this.m_pairCount = 0;

    // Query the tree with all moving boxes
    for (let i = 0; i < this.m_moveCount; ++i)
    {
        this.m_currentIndex = this.m_moveBuffer[i];

        if (this.m_currentIndex < 0)
            continue;

        const aabb = this.m_tree.GetFatAABB(this.m_currentIndex);
        
        this.m_tree.Query(this, aabb);
    }

    // Reset move buffer
    this.m_moveCount = 0;

    // Sort pairs***
    //this.m_pairBuffer.sort(ContactPairSort);
    this.m_pairBuffer
      .slice(0, this.m_pairCount)
      .sort(ContactPairSort);
    
    // ***ISSUE: JS sort function does not ignore null values.
    // BUT there shouldn't be null values in the buffer.
    // Using slice here is a temporary solution

    // Add contacts (skip duplicates)
    let i = 0;
    while (i < this.m_pairCount)
    {
        const pair = this.m_pairBuffer[i];
      

        const A = this.m_tree.GetUserData(pair.A);
        const B = this.m_tree.GetUserData(pair.B);

        if (A && B)
        {
            this.m_manager.AddContact(A, B);
        }
        
        ++i;

        // Skip duplicates
        while (i < this.m_pairCount)
        {
            const potentialDup = this.m_pairBuffer[i];

            if (pair.A !== potentialDup.A || pair.B !== potentialDup.B)
                break;

            ++i;
        }
    }

    this.m_tree.Validate();
};

//--------------------------------------------------------------------------------------------------
// Update exactly like qu3e
//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.Update = function(id, aabb)
{
    // Do not modify AABB here or fatten
    let node = this.m_tree.m_nodes[id];
  
    if (this.m_tree.Update(id, aabb))
    {
        this.BufferMove(id);
    }
  
};

//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.TestOverlap = function(A, B)
{
    return q3AABBtoAABB(
        this.m_tree.GetFatAABB(A),
        this.m_tree.GetFatAABB(B)
    );
};

//--------------------------------------------------------------------------------------------------
// BufferMove exact behavior
//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.BufferMove = function(id)
{
    if (this.m_moveCount === this.m_moveCapacity)
    {
        this.m_moveCapacity *= 2;

        const newBuffer = new Array(this.m_moveCapacity);

        for (let i = 0; i < this.m_moveCount; i++)
            newBuffer[i] = this.m_moveBuffer[i];

        this.m_moveBuffer = newBuffer;
    }

    this.m_moveBuffer[this.m_moveCount++] = id;
};

/*
q3BroadPhase.prototype.AddPair = function(A, B)
{
    const pair = {
        A: A,
        B: B
    };

    this.m_pairBuffer[this.m_pairCount++] = pair;
}
*/

//--------------------------------------------------------------------------------------------------
// Tree callback
//--------------------------------------------------------------------------------------------------

q3BroadPhase.prototype.TreeCallBack = function(index)
{
    const proxyA = this.m_currentIndex;
    const proxyB = index;

    if (proxyA === proxyB)
        return true;

    const bodyA = this.m_tree.GetUserData(proxyA);
    const bodyB = this.m_tree.GetUserData(proxyB);

    if (bodyA === bodyB)
        return true;
    
    
    if (this.m_pairCount === this.m_pairCapacity)
    {
        this.m_pairCapacity *= 2;

        const newBuffer = new Array(this.m_pairCapacity);

        for (let i = 0; i < this.m_pairCount; i++)
            newBuffer[i] = this.m_pairBuffer[i];

        for (let i = this.m_pairCount; i < this.m_pairCapacity; i++)
            newBuffer[i] = new q3ContactPair();

        this.m_pairBuffer = newBuffer;
    }

    const a = Math.min(proxyA, proxyB);
    const b = Math.max(proxyA, proxyB);

    const pair = this.m_pairBuffer[this.m_pairCount++];
    pair.A = a;
    pair.B = b;

    return true;
};

