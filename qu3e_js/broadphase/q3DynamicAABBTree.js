//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree.js (1:1 JS Port)
//--------------------------------------------------------------------------------------------------
function q3Max(a, b) {
    return a > b ? a : b;
}

function q3Abs(x) {
    return Math.abs(x);
}

class q3DynamicAABBTree {
    constructor() {
        this.m_root = q3DynamicAABBTree.Node.Null;
        this.m_capacity = 1024;
        this.m_count = 0;
        this.m_nodes = new Array(this.m_capacity);
        for (let i = 0; i < this.m_capacity; i++) this.m_nodes[i] = new q3DynamicAABBTree.Node();

        this.AddToFreeList(0);
    }

    static FattenAABB(aabb) {
        const k_fattener = 0.5;
        const v = new q3Vec3(k_fattener, k_fattener, k_fattener);
        aabb.min = aabb.min.sub(v);
        aabb.max = aabb.max.add(v);
    }
  
    Insert(aabb, userData) {

        const id = this.AllocateNode();

        //console.log("Creating new node, and inserting into tree. AABB: ", aabb, "userData: ", userData, "Node ID: ", id);
        //alert("Created node");
        
        this.m_nodes[id].aabb = aabb.clone();
      
        q3DynamicAABBTree.FattenAABB(this.m_nodes[id].aabb);
      
        this.m_nodes[id].userData = userData;
        this.m_nodes[id].height = 0;

        this.InsertLeaf(id);
        return id;
    }

    Remove(id) {
        const node = this.m_nodes[id];
        if (!node.IsLeaf()) throw new Error("Remove: Node is not a leaf");
        this.RemoveLeaf(id);
        this.DeallocateNode(id);
    }

    Update(id, aabb) {
        const node = this.m_nodes[id];
        if (!node.IsLeaf()) throw new Error("Update: Node is not a leaf");

        if (node.aabb.ContainsAABB(aabb))
            return false;

        this.RemoveLeaf(id);
        node.aabb = aabb.clone();
      
        q3DynamicAABBTree.FattenAABB(node.aabb);
      
        this.InsertLeaf(id);
        return true;
    }

    GetUserData(id) { return this.m_nodes[id].userData; }
    GetFatAABB(id) { return this.m_nodes[id].aabb.clone(); }

    //--------------------------------------------------------------------------------------------------
    // Render (Debug)
    Render(render) {
        if (this.m_root !== q3DynamicAABBTree.Node.Null) {
            render.SetPenColor(0.5, 0.5, 1.0);
            this.RenderNode(render, this.m_root);
        }
    }

    RenderNode(render, index) {
        const n = this.m_nodes[index];
        const b = n.aabb;

        render.SetPenPosition(b.min.x, b.max.y, b.min.z);
        render.Line(b.min.x, b.max.y, b.max.z);
        render.Line(b.max.x, b.max.y, b.max.z);
        render.Line(b.max.x, b.max.y, b.min.z);
        render.Line(b.min.x, b.max.y, b.min.z);

        render.SetPenPosition(b.min.x, b.min.y, b.min.z);
        render.Line(b.min.x, b.min.y, b.max.z);
        render.Line(b.max.x, b.min.y, b.max.z);
        render.Line(b.max.x, b.min.y, b.min.z);
        render.Line(b.min.x, b.min.y, b.min.z);

        render.SetPenPosition(b.min.x, b.min.y, b.min.z);
        render.Line(b.min.x, b.max.y, b.min.z);
        render.SetPenPosition(b.max.x, b.min.y, b.min.z);
        render.Line(b.max.x, b.max.y, b.min.z);
        render.SetPenPosition(b.max.x, b.min.y, b.max.z);
        render.Line(b.max.x, b.max.y, b.max.z);
        render.SetPenPosition(b.min.x, b.min.y, b.max.z);
        render.Line(b.min.x, b.max.y, b.max.z);

        if (!n.IsLeaf()) {
            this.RenderNode(render, n.left);
            this.RenderNode(render, n.right);
        }
    }

        //--------------------------------------------------------------------------------------------------
    // Free list
    AddToFreeList(index) {
        for (let i = index; i < this.m_capacity - 1; i++) {
            this.m_nodes[i].next = i + 1;
            this.m_nodes[i].height = q3DynamicAABBTree.Node.Null;
        }
        this.m_nodes[this.m_capacity - 1].next = q3DynamicAABBTree.Node.Null;
        this.m_nodes[this.m_capacity - 1].height = q3DynamicAABBTree.Node.Null;
        this.m_freeList = index;
    }

    AllocateNode() {
        if (this.m_freeList === q3DynamicAABBTree.Node.Null) {
            const oldCap = this.m_capacity;
            this.m_capacity *= 2;
            for (let i = oldCap; i < this.m_capacity; i++) this.m_nodes[i] = new q3DynamicAABBTree.Node();
            this.AddToFreeList(oldCap);
        }

        const freeNode = this.m_freeList;
        const node = this.m_nodes[freeNode];
        this.m_freeList = node.next;

        node.aabb = new q3AABB(); // reset to empty AABB
        node.height = 0;
        node.left = q3DynamicAABBTree.Node.Null;
        node.right = q3DynamicAABBTree.Node.Null;
        node.parent = q3DynamicAABBTree.Node.Null;
        node.userData = null;

        this.m_count++;
        return freeNode;
    }

    DeallocateNode(index) {
        const node = this.m_nodes[index];
        node.next = this.m_freeList;
        node.height = q3DynamicAABBTree.Node.Null;
        this.m_freeList = index;
        this.m_count--;
    }


    Validate = function(){
        if (this.m_root === -1)
            return;

        this.ValidateStructure(this.m_root);
        this.ValidateMetrics(this.m_root);
    }
    
    
    
    //--------------------------------------------------------------------------------------------------
    Balance(iA) {
        const A = this.m_nodes[iA];
        if (A.IsLeaf() || A.height == 1) return iA; // <= 1?

        const iB = A.left, iC = A.right;
        const B = this.m_nodes[iB], C = this.m_nodes[iC];

        const balance = C.height - B.height;

        // Promote C
        if (balance > 1) {
            const iF = C.left, iG = C.right;
            const F = this.m_nodes[iF], G = this.m_nodes[iG];

            // Grandparent
            if (A.parent !== q3DynamicAABBTree.Node.Null) {
                const parent = this.m_nodes[A.parent];
                if (parent.left === iA) parent.left = iC; else parent.right = iC;
            } else this.m_root = iC;

            C.left = iA; C.parent = A.parent; A.parent = iC;

            if (F.height > G.height) {
                C.right = iF; A.right = iG; G.parent = iA;
                A.aabb = q3Combine(B.aabb, G.aabb);
                C.aabb = q3Combine(A.aabb, F.aabb);
                A.height = 1 + q3Max(B.height, G.height);
                C.height = 1 + q3Max(A.height, F.height);
            } else {
                C.right = iG; A.right = iF; F.parent = iA;
                A.aabb = q3Combine(B.aabb, F.aabb);
                C.aabb = q3Combine(A.aabb, G.aabb);
                A.height = 1 + q3Max(B.height, F.height);
                C.height = 1 + q3Max(A.height, G.height);
            }
            return iC;
        }

        // Promote B
        else if (balance < -1) {
            const iD = B.left, iE = B.right;
            const D = this.m_nodes[iD], E = this.m_nodes[iE];

            if (A.parent !== q3DynamicAABBTree.Node.Null) {
                const parent = this.m_nodes[A.parent];
                if (parent.left === iA) parent.left = iB; else parent.right = iB;
            } else this.m_root = iB;

            B.right = iA; B.parent = A.parent; A.parent = iB;

            if (D.height > E.height) {
                B.left = iD; A.left = iE; E.parent = iA;
                A.aabb = q3Combine(C.aabb, E.aabb);
                B.aabb = q3Combine(A.aabb, D.aabb);
                A.height = 1 + q3Max(C.height, E.height);
                B.height = 1 + q3Max(A.height, D.height);
            } else {
                B.left = iE; A.left = iD; D.parent = iA;
                A.aabb = q3Combine(C.aabb, D.aabb);
                B.aabb = q3Combine(A.aabb, E.aabb);
                A.height = 1 + q3Max(C.height, D.height);
                B.height = 1 + q3Max(A.height, E.height);
            }
            return iB;
        }

        return iA;
    }

    InsertLeaf(id) {
        if (this.m_root === q3DynamicAABBTree.Node.Null) {
            this.m_root = id;
            this.m_nodes[id].parent = q3DynamicAABBTree.Node.Null;
            return;
        }

        // Find best sibling
        let searchIndex = this.m_root;
        const leafAABB = this.m_nodes[id].aabb.clone();

        while (!this.m_nodes[searchIndex].IsLeaf()) {
            const node = this.m_nodes[searchIndex];
            const left = node.left, right = node.right;

            const combined = q3Combine(leafAABB, node.aabb);
            const combinedArea = combined.SurfaceArea();
            const branchCost = 2.0 * combinedArea;
            const inheritedCost = 2.0 * (combinedArea - node.aabb.SurfaceArea());

            const leftNode = this.m_nodes[left];
            const rightNode = this.m_nodes[right];

            const leftCost = leftNode.IsLeaf()
                ? q3Combine(leafAABB, leftNode.aabb).SurfaceArea() + inheritedCost
                : q3Combine(leafAABB, leftNode.aabb).SurfaceArea() - leftNode.aabb.SurfaceArea() + inheritedCost;

            const rightCost = rightNode.IsLeaf()
                ? q3Combine(leafAABB, rightNode.aabb).SurfaceArea() + inheritedCost
                : q3Combine(leafAABB, rightNode.aabb).SurfaceArea() - rightNode.aabb.SurfaceArea() + inheritedCost;

            if (branchCost < leftCost && branchCost < rightCost) break;
            searchIndex = leftCost < rightCost ? left : right;
        }

        const sibling = searchIndex;
        const oldParent = this.m_nodes[sibling].parent;
        const newParent = this.AllocateNode();
        const npNode = this.m_nodes[newParent];
        npNode.parent = oldParent;
        npNode.userData = null;
        npNode.aabb = q3Combine(leafAABB, this.m_nodes[sibling].aabb);
        npNode.height = this.m_nodes[sibling].height + 1;

        if (oldParent === q3DynamicAABBTree.Node.Null) {
            npNode.left = sibling; npNode.right = id;
            this.m_nodes[sibling].parent = newParent;
            this.m_nodes[id].parent = newParent;
            this.m_root = newParent;
        } else {
            const parentNode = this.m_nodes[oldParent];
            if (parentNode.left === sibling) parentNode.left = newParent; else parentNode.right = newParent;
            npNode.left = sibling; npNode.right = id;
            this.m_nodes[sibling].parent = newParent;
            this.m_nodes[id].parent = newParent;
        }

        this.SyncHierarchy(this.m_nodes[id].parent);
    }

    RemoveLeaf(id) {
        if (id === this.m_root) {
            this.m_root = q3DynamicAABBTree.Node.Null;
            return;
        }

        const parent = this.m_nodes[id].parent;
        const grandParent = this.m_nodes[parent].parent;
        const sibling = (this.m_nodes[parent].left === id) ? this.m_nodes[parent].right : this.m_nodes[parent].left;

        if (grandParent !== q3DynamicAABBTree.Node.Null) {
            const gpNode = this.m_nodes[grandParent];
            if (gpNode.left === parent) gpNode.left = sibling; else gpNode.right = sibling;
            this.m_nodes[sibling].parent = grandParent;
        } else {
            this.m_root = sibling;
            this.m_nodes[sibling].parent = q3DynamicAABBTree.Node.Null;
        }

        this.DeallocateNode(parent);
        this.SyncHierarchy(grandParent);
    }

    SyncHierarchy(index) {
        while (index !== q3DynamicAABBTree.Node.Null) {
            index = this.Balance(index);

            const node = this.m_nodes[index];
          
            // rare case check
            //if (node.left === q3DynamicAABBTree.Node.Null || node.right === q3DynamicAABBTree.Node.Null) break;
          
            const left = node.left, right = node.right;

            node.height = 1 + q3Max(this.m_nodes[left].height, this.m_nodes[right].height);
            node.aabb = q3Combine(this.m_nodes[left].aabb, this.m_nodes[right].aabb);

            index = node.parent;
        }
    }

    //--------------------------------------------------------------------------------------------------
    // Template-style queries (callback object must have TreeCallBack(id) method)
    Query(cb, aabb) {
        let hits = 0;
        
        const stack = [this.m_root];
        while (stack.length > 0) {
            const id = stack.pop();
            if (id === q3DynamicAABBTree.Node.Null) {
                continue;
            }

            const n = this.m_nodes[id];
            if (!n) {
                throw new Error("Invalid node in AABB tree: " + id);
                continue;
            }
            
            if (q3AABBtoAABB(aabb, n.aabb)) {
                if (n.IsLeaf()) {
                    hits++;
                    if (!cb.TreeCallBack(id)) return;
                } else {
                    stack.push(n.left);
                    stack.push(n.right);
                }
            }
        }
    }

    QueryRay(cb, rayCast) {
        const k_epsilon = 1e-6;
        const stack = [this.m_root];
        const p0 = rayCast.start;
        const p1 = p0.add(rayCast.dir.mul(rayCast.t));

        while (stack.length > 0) {
            const id = stack.pop();
            if (id === q3DynamicAABBTree.Node.Null) continue;
            const n = this.m_nodes[id];

            const e = n.aabb.max.sub(n.aabb.min);
            const d = p1.sub(p0);
            const m = p0.add(p1).sub(n.aabb.min).sub(n.aabb.max);

            if (q3Abs(m.x) > e.x + q3Abs(d.x)) continue;
            if (q3Abs(m.y) > e.y + q3Abs(d.y)) continue;
            if (q3Abs(m.z) > e.z + q3Abs(d.z)) continue;

            if (q3Abs(m.y*d.z - m.z*d.y) > e.y*q3Abs(d.z) + e.z*q3Abs(d.y)) continue;
            if (q3Abs(m.z*d.x - m.x*d.z) > e.x*q3Abs(d.z) + e.z*q3Abs(d.x)) continue;
            if (q3Abs(m.x*d.y - m.y*d.x) > e.x*q3Abs(d.y) + e.y*q3Abs(d.x)) continue;

            if (n.IsLeaf()) {
                if (!cb.TreeCallBack(id)) return;
            } else {
                stack.push(n.left);
                stack.push(n.right);
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
// Node structure
q3DynamicAABBTree.Node = class {
    constructor() {
        this.aabb = new q3AABB();
        this.parent = q3DynamicAABBTree.Node.Null;
        this.left = q3DynamicAABBTree.Node.Null;
        this.right = q3DynamicAABBTree.Node.Null;
        this.userData = null;
        this.height = -1;
        this.next = -1;
    }
  
    IsLeaf() {
        return this.left === q3DynamicAABBTree.Node.Null;// /* optional: */ && this.right === q3DynamicAABBTree.Node.Null;
    }
};

q3DynamicAABBTree.Node.Null = -1;

q3DynamicAABBTree.prototype.ValidateStructure = function(index)
{
    if (index === -1)
        return;

    let node = this.m_nodes[index];

    if (!node)
        return;

    if (index === this.m_root && node.parent !== -1)
        throw new Error("Tree validation failed: root has parent");

    let child1 = node.left;
    let child2 = node.right;

    if (node.IsLeaf())
    {
        if (child1 !== -1 || child2 !== -1)
            throw new Error("Tree validation failed: leaf has children");

        return;
    }

    if (child1 !== -1 && this.m_nodes[child1] && this.m_nodes[child1].parent !== index)
        throw new Error("Tree validation failed: bad parent pointer");

    if (child2 !== -1 && this.m_nodes[child2] && this.m_nodes[child2].parent !== index)
        throw new Error("Tree validation failed: bad parent pointer");

    this.ValidateStructure(child1);
    this.ValidateStructure(child2);
};


q3DynamicAABBTree.prototype.ValidateMetrics = function(index)
{
    if (index === -1)
        return 0;

    let node = this.m_nodes[index];
    if (!node)
        return 0;

    if (node.IsLeaf())
        return 0;

    let child1 = node.left;
    let child2 = node.right;

    let height1 = this.ValidateMetrics(child1);
    let height2 = this.ValidateMetrics(child2);

    let height = 1 + Math.max(height1, height2);

    if (node.height !== height)
        throw new Error("Tree validation failed: incorrect height");

    return height;
};
