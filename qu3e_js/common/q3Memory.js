//--------------------------------------------------------------------------------------------------
// q3Memory.js (Safe & Robust JS Port)
// Everything in this file is unused.
// JS takes care of all this. Still ported for the sake of it.
//--------------------------------------------------------------------------------------------------

//-----------------------------
// Type aliases (for clarity)
//-----------------------------
/*
const r32 = Number;
const r64 = Number;
const f32 = Number;
const f64 = Number;
const i8  = Number;
const i16 = Number;
const i32 = Number;
const u8  = Number;
const u16 = Number;
const u32 = Number;
*/



//-----------------------------
// Q3_UNUSED helper
//-----------------------------
function Q3_UNUSED(a) { return a; }

//--------------------------------------------------------------------------------------------------
// q3Alloc / q3Free
//--------------------------------------------------------------------------------------------------
function q3Alloc(size) {
    if (size <= 0) throw new Error("q3Alloc: size must be positive");
    return new ArrayBuffer(size);
}

function q3Free(ptr) {
    // GC handles memory in JS
    Q3_UNUSED(ptr);
}

// Pointer arithmetic helper (safe)
function Q3_PTR_ADD(ptr, bytes) {
    const buffer = ptr instanceof DataView ? ptr.buffer : ptr;
    const offset = (ptr.byteOffset || 0) + bytes;
    if (offset < 0 || offset > buffer.byteLength) {
        throw new Error("Q3_PTR_ADD: offset out of bounds");
    }
    return new DataView(buffer, offset, buffer.byteLength - offset);
}

//--------------------------------------------------------------------------------------------------
// q3Heap (simple heap allocator)
//--------------------------------------------------------------------------------------------------
class q3Heap {
    constructor(heapSize = 1024*1024*20, initialCapacity = 1024) {
        if (heapSize <= 0) throw new Error("q3Heap: heapSize must be positive");

        this.m_memory = new ArrayBuffer(heapSize);
        this.m_freeBlocks = [{ offset: 0, size: heapSize }];
        this.m_freeBlockCapacity = initialCapacity;
    }

    allocate(size) {
        if (size <= 0) return null;

        let blockIndex = -1;
        for (let i = 0; i < this.m_freeBlocks.length; i++) {
            if (this.m_freeBlocks[i].size >= size) {
                blockIndex = i;
                break;
            }
        }

        if (blockIndex === -1) return null;

        const block = this.m_freeBlocks[blockIndex];
        const offset = block.offset;

        if (block.size === size) {
            this.m_freeBlocks.splice(blockIndex, 1);
        } else {
            block.offset += size;
            block.size -= size;
        }

        return new DataView(this.m_memory, offset, size);
    }

    free(dataView) {
        const offset = dataView.byteOffset;
        const size = dataView.byteLength;

        let inserted = false;
        for (let i = 0; i < this.m_freeBlocks.length; i++) {
            const block = this.m_freeBlocks[i];

            if (offset + size === block.offset) {
                block.offset = offset;
                block.size += size;
                inserted = true;
                break;
            } else if (block.offset + block.size === offset) {
                block.size += size;
                if (i + 1 < this.m_freeBlocks.length) {
                    const next = this.m_freeBlocks[i + 1];
                    if (block.offset + block.size === next.offset) {
                        block.size += next.size;
                        this.m_freeBlocks.splice(i + 1, 1);
                    }
                }
                inserted = true;
                break;
            } else if (offset < block.offset) {
                this.m_freeBlocks.splice(i, 0, { offset, size });
                inserted = true;
                break;
            }
        }

        if (!inserted) this.m_freeBlocks.push({ offset, size });

        // Sort for next-fit allocation
        this.m_freeBlocks.sort((a,b) => a.offset - b.offset);
    }
}

//--------------------------------------------------------------------------------------------------
// q3Stack (stack allocator)
//--------------------------------------------------------------------------------------------------
class q3Stack {
    constructor() {
        this.m_memory = null;
        this.m_entries = [];
        this.m_index = 0;
        this.m_allocation = 0;
        this.m_stackSize = 0;
    }

    reserve(size) {
        if (size <= 0) return;
        if (!this.m_memory || size > this.m_stackSize) {
            this.m_memory = q3Alloc(size);
            this.m_stackSize = size;
            this.m_index = 0;
            this.m_entries = [];
        }
    }

    allocate(size) {
        if (size <= 0) throw new Error("q3Stack.allocate: size must be positive");
        if (this.m_index + size > this.m_stackSize) {
            throw new Error("q3Stack overflow");
        }
        const data = new DataView(this.m_memory, this.m_index, size);
        this.m_index += size;
        this.m_entries.push({ data, size });
        this.m_allocation += size;
        return data;
    }

    free(data) {
        const entry = this.m_entries.pop();
        if (!entry || entry.data !== data) {
            throw new Error("q3Stack free must be in reverse order");
        }
        this.m_index -= entry.size;
        this.m_allocation -= entry.size;
    }
}

//--------------------------------------------------------------------------------------------------
// q3PagedAllocator (fixed-size object pool)
//--------------------------------------------------------------------------------------------------
class q3PagedAllocator {
    constructor(elementSize, elementsPerPage) {
        if (elementSize <= 0 || elementsPerPage <= 0) {
            throw new Error("q3PagedAllocator: invalid element size or page count");
        }

        this.m_blockSize = elementSize;
        this.m_blocksPerPage = elementsPerPage;
        this.m_pages = [];
        this.m_freeList = [];
    }

    allocate() {
        if (this.m_freeList.length > 0) return this.m_freeList.pop();

        const page = [];
        for (let i = 0; i < this.m_blocksPerPage; i++)
            page.push({});
      
        this.m_pages.push(page);

        // Add all but first block to free list
        for (let i = 1; i < page.length; i++) this.m_freeList.push(page[i]);
        return page[0];
    }

    free(block) {
        if (!block) throw new Error("q3PagedAllocator free: block is null");
        this.m_freeList.push(block);
    }

    clear() {
        this.m_pages = [];
        this.m_freeList = [];
    }
}

//--------------------------------------------------------------------------------------------------
// Optional helper: zero out a DataView
//--------------------------------------------------------------------------------------------------
function q3ZeroMemory(dataView) {
    for (let i = 0; i < dataView.byteLength; i++) dataView.setUint8(i, 0);
}