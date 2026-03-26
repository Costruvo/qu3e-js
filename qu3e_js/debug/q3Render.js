//--------------------------------------------------------------------------------------------------
// debug/q3Render.js
// JS port of qu3e's abstract q3Render interface
//--------------------------------------------------------------------------------------------------

class q3Render {
    // Sets pen color
    SetPenColor(r, g, b, a = 1.0) {
        throw new Error("SetPenColor must be implemented by subclass");
    }

    // Sets pen position
    SetPenPosition(x, y, z) {
        throw new Error("SetPenPosition must be implemented by subclass");
    }

    // Sets scale for subsequent Point calls
    SetScale(sx, sy, sz) {
        throw new Error("SetScale must be implemented by subclass");
    }

    // Draw line from current pen position to (x,y,z)
    // Moves pen to new position
    Line(x, y, z) {
        throw new Error("Line must be implemented by subclass");
    }

    // Sets triangle normal for the next Triangle call
    SetTriNormal(x, y, z) {
        throw new Error("SetTriNormal must be implemented by subclass");
    }

    // Draw triangle using the current normal
    Triangle(x1, y1, z1, x2, y2, z2, x3, y3, z3) {
        throw new Error("Triangle must be implemented by subclass");
    }

    // Draw a point at current pen position using current scale
    Point() {
        throw new Error("Point must be implemented by subclass");
    }
}

