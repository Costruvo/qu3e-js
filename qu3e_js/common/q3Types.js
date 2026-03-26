//--------------------------------------------------------------------------------------------------
// q3Types.js
// JavaScript port of Randy Gaul's q3Types
//--------------------------------------------------------------------------------------------------

// Floating point types
// JS only has Number (64-bit float), but naming preserved for readability
const r32 = Number;
const r64 = Number;
const f32 = Number;
const f64 = Number;

// Integer types (all JS numbers, may lose exact 32-bit int semantics for very large values)
const i8  = Number;
const i16 = Number;
const i32 = Number;
const u8  = Number;
const u16 = Number;
const u32 = Number;

// Unused macro replacement
function Q3_UNUSED(a) { /* intentionally unused */ }