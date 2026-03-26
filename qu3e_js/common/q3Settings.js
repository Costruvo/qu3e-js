//--------------------------------------------------------------------------------------------------
// q3Settings.js
//--------------------------------------------------------------------------------------------------

// Sleep thresholds
const Q3_SLEEP_LINEAR   = 0.01;
const Q3_SLEEP_ANGULAR  = (3.0 / 180.0) * q3PI; // radians
const Q3_SLEEP_TIME     = 0.5;

// Constraint solver constants
const Q3_BAUMGARTE       = 0.2;
const Q3_PENETRATION_SLOP = 0.05;