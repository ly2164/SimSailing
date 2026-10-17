// sim-core.js - the sailing physics used by every page on this site.
//
// Single source of truth: the simulator builds and the principles page both read
// these numbers from here, so the two can never drift apart. Every constant and
// curve below is shared verbatim with the demo versions.
//
// Note on what drives what: this is a polar-by-angle model, not a lift model.
// Drive is read straight from the measured polar curve, so speed falls to zero
// inside the no-go zone because of the angle, not because of a computed angle of
// attack. The three helm inputs therefore act as: sail -> drive, rudder -> turn
// rate, centreboard -> leeway.
window.SimCore = (function () {
    'use strict';

    const DEG = 180 / Math.PI;

    // Wind comes FROM the north; screen +y points south.
    const WIND_ANGLE = 0;
    const WIND_SPEED = 1.0;
    const MAX_SAIL_ANGLE = 1.2;          // rad, max sheeting either side of centreline
    const NO_GO = 32 / DEG;              // a small dinghy cannot point closer than this
    const BEAM = Math.PI / 2;

    // Boat speed as a fraction of maximum, versus angle off the wind in degrees.
    const POLAR = [
        [  0, 0.00], [ 32, 0.00], [ 35, 0.35], [ 40, 0.52], [ 45, 0.62],
        [ 50, 0.68], [ 60, 0.78], [ 70, 0.86], [ 80, 0.94], [ 90, 1.00],
        [100, 0.97], [110, 0.93], [120, 0.88], [135, 0.82], [150, 0.76],
        [180, 0.70]
    ];

    // Sideways slip demand as a fraction of the leeway limit.
    const LEEWAY_PROFILE = [
        [  0, 0.00], [ 32, 0.00], [ 40, 0.85], [ 50, 1.00], [ 70, 0.90],
        [ 90, 0.75], [120, 0.45], [150, 0.20], [180, 0.00]
    ];

    const MAX_LEEWAY_DOWN = 12 / DEG;    // centreboard fully down
    const MAX_LEEWAY_UP = 38 / DEG;      // centreboard fully raised

    function tableLookup(table, deg) {
        if (deg <= table[0][0]) return table[0][1];
        for (let i = 0; i < table.length - 1; i++) {
            const d0 = table[i][0], s0 = table[i][1];
            const d1 = table[i + 1][0], s1 = table[i + 1][1];
            if (deg <= d1) return s0 + (s1 - s0) * (deg - d0) / (d1 - d0);
        }
        return table[table.length - 1][1];
    }

    // Signed angle between the heading and the direction the wind comes from.
    function angleOffWind(heading) {
        return Math.atan2(Math.sin(heading - WIND_ANGLE), Math.cos(heading - WIND_ANGLE));
    }

    function calcSpeed(heading) {
        const absDiff = Math.abs(angleOffWind(heading));
        if (absDiff <= NO_GO) return 0;
        return tableLookup(POLAR, absDiff * DEG);
    }

    // Sail trim (0..1) that works best at this heading.
    function optimalTrim(heading) {
        const absDiff = Math.abs(angleOffWind(heading));
        if (absDiff < BEAM) return ((absDiff - NO_GO) / (BEAM - NO_GO)) * 0.5;
        return 0.5 + ((absDiff - BEAM) / (Math.PI - BEAM)) * 0.5;
    }

    function calcSailEfficiency(heading, trim) {
        const eff = 1 - 2.5 * (trim - optimalTrim(heading)) ** 2;
        return Math.max(0.3, Math.min(1.0, eff));
    }

    function leewayDemand(heading) {
        return tableLookup(LEEWAY_PROFILE, Math.abs(angleOffWind(heading)) * DEG);
    }

    function calcLeewayAngle(heading, board) {
        const maxLeeway = MAX_LEEWAY_UP + (MAX_LEEWAY_DOWN - MAX_LEEWAY_UP) * board;
        return maxLeeway * leewayDemand(heading);
    }

    // Runs the same speed easing the simulator uses, to a settled value.
    function settleSpeed(heading, trim, frames) {
        let speed = 0;
        for (let i = 0; i < (frames || 300); i++) {
            speed += (calcSpeed(heading) * WIND_SPEED * calcSailEfficiency(heading, trim) - speed) * 0.15;
        }
        return speed;
    }

    // Everything the principles page needs for one set of helm settings.
    function analyse(heading, trim, board, rudder) {
        const twa = angleOffWind(heading);
        const twaDeg = twa * DEG;
        const twaAbs = Math.abs(twaDeg);
        const speed = settleSpeed(heading, trim);

        // The sail always sets to leeward, which is the side the wind is not.
        const sailSide = Math.sign(twa) || 1;
        const sailAngle = sailSide * trim * MAX_SAIL_ANGLE;
        const want = optimalTrim(heading);
        const efficiency = calcSailEfficiency(heading, trim);

        const leeway = calcLeewayAngle(heading, board);

        // Turn rate from the simulator's own steering term, per 60 fps frame.
        const speedForTurn = Math.min((Math.abs(speed) + 0.05) / 0.3, 1.0);
        const turnFactor = 0.4 + 0.6 * (1 - board);
        const turnRate = rudder * 0.045 * turnFactor * speedForTurn * 60 * DEG;

        // Rudder also swings the stern, which pushes the bow toward the wind.
        const rudderCrab = -speed * 0.12 * Math.sin(rudder * 1.2) * (0.5 + 0.5 * (1 - board));
        const leewaySpeed = speed * 1.8 * Math.tan(leeway);
        const lateral = Math.abs(leewaySpeed + rudderCrab);
        const forward = speed * 1.8;

        return {
            heading,
            twaDeg, twaAbs,
            tack: twaDeg >= 0 ? 'starboard' : 'port',
            inIrons: twaAbs <= NO_GO * DEG,

            speed, kn: speed * 6,
            vmgUpwindKn: speed * Math.cos(twa) * 6,
            vmgDownwindKn: -speed * Math.cos(twa) * 6,

            trim, optimalTrim: want,
            trimError: (trim - want) * 100,
            efficiency,
            sailAngleDeg: sailAngle * DEG,
            sailSide,
            idealSailAngleDeg: want * MAX_SAIL_ANGLE * DEG,

            leewayDeg: leeway * DEG,
            turnRate,

            driveFrac: forward,
            lateralFrac: lateral,
            lateralVsDrive: forward > 0 ? lateral / forward : 0
        };
    }

    return {
        DEG, WIND_ANGLE, WIND_SPEED, MAX_SAIL_ANGLE, NO_GO, BEAM,
        POLAR, LEEWAY_PROFILE, MAX_LEEWAY_DOWN, MAX_LEEWAY_UP,
        angleOffWind, tableLookup, calcSpeed, optimalTrim, calcSailEfficiency,
        leewayDemand, calcLeewayAngle, settleSpeed, analyse
    };
})();