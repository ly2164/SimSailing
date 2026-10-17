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

    // Yaw authority per frame at full helm. Chosen so the peak rate is about
    // 6 deg/s: a dinghy under sail answers the helm in roughly 3-6 deg/s, and
    // 0.045 rad/frame (the old value) was spinning the bow through 90 deg in
    // 1.2 s, which is nothing like a real boat.
    const TURN_BASE = 0.00175;

    // Backing the sail. To come through the wind you first push the boom to
    // windward. The loaded sail then becomes a large lateral force that pivots
    // the hull even when the boat has almost no way on - which is the entire
    // reason the manoeuvre works, so this term deliberately does NOT scale with
    // speed the way helm authority does. It only bites while turning toward
    // the wind and while still near it.
    const BACKING_ZONE = 1.6;    // ~51 deg either side of the wind, so it bites from close-hauled
    const BACKING_RATE = 0.0052; // rad per frame at full helm, ~18 deg/s
    const BACKING_DRAG = 0.45;   // fraction of speed scrubbed off while backed

    // 1 while the helm is turning the boat toward the wind inside the zone.
    function isBacking(heading, rudder) {
        if (!rudder) return 0;
        const twa = angleOffWind(heading);
        if (Math.abs(twa) > BACKING_ZONE * NO_GO) return 0;
        return -Math.sign(twa) === Math.sign(rudder) ? 1 : 0;
    }

    // Yaw in rad/frame: steady helm authority plus any backing burst.
    function turnStep(heading, rudder, speed, board) {
        const speedForTurn = Math.min((Math.abs(speed) + 0.05) / 0.3, 1.0);
        const turnFactor = 0.4 + 0.6 * (1 - board);
        const helm = rudder * TURN_BASE * turnFactor * speedForTurn;
        const back = isBacking(heading, rudder);
        const backing = back ? rudder * BACKING_RATE : 0;
        return { step: helm + backing, helm, backing, back };
    }

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

    // Single-step version for live use. Chasing steady state from scratch on
    // every redraw is both wasteful and visibly steppy; stepping a carried
    // value is what the simulator itself does and stays smooth.
    function easeSpeed(current, heading, trim, frames) {
        const target = calcSpeed(heading) * WIND_SPEED * calcSailEfficiency(heading, trim);
        let s = current;
        const n = frames || 1;
        for (let i = 0; i < n; i++) s += (target - s) * 0.15;
        return s;
    }

    // Everything the explainer needs for one set of helm settings. Pass a
    // carried speed to keep it continuous across frames; omit it and the speed
    // is solved to steady state instead.
    function analyse(heading, trim, board, rudder, carriedSpeed) {
        const twa = angleOffWind(heading);
        const twaDeg = twa * DEG;
        const twaAbs = Math.abs(twaDeg);
        const back = isBacking(heading, rudder);
        // Only solve to steady state when the caller has not carried a speed;
        // doing both would put 300 iterations back on every live repaint.
        const raw = (carriedSpeed === undefined) ? settleSpeed(heading, trim) : carriedSpeed;
        const speed = raw * (1 - BACKING_DRAG * back);

        // The sail always sets to leeward, which is the side the wind is not.
        const sailSide = Math.sign(twa) || 1;
        const sailAngle = sailSide * trim * MAX_SAIL_ANGLE;
        const want = optimalTrim(heading);
        const efficiency = calcSailEfficiency(heading, trim);

        const leeway = calcLeewayAngle(heading, board);

        // Rudder side force. A rudder works by deflecting water, so its force
        // scales with the dynamic pressure, roughly v squared, not with v.
        // Modelling it as linear with v left it almost invisible at speed.
        // Swinging the helm to starboard shoves the hull to PORT. "To port" is
        // to leeward on one tack and to windward on the other, so the term has
        // to flip with the tack. Without that flip the rudder pushed the boat to
        // leeward on BOTH tacks, so with the bow swung toward the wind the track
        // line ran further downwind instead of swinging back toward it.
        const rudderCrab = -sailSide * speed * speed * 0.22 * Math.sin(rudder * 1.2) * (0.5 + 0.5 * (1 - board));
        const leewaySpeed = speed * 1.8 * Math.tan(leeway);
        const lateral = leewaySpeed + rudderCrab;
        const forward = speed * 1.8;

        // The sideways slip the hull actually has includes the rudder's crab,
        // so holding the helm over visibly bends the track. Reporting only the
        // board term made the rudder look like it did nothing at all.
        const totalLeeway = forward > 0 ? Math.atan2(Math.abs(lateral), forward) : 0;

// Yaw authority needs water flowing past the blade, so helm fades out as the
        // boat stops; the backing term is sail-driven and does not.
const t = turnStep(heading, rudder, speed, board);
        const turnRate = t.step * 60 * DEG;

        return {
            heading,
            twaDeg, twaAbs,
            // Which side the wind is coming OVER decides the tack: bow to the
            // right of the wind source means the wind is over the port side.
            // (Sailors name the tack for the windward side, not the leeward one.)
            tack: twaDeg >= 0 ? 'port' : 'starboard',
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

leewayDeg: totalLeeway * DEG,
            // Signed slip: which side of the bow the boat is actually tracking to.
            // totalLeeway is a magnitude, so it cannot express the case where a
            // hard helm drives the lateral force through zero and the track bends
            // to WINDWARD of the bow. Drawers must use this, not the magnitude.
            lateralDeg: forward > 0 ? Math.atan2(lateral, forward) * DEG : 0,
            hullLeewayDeg: leeway * DEG,
            crabDeg: Math.abs(totalLeeway * DEG - leeway * DEG),
            turnRate,

driveFrac: forward,
            lateralFrac: lateral,
            lateralVsDrive: forward > 0 ? lateral / forward : 0,
            backing: back,
            backingSpeedLoss: BACKING_DRAG * back
        };
    }

    return {
        DEG, WIND_ANGLE, WIND_SPEED, MAX_SAIL_ANGLE, NO_GO, BEAM,
        POLAR, LEEWAY_PROFILE, MAX_LEEWAY_DOWN, MAX_LEEWAY_UP,
        angleOffWind, tableLookup, calcSpeed, optimalTrim, calcSailEfficiency,
        leewayDemand, calcLeewayAngle, settleSpeed, easeSpeed, analyse,
        isBacking, turnStep, BACKING_ZONE, BACKING_RATE, BACKING_DRAG, TURN_BASE
    };
})();