# Cable payload support in the simulator

Hanging masses are separate point bodies with gravity, drag, wake and ground/building/drone contacts.
The cable pulls only when attachment-to-ball separation exceeds its rest length. A diagonal cable can
be taut before the hub reaches an altitude equal to its length; the geometric separation is what matters.

The rigid aircraft model excludes hanging masses from mass, CoG and inertia. When “Controller knows
the static load” is checked, the simulator supplies a separate supported-weight force and attachment
moment to the flight core and supervisor. This is zero while slack, bounded by the known mass's weight,
and follows the vertical cable support as the bag is lifted. It is ideal simulated support knowledge;
no physical cable-tension sensor or firmware transport is implemented. Unknown loads remain physical
forces and receive no feedforward compensation. Detaching the bag removes its known load.

The supervisor includes that external force/moment as a fixed, unallocatable input in its hover/lift
check. Design feasibility still assesses the full carried payload: a drone can climb on a slack rope yet
be unable to lift the bag. Hardware airframe exports retain their previous fully carried static-load
assumption, since they do not have the simulator support input.

Lengthening or moving an attachment preserves an already slack ball's position/velocity. A shortened
rope limits excess separation and respects the ball's ground radius; it no longer projects a slack ball
to full rope length underground. Rope-line collisions and a physical slack-rope rendering remain outside
this change. See `tools/test_cable_slack.cjs` for the reported 1.83 m / 1.06 kg case, normal pickup,
known/unknown/detached loads, diagonal geometry, exports, edit/reset and learning/autotune guards.
