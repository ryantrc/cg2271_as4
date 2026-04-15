import { devicePath, firebaseFetch } from "../firebase";

let lastLoggedEvent = null;
let lastLoggedCommandId = null;

function firebaseTimeToIso(value) {
  const ms = Number(value);
  if (!Number.isFinite(ms) || ms <= 0) return null;
  const d = new Date(ms);
  if (Number.isNaN(d.getTime())) return null;
  return d.toISOString();
}

export async function GET() {
  try {
    const [telemetry, command] = await Promise.all([
      firebaseFetch(devicePath("/telemetry")),
      firebaseFetch(devicePath("/command"))
    ]);

    const receivedAt = firebaseTimeToIso(telemetry?.updatedAtMs) || telemetry?.updatedAt || null;
    const ultrasonicDetected = Number(telemetry?.distanceCm) <= Number(process.env.PET_DISTANCE_THRESHOLD_CM || 30);
    const gyDetected = telemetry?.gyDetected === true;

    const legacyIsAround = Boolean(ultrasonicDetected || gyDetected);
    const legacyTriggerSensor = ultrasonicDetected && gyDetected
      ? "ultrasonic+gy"
      : ultrasonicDetected
        ? "ultrasonic"
        : gyDetected
          ? "gy"
          : null;

    // Log new events to Firebase history (skip pet_left and boot)
    if (telemetry?.lastEvent && telemetry.lastEvent !== lastLoggedEvent) {
      lastLoggedEvent = telemetry.lastEvent;
      const evt = telemetry.lastEvent;
      if (evt !== "pet_left" && evt !== "boot") {
        const kind = evt === "pet_arrived" ? "pet_around" : evt;
        try {
          await firebaseFetch(devicePath("/history/events"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              kind,
              sensor: triggerSensor || "device",
              message: kind,
              ts: receivedAt || new Date().toISOString()
            })
          });
        } catch (e) { /* don't block state response */ }
      }
    }

    // Log new commands to Firebase history
    if (command?.id && command.id !== lastLoggedCommandId) {
      lastLoggedCommandId = command.id;
      try {
        await firebaseFetch(devicePath("/history/commands"), {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(command)
        });
      } catch (e) { /* don't block state response */ }
    }

    const hasEspPresence = typeof telemetry?.petAround === "boolean";
    const isAround = hasEspPresence ? telemetry.petAround === true : legacyIsAround;
    const triggerSensor = telemetry?.lastTriggerSensor || legacyTriggerSensor;
    const lastSeenAt = telemetry?.lastSeenAt || (isAround ? receivedAt : null);
    const updatedAt = telemetry?.presenceUpdatedAt || receivedAt;

    return Response.json({
      ok: true,
      telemetry: telemetry
        ? {
            ...telemetry,
            receivedAt
          }
        : null,
      presence: {
        isAround,
        lastSeenAt,
        lastTriggerSensor: triggerSensor,
        updatedAt
      },
      petEvents: [],
      commands: command ? [command] : [],
      stats: {
        totalCommands: command ? 1 : 0,
        queuedCount: command?.status === "queued" ? 1 : 0
      }
    });
  } catch (error) {
    return Response.json({ ok: false, error: error.message || "Firebase state error" }, { status: 500 });
  }
}