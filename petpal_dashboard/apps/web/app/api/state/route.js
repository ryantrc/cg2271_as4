import { devicePath, firebaseFetch } from "../firebase";

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
