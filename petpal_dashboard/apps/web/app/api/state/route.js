import { devicePath, firebaseFetch } from "../firebase";

export async function GET() {
  try {
    const [telemetry, command] = await Promise.all([
      firebaseFetch(devicePath("/telemetry")),
      firebaseFetch(devicePath("/command"))
    ]);

    const ultrasonicDetected = Number(telemetry?.distanceCm) <= Number(process.env.PET_DISTANCE_THRESHOLD_CM || 30);
    const shockDetected = telemetry?.shockDetected === true;
    const isAround = Boolean(ultrasonicDetected || shockDetected);
    const triggerSensor = ultrasonicDetected && shockDetected
      ? "ultrasonic+shock"
      : ultrasonicDetected
        ? "ultrasonic"
        : shockDetected
          ? "shock"
          : null;

    return Response.json({
      ok: true,
      telemetry: telemetry
        ? {
            ...telemetry,
            receivedAt: telemetry.updatedAt
          }
        : null,
      presence: {
        isAround,
        lastSeenAt: isAround ? telemetry?.updatedAt : null,
        lastTriggerSensor: triggerSensor,
        updatedAt: telemetry?.updatedAt || null
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
