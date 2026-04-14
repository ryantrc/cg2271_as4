import { devicePath, firebaseFetch } from "../firebase";

export async function GET() {
  try {
    const [telemetry, command] = await Promise.all([
      firebaseFetch(devicePath("/telemetry")),
      firebaseFetch(devicePath("/command"))
    ]);

    const petEvents = telemetry?.lastEvent
      ? [{
          id: telemetry.updatedAt || "latest",
          kind: telemetry.lastEvent,
          sensor: "device",
          message: telemetry.lastEvent,
          ts: telemetry.updatedAt || null
        }]
      : [];

    return Response.json({
      ok: true,
      petEvents,
      commands: command ? [command] : []
    });
  } catch (error) {
    return Response.json({ ok: false, error: error.message || "Firebase history error" }, { status: 500 });
  }
}
