import { devicePath, firebaseFetch } from "../firebase";

export async function POST(request) {
  try {
    const body = await request.json();
    const now = new Date().toISOString();
    const command = {
      id: String(Date.now()),
      type: body.type,
      source: "web-dashboard",
      status: "queued",
      createdAt: now,
      fetchedAt: null,
      executedAt: null
    };

    await firebaseFetch(devicePath("/command"), {
      method: "PUT",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(command)
    });

    return Response.json({ ok: true, command }, { status: 201 });
  } catch (error) {
    return Response.json({ ok: false, error: error.message || "Firebase command error" }, { status: 500 });
  }
}
