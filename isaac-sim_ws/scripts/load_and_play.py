from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": True
})

import omni.usd
import omni.timeline

USD_PATH = "/workspace/zed_streamer_warehouse.usd"

# Abre a cena
omni.usd.get_context().open_stage(USD_PATH)

# Aguarda carregar
omni.usd.get_context().get_stage_event_stream().wait()

# Dá play
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print(f"[ISAAC] Scene loaded and playing: {USD_PATH}")

# Mantém a sim rodando
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
