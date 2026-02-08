import omni.ext
import omni.usd
import omni.timeline
import carb
import asyncio
import omni.kit.app
import omni.kit.commands
from pxr import Tf


USD_PATH = "/workspace/zed_streamer_warehouse.usd"


class USDAutoplayExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[USD][Autoplay] Extension started")
        asyncio.ensure_future(self._startup_sequence())
        
    async def _startup_sequence(self):
        app = omni.kit.app.get_app()

        # Wait for several update cycles to ensure the app is fully initialized
        for _ in range(30):
            await app.next_update_async()

        print("[USD][Autoplay] App initialized")

        # Get the USD context
        ctx = omni.usd.get_context()

        # Open the USD stage
        print("[USD][Autoplay] Opening stage")

        await ctx.open_stage_async(USD_PATH)

        # Wait until the stage is fully loaded
        while not ctx.get_stage():
            print("[USD][Autoplay] Loading stage...")
            await app.next_update_async()


        # Start the timeline playback
        print(f"[USD][Autoplay] Stage {USD_PATH} loaded, starting timeline")
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        for _ in range(15):
            await app.next_update_async()
        timeline.play()
        print("[USD][Autoplay] Timeline started")
        
        # TODO: Improve readiness signaling mechanism
        for _ in range(100):
            await app.next_update_async()

        print("[USD][Autoplay] Signaling Isaac core readiness")
        with open("/tmp/isaac_core_ready", "w") as f:
            f.write("ready\n")

    def on_shutdown(self):
        print("[USD][Autoplay] Extension shutdown")

    