import omni.usd
import omni.isaac.core as ic
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Initialize Isaac Sim
kit = ic.World()
kit.scene.add_default_ground_plane()

# Create a prim (e.g., a cube)
# cube = kit.scene.add(ic.Cube(prim_path="/World/Cube", position=ic.Vec3(0, 0, 0.5)))

# Setup camera and synthetic data helper
# camera = kit.scene.add_camera(prim_path="/World/Camera", position=ic.Vec3(1, 1, 1), target=ic.Vec3(0,0,0))
# sd_helper = SyntheticDataHelper()
# sd_helper.initialize(sensor_names=["rgb", "depth", "semantic_segmentation"])

# Define a custom render product for data capture
# render_product = sd_helper.create_or_get_render_product(camera_prim_path=camera.prim_path, resolution=(1024, 1024))

# To generate data in a loop:
# for i in range(100):
#     kit.step(render=True)
#     sd_helper.get_ground_truth(["rgb", "depth", "semantic_segmentation"], render_product)
#     # Move objects, change lighting, etc., programmatically

# kit.run_simulation() # For interactive simulation

# kit.clear()
# kit.shutdown()
