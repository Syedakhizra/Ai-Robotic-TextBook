# Example: Domain Randomization for Object Pose Estimation

This example demonstrates how to use NVIDIA Isaac Sim's Replicator API to perform domain randomization, generating diverse synthetic datasets for training an object pose estimation model. The goal is to make the trained model robust to variations it might encounter in the real world by exposing it to a wide range of randomized scene parameters during training.

## Goal

Generate synthetic images with randomized object poses, textures, and lighting, along with ground truth annotations (e.g., 2D bounding boxes, instance segmentation masks, 6D pose), to train an object pose estimation model.

## Prerequisites

*   **NVIDIA Isaac Sim**: Installed and configured (version 5.1.0+ recommended).
*   **Isaac Sim Replicator API**: Familiarity with its basic usage.
*   **ROS 2 Humble/Iron**: Installed and sourced (for potential downstream use of data).

## Example Files

This example will primarily consist of a Python script designed to be run within the Isaac Sim environment or as an Omniverse Kit application.

### `domain_randomization.py` (Python Script for Isaac Sim)

```python
import omni.replicator.core as rep
import omni.isaac.core.utils.nucleus as nucleus_utils
from pxr import Gf, UsdLux, Sdf

# -------------------- Configuration Parameters --------------------
ASSET_PATH = nucleus_utils.get_nucleus_assets_path() + "/Props/Blocks/block_red.usd" # Example asset
OUTPUT_DIR = "output/synthetic_data/object_pose_estimation"
NUM_FRAMES = 100 # Number of synthetic images to generate
# ------------------------------------------------------------------

def create_scene_and_randomizers():
    # Initialize Replicator
    rep.initialize()

    # Get the current stage (scene)
    stage = omni.usd.get_context().get_stage()

    # 1. Setup Camera and Renderer
    # Create a camera and attach it to the current view
    camera = rep.create.camera(position=Gf.Vec3d(1, 1, 1), look_at=Gf.Vec3d(0, 0, 0))
    render_product = rep.create.render_product(camera, (1024, 1024))

    # 2. Load Environment (e.g., a simple lab or warehouse scene)
    # Example: Load a simple ground plane
    # rep.create.plane(scale=Gf.Vec3d(10, 10, 1), position=Gf.Vec3d(0, 0, 0))

    # 3. Create objects to randomize
    # Load a prim (e.g., a block) that we want to estimate the pose of
    # We will instance it multiple times if desired or just randomize a single instance
    with rep.create.prims(num_prims=1, prim_paths="/World/TargetObject_#", semantic_labels=["target_object"]) as target_objects:
        rep.create.from_usd(ASSET_PATH, rep_prims=target_objects)

        # 4. Apply Domain Randomization
        with target_objects:
            # Randomize position (within a certain volume)
            rep.randomizer.scatter_3d(
                min_pos=Gf.Vec3d(-0.5, -0.5, 0.1),
                max_pos=Gf.Vec3d(0.5, 0.5, 0.5),
                check_for_collisions=True # Avoid object overlap
            )
            # Randomize rotation (full 360 degrees on all axes)
            rep.randomizer.rotation(
                min_rot=Gf.Vec3d(0, 0, 0),
                max_rot=Gf.Vec3d(360, 360, 360)
            )
            # Randomize scale (e.g., +/- 20%)
            rep.randomizer.scale(
                min_scale=Gf.Vec3d(0.8, 0.8, 0.8),
                max_scale=Gf.Vec3d(1.2, 1.2, 1.2)
            )
            # Randomize material (e.g., different textures or colors)
            # This would typically involve creating a pool of materials and sampling from them
            # rep.randomizer.material(materials_pool)

    # 5. Randomize Lighting
    with rep.create.light(
        light_type="DomeLight",
        intensity=rep.distribution.uniform(1000, 5000),
        temperature=rep.distribution.uniform(2000, 8000),
        color=rep.distribution.uniform(Gf.Vec3f(0.8, 0.8, 0.8), Gf.Vec3f(1.0, 1.0, 1.0))
    ):
        pass

    # 6. Setup Annotators and Writer
    writer = rep.writers.BasicWriter(output_dir=OUTPUT_DIR,
                                     rgb=True,
                                     bounding_box_2d_tight=True,
                                     bounding_box_3d=True,
                                     instance_segmentation=True,
                                     # Add other annotators like 6D pose, depth, etc.
                                    )
    rep.orchestrator.set_writer(writer)

    # Trigger randomization and data capture for each frame
    rep.orchestrator.step(NUM_FRAMES, rt_subframes=4) # rt_subframes steps per randomization
    
    print(f"Generated {NUM_FRAMES} synthetic data frames to {OUTPUT_DIR}")

def main():
    try:
        create_scene_and_randomizers()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rep.shutdown()

if __name__ == "__main__":
    main()
```

## Running the Example (within Isaac Sim)

1.  **Launch Isaac Sim**: Start Isaac Sim from the Omniverse Launcher.
2.  **Open Script Editor**: Go to Window > Scripting > Script Editor.
3.  **Load and Run**: Copy the content of `domain_randomization.py` into the script editor and click the "Run" button (or save as a Python extension and enable it).
4.  **Observe**: The script will generate a series of randomized images and annotations (e.g., 2D bounding boxes, 3D bounding boxes, instance segmentation) into the specified `OUTPUT_DIR`.

This example provides a foundation for creating high-quality synthetic datasets that are critical for training robust object pose estimation models in AI robotics. By diversifying the training data, you can significantly improve your model's ability to generalize to real-world conditions.