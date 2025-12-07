# Mini-Lab 3: Generate Synthetic Data in Isaac Sim and Use it to Train a Simple Object Detector

This mini-lab challenges you to apply the concepts of Synthetic Data Generation (SDG) and Domain Randomization (DR) using NVIDIA Isaac Sim. You will generate a dataset of synthetic images with randomized object properties and then use this data to train a simple object detection model.

## Learning Objectives

*   Utilize NVIDIA Isaac Sim's Replicator API for synthetic data generation.
*   Implement basic domain randomization techniques for visual properties.
*   Understand the workflow of generating ground truth annotations (bounding boxes).
*   Use synthetic data to train a simple object detection model (conceptual, as full training is outside this lab's scope).

## Scenario

You will create a scene in Isaac Sim with a target object (e.g., a simple cube or a USD asset). Using the Replicator API, you will randomize its position, rotation, scale, and possibly material properties. For each randomized configuration, you will capture an RGB image and its corresponding 2D bounding box annotation. This synthetic dataset will then be conceptually used to train an object detector.

## Task Breakdown

1.  **Isaac Sim Scene Setup**:
    *   Start NVIDIA Isaac Sim (5.1.0+).
    *   Open the Script Editor (Window > Scripting > Script Editor).
    *   (Optional) Load a simple environment USD from Nucleus (e.g., a simple room).

2.  **Define Target Object**:
    *   Import a simple object (e.g., `Props/Blocks/block_red.usd` from Nucleus) into your scene. Give it a semantic label (e.g., "target_object") for annotation.

3.  **Implement Domain Randomization Script**:
    *   Write a Python script (similar to `examples/module-3/domain_randomization.py`) that uses `omni.replicator.core`:
        *   Instantiate the target object.
        *   Apply randomizers for:
            *   **Position**: Scatter the object randomly within a defined volume.
            *   **Rotation**: Randomize its orientation (full 360 degrees).
            *   **Scale**: Randomize its size within a reasonable range.
            *   (Optional but Recommended) **Texture/Material**: Randomize the texture or material of the object and/or background.
        *   Randomize lighting conditions (e.g., `DomeLight` intensity and temperature).

4.  **Set Up Annotators**:
    *   Configure `omni.replicator.core.writers.BasicWriter` to capture:
        *   `rgb`: The synthetic image.
        *   `bounding_box_2d_tight`: 2D bounding box annotations for the target object.
        *   (Optional) `instance_segmentation`: If you want pixel-level masks.

5.  **Generate Data**:
    *   Run the script to generate a specified number of synthetic images (e.g., 100-500) with corresponding annotations. Save them to a structured output directory.

## Conceptual Object Detector Training Workflow

While a full object detector training implementation is beyond the scope of this mini-lab, the generated data would be used as follows:

1.  **Data Preprocessing**: Convert the generated synthetic images and annotations into a format compatible with your chosen object detection framework (e.g., YOLO, Detectron2, TensorFlow Object Detection API).
2.  **Model Selection**: Choose a suitable lightweight object detection model.
3.  **Training**: Train the model on your synthetic dataset.
4.  **Evaluation**: Evaluate the model's performance on a held-out set of synthetic data and, critically, on a small set of real-world images of the same object to assess the sim-to-real transfer.

## Execution and Verification

1.  **Run Isaac Sim**: Start Isaac Sim.
2.  **Execute Script**: Run your Python script within Isaac Sim's Script Editor.
3.  **Inspect Output**: Verify that images and annotation files are generated in your specified output directory.
4.  **Conceptual Training**: Discuss how this data would be used for training, considering the advantages of perfect ground truth and varied data.

## Self-Reflection Questions

*   How does increasing the diversity of randomizations affect the sim-to-real gap?
*   What are the limitations of training solely on synthetic data, even with extensive domain randomization?
*   How would you integrate real-world images into your training process alongside synthetic data?

This mini-lab provides a foundational understanding of how synthetic data generation and domain randomization can accelerate the development of AI perception models for robotics.