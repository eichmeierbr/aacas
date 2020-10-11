Safety Test Instructions:

Files changed:
aacas_motion/config/vector_field_planner_params.yaml
    - Add Safety Parameters:
        x_constraint: [0.0, 20.0]
        y_constraint: [-10.0, 10.0]

    These initially include the entire workspace, change values [min, max] to constrain safety flying area.
    Added to vectFieldController class attributes.

aacas_motion/src/vectFieldController_multiObject.py
    - Function safetyCheck (L354):
        Check if currenty x, y position and velocity exceeds the constraints.
        Use vectFieldController.v_max as velocity constraint.
        If flying space is not safe, set field.is_safe to False.

    - Function hoverInPlace (L371):
        If commanded, hover in the same position for 10 seconds.

    - Function rush (L380):
        Test for the velocity check.
        Commands the drone to accelerate to 1.5*v_max so that it violates safety constraints.
        Uncomment L419 (field.rush()) and comment L418 (field.move()) to test.

    - main:
        Add safety process so that when safety conditions violated, it first hovers for 10 seconds and then lands.