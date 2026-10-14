# Odometry template
This is currently a template for odometry and motion control for VEX V5 using [vexide](https://vexide.dev). This may be repurposed for actual bot code later.

A development decision made is to perform most operations within units. This is intended, and certain APIs (e.g. the Translation and Pose structs) have been designed this way to reflect WPILib's units and mathematics libraries. For an explanation of the rationale, see [WPILib's documentation](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).
