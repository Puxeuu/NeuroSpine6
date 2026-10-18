"""
Panells contextuals per cada mode del workspace del Robot.

Cada mode té el seu conjunt de widgets a la columna dreta:
    FK        · sliders de joints (moure joint a joint)
    IK 6DOF   · sliders cartesians (X, Y, Z, Roll, Pitch, Yaw)
    IK 5DOF   · sliders cartesians + panell broca
    MANDO     · estat del mando + botó de control
    HANDGUIDE · placeholder (fase B)

Els panells reutilitzen widgets del viewer_cobot on és possible per no
haver de duplicar lògica.
"""
