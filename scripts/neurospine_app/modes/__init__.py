"""
Modes de control del cobot.

L'app suporta 5 modes cinemàtics, tots mutuament exclusius. En qualsevol
moment només un mode està actiu:

    FK          Forward Kinematics — sliders per cada joint
    IK6         Inverse Kinematics complet 6 DOF (incloent J6)
    IK5         Inverse Kinematics 5 DOF (J1-J5), J6 lliure per la broca
    MANDO       Control amb l'Arduino 6-DOF
    HANDGUIDE   Guiat manual per força

El motor J6 (broca) és un actuador independent que només és utilitzable
en el mode IK5 (posiciones el braç amb 5 graus, la broca gira lliure).

Fitxers:
    modes_config.py    Definició de metadades dels modes (nom, color, icona)
    mode_selector.py   Widget visual per triar-los
"""
