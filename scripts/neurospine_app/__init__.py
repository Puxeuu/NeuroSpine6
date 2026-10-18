"""
NeuroSpine Navigator — aplicació unificada.

Aquesta és l'app que integra tot el sistema NeuroSpine6 en una sola
finestra amb 3 workspaces:

    · Planning     — planificació de cargols pediculars sobre TAC
    · Robot        — control i visualització del braç robòtic
    · Navigation   — vista quirúrgica fusionada (registre TAC↔robot)

Estructura:
    theme.py             Paleta i estils compartits (Apple blue)
    main_window.py       QMainWindow principal amb header + sidebar
    header.py            Barra superior amb indicadors d'estat
    sidebar.py           Selector de workspaces
    mando_service.py     Servei únic de lectura de l'Arduino (compartit)
    workspaces/          Widgets de cada workspace
"""
