"""
Definició declarativa dels 5 modes de control.

Cada mode té:
    id          Identificador intern
    name        Etiqueta curta al selector
    subtitle    Descripció d'una línia
    accent      Color d'accent principal (línia lateral i icona)
    icon        Caràcter Unicode per la icona (geomètric i limpiós)
    allow_broca True si la broca pot funcionar en aquest mode
    is_ready    True si el mode està disponible al viewer
    hotkey      Tecla ràpida (opcional)
"""

MODES = [                                                                       # assignem MODES
    {
        "id":          "fk",
        "name":        "FK",
        "subtitle":    "Joint by joint",
        "long":        "Forward Kinematics · sliders individuals",
        "accent":      "#8e8e93",     # gris
        "icon":        "☰",
        "allow_broca": False,
        "is_ready":    True,
        "hotkey":      "1",
    },
    {
        "id":          "ik6",
        "name":        "IK 6DOF",
        "subtitle":    "Full inverse",
        "long":        "Inversa completa 6 graus · TCP orientable",
        "accent":      "#8e8e93",     # gris
        "icon":        "☰",
        "allow_broca": False,
        "is_ready":    True,
        "hotkey":      "2",
    },
    {
        "id":          "ik5",
        "name":        "IK 5DOF",
        "subtitle":    "Drill ready",
        "long":        "Inversa 5 graus · J6 lliure per la broca",
        "accent":      "#8e8e93",     # gris
        "icon":        "☰",
        "allow_broca": True,
        "is_ready":    True,
        "hotkey":      "3",
    },
    {
        "id":          "mando",
        "name":        "MANDO",
        "subtitle":    "Haptic control",
        "long":        "Control amb l'Arduino 6-DOF",
        "accent":      "#8e8e93",     # gris
        "icon":        "☰",
        "allow_broca": False,
        "is_ready":    True,
        "hotkey":      "4",
    },
    {
        "id":          "handguide",
        "name":        "HAND-GUIDE",
        "subtitle":    "Guided by touch",
        "long":        "Guiat manual per força (J1·J2·J3)",
        "accent":      "#8e8e93",     # gris
        "icon":        "☰",
        "allow_broca": False,
        "is_ready":    True,
        "hotkey":      "5",
    },
]


def get_mode(mode_id):                                                          # definim la funció get_mode()
    """Retorna el dict del mode donat el seu id, o None si no existeix."""      # """Retorna el dict del mode donat el seu id, o None si no...
    for m in MODES:                                                             # iterem: m in MODES
        if m["id"] == mode_id:                                                  # si m["id"] == mode_id
            return m                                                            # retornem: m
    return None                                                                 # retornem: None


def allow_broca(mode_id):                                                       # definim la funció allow_broca()
    """True si el mode permet activar la broca simultàniament."""               # """True si el mode permet activar la broca simultàniament...
    m = get_mode(mode_id)                                                       # m ← resultat de get_mode()
    return bool(m and m["allow_broca"])                                         # retornem: bool(m and m["allow_broca"])
