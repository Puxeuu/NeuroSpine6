MODES = [
    {
        "id":          "fk",
        "name":        "FK",
        "subtitle":    "Joint by joint",
        "long":        "Forward Kinematics · sliders individuals",
        "accent":      "#8e8e93",
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
        "accent":      "#8e8e93",
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
        "accent":      "#8e8e93",
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
        "accent":      "#8e8e93",
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
        "accent":      "#8e8e93",
        "icon":        "☰",
        "allow_broca": False,
        "is_ready":    True,
        "hotkey":      "5",
    },
]


def get_mode(mode_id):
    for m in MODES:
        if m["id"] == mode_id:
            return m
    return None


def allow_broca(mode_id):
    m = get_mode(mode_id)
    return bool(m and m["allow_broca"])

