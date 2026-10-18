"""
Detecció de col·lisions del robot amb ell mateix.

Fem servir FCL per comprovar totes les parelles de primitives, saltant:
    - Primitives del mateix link (són el mateix cos rígid).
    - Links adjacents (sempre estan enganxats, no és una col·lisió real).

Dues versions:
    check_collisions               -> detecta col·lisió estricta (contacte)
    check_collisions_with_margin   -> detecta si dues primitives estan
                                       a menys de 'margin' metres entre elles

Retornen (bool, llista_de_parells).
"""

import fcl
from src.kinematics.forward import forward_kinematics


# ==============================================================================
# ADJACENT_PAIRS: parelles de links que no hem de tenir en compte perquè estan sempre en contacte per dir algo. 
# ==============================================================================

ADJACENT_PAIRS = {
    ("world", "base_link"),
    ("base_link", "arm1"),
    ("arm1", "arm2"),
    ("arm2", "arm3"),
    ("arm3", "arm4"),
    ("arm4", "arm5"),
    ("arm5", "capcal"),
    ("capcal", "tcp_link"),
}


# ==============================================================================
# CHECK_COLLISIONS: Col·lisió estricta on dues primitives es toquen o s'interpenetren.
# ==============================================================================

def check_collisions(robot, q, prims_by_link):

    poses = forward_kinematics(robot, q)

    world_objects = []
    for link_name, prims in prims_by_link.items():                              # Itera cada link i la seva llista de primitives.
        if link_name not in poses:
            continue
        T_link = poses[link_name]                                               # Agafa la pose actual del link al món (matriu 4×4).
        for p in prims:                                                         # Itera cada primitiva del link.
            T_world = T_link @ p.T_local
            tf = fcl.Transform(T_world[:3, :3], T_world[:3, 3])                 # Converteix la matriu 4×4 al format que FCL entén: rotació 3×3 + translació 3D separades.
            obj = fcl.CollisionObject(p.fcl_object, tf)                         # Crea l'objecte FCL de col·lisió combinant: la geometria (p.fcl_object: cilindre, caixa, esfera) + la pose (tf).
            world_objects.append((link_name, obj))



    collisions = []
    n = len(world_objects)
    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()

    for i in range(n):
        link_i, obj_i = world_objects[i]                                        # Doble bucle que recorre totes les parelles sense repetir.
        for j in range(i + 1, n):
            link_j, obj_j = world_objects[j]

            if link_i == link_j:                                                # Salta col·lisions entre primitives del MATEIX link.
                continue
            if (link_i, link_j) in ADJACENT_PAIRS or (link_j, link_i) in ADJACENT_PAIRS:
                continue

            result = fcl.CollisionResult()
            ret = fcl.collide(obj_i, obj_j, request, result)
            if ret > 0:                                                         # Fa el test real amb FCL. ret > 0 → hi ha col·lisió → la guarda a la llista.
                collisions.append((link_i, link_j))

    return len(collisions) > 0, collisions



# ==============================================================================
# CHECK_COLLISIONS_WITH_MARGIN: Igual però considerem col·lisió si la distància és menor que 'margin' (ex: 5 mm)
# ==============================================================================

def check_collisions_with_margin(robot, q, prims_by_link, margin=0.005):

    poses = forward_kinematics(robot, q)

    world_objects = []
    for link_name, link_prims in prims_by_link.items():
        if link_name not in poses:
            continue
        T_link = poses[link_name]
        for p in link_prims:
            T_world = T_link @ p.T_local
            tf = fcl.Transform(T_world[:3, :3], T_world[:3, 3])
            obj = fcl.CollisionObject(p.fcl_object, tf)
            world_objects.append((link_name, obj))

    violations = []
    n = len(world_objects)
    dist_request = fcl.DistanceRequest()

    for i in range(n):
        link_i, obj_i = world_objects[i]
        for j in range(i + 1, n):
            link_j, obj_j = world_objects[j]

            if link_i == link_j:
                continue
            if (link_i, link_j) in ADJACENT_PAIRS or (link_j, link_i) in ADJACENT_PAIRS:
                continue

            dist_result = fcl.DistanceResult()
            dist = fcl.distance(obj_i, obj_j, dist_request, dist_result)

            if dist < margin:                                                   # Si la distancia es més petita que el MERGE. 
                violations.append((link_i, link_j))

    return len(violations) > 0, violations