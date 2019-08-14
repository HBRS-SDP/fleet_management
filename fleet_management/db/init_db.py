from task_planner.knowledge_base_interface import KnowledgeBaseInterface


def initialize_knowledge_base(kb_database_name):
    kb_interface = KnowledgeBaseInterface(kb_database_name)

    print('[initialize_knowledge_base] Initializing elevators')
    # TODO: Use the actual areas where the elevators are
    elevator_facts = [('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'AMK_B_L-1_C2')]),
                      ('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'AMK_B_L4_C0')])]
    kb_interface.insert_facts(elevator_facts)

    elevator_fluents = [('elevator_floor', [('elevator', 'elevator0')], 100)]
    kb_interface.insert_fluents(elevator_fluents)

    elevator_location_fluents = [('location_floor', [('loc', 'AMK_B_L-1_C2')], -1),
                                 ('location_floor', [('loc', 'AMK_B_L4_C0')], 4)]
    kb_interface.insert_fluents(elevator_location_fluents)
