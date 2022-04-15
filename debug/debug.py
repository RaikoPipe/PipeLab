from data_class.BuildingInstruction import BuildingInstruction
from process_planning.pp_util import get_outgoing_node_pairs, get_outgoing_node_directions

completed_layouts = {
    ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7)): BuildingInstruction(att_set={(0, 3)},
                                                                                          pipe_set={(0, 3)},
                                                                                          fit_set={(0, 7), (0, 0)},
                                                                                          pipe_id=6,
                                                                                          required_fit_positions=(
                                                                                          (0, 0), (0, 7)),
                                                                                          recommended_attachment_pos=(
                                                                                          1, 5), completed=True),
    ((0, 7), (1, 7), (2, 7), (3, 7), (4, 7)): BuildingInstruction(att_set={(3, 7)}, pipe_set={(3, 7)},
                                                                  fit_set={(0, 7), (4, 7)}, pipe_id=3,
                                                                  required_fit_positions=((0, 7), (4, 7)),
                                                                  recommended_attachment_pos=(2, 1), completed=True)}

get_outgoing_node_pairs(completed_layouts)
layout_outgoing_nodes_dict = get_outgoing_node_directions(completed_layouts)

print(get_outgoing_node_directions(completed_layouts))
