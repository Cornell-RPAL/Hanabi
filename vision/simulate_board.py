

def id_to_card(id):
    assert id_ < 60
    assert id_ >= 0

    colors = ['green', 'blue', 'red', 'yellow', 'white']
    numbers = {
            0: 1,
            1: 1,
            2: 1,
            3: 2,
            4: 2,
            5: 3,
            6: 3,
            7: 4,
            8: 4,
            9: 5
            }
    
    color = colors[id_ / 10]
    number = numbers.get(id_ % 10)

    return Card(color, number, id)

def detectState_simulate(tags, empty_draw_pile, discard_threshold, hand_threshold, nearnes_threshold):
    res = {"discard": discard, "hand": hand, "board": board, "gripper": gripper}
    for key in res: 
        if res[key]: 
            log([tag.tag_id for tag in res[key]])
            res[key] = id_to_card(tag.tag_id for tag in res[key]
    return res

